"""
cam_bridge_hybrid.py — Synchronous Smooth Transform Bridge
- Mode synchrone CARLA + world.tick() contrôlé à 20Hz
- Suivi strict et absolu des positions GPS (CAMs réels de l'Origin DS7)
- Interpolation linéaire entre les messages CAM pour éliminer les saccades
- Conserve la physique active pour les capteurs (LiDAR, Caméras) et les collisions
"""

import carla
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from etsi_its_cam_msgs.msg import CAM

# Configuration Fréquences
TICK_HZ   = 20
TICK_DT   = 1.0 / TICK_HZ
CAM_HZ    = 10
STEPS     = TICK_HZ // CAM_HZ  # 2 étapes d'interpolation entre chaque CAM

# Référence géographique UPHF (Centre de la carte CARLA)
REF_LAT = 50.318415
REF_LON = 3.510451


def gps_to_carla(lat, lon):
    """WGS84 → coordonnées locales CARLA (mètres, axe Y inversé)."""
    R = 6371000.0
    x =  R * math.cos(math.radians(REF_LAT)) * math.radians(lon - REF_LON)
    y = -R * math.radians(lat - REF_LAT)
    return x, y


class CamBridgeHybrid(Node):
    def __init__(self):
        super().__init__('cam_bridge_hybrid')

        # ── Connexion Client CARLA ──
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(60.0)
        self.world  = self.client.get_world()
        self.amap   = self.world.get_map()

        self.get_logger().info(f'Carte chargée: {self.amap.name}')

        # ── Configuration du Mode Synchrone (Strict) ──
        settings = self.world.get_settings()
        settings.synchronous_mode    = True
        settings.fixed_delta_seconds = TICK_DT
        self.world.apply_settings(settings)
        self.get_logger().info(f'Mode synchrone configuré à {TICK_HZ}Hz (dt={TICK_DT}s)')

        # ── Configuration du Blueprint Véhicule ──
        bp_lib  = self.world.get_blueprint_library()
        self.bp = bp_lib.find('vehicle.tesla.model3')
        self.bp.set_attribute('color', '255,0,0') # Rouge pour le distinguer

        # ── Threading & Gestion d'état ──
        self._spawn_lock = threading.Lock()
        self._z_cache    = {}
        self.vehicles    = {}

        # ROS 2 Multi-threading
        self.cb_group = ReentrantCallbackGroup()

        # Subscription CAM (10Hz)
        self.sub = self.create_subscription(
            CAM, '/its/cam_received', self.cam_callback, 10,
            callback_group=self.cb_group)

        # Timer principal qui cadence CARLA (20Hz)
        self.tick_timer = self.create_timer(
            TICK_DT, self.world_tick,
            callback_group=self.cb_group)

        self.get_logger().info('Bridge Hybride initialisé avec succès.')

    def get_road_z(self, x, y):
        """Récupère l'altitude de la route pour éviter que le véhicule flotte ou s'enfonce."""
        key = (round(x, 1), round(y, 1))
        if key in self._z_cache:
            return self._z_cache[key]
        
        wp = self.amap.get_waypoint(
            carla.Location(x=x, y=y, z=0),
            project_to_road=True,
            lane_type=carla.LaneType.Driving)
        
        z = (wp.transform.location.z + 0.3) if wp else 0.5
        self._z_cache[key] = z
        return z

    @staticmethod
    def decode(msg):
        """Décodage standard ETSI ITS CAM."""
        hfc = msg.cam.cam_parameters \
                  .high_frequency_container \
                  .basic_vehicle_container_high_frequency

        lat = msg.cam.cam_parameters.basic_container \
                  .reference_position.latitude.value / 1e7
        lon = msg.cam.cam_parameters.basic_container \
                  .reference_position.longitude.value / 1e7

        cx, cy = gps_to_carla(lat, lon)
        yaw    = (90.0 - hfc.heading.heading_value.value / 10.0) % 360.0
        speed  = hfc.speed.speed_value.value / 100.0
        sid    = msg.header.station_id.value

        return sid, cx, cy, yaw, speed

    def cam_callback(self, msg):
        """Callback exécuté à la réception d'un message CAM (10Hz)."""
        try:
            sid, cx, cy, yaw, speed = self.decode(msg)
            cz = self.get_road_z(cx, cy)

            with self._spawn_lock:
                already_alive = (sid in self.vehicles and self.vehicles[sid]['actor'].is_alive)
                
                # 1. Premier enregistrement : On spawn l'acteur directement aux coordonnées brutes
                if not already_alive:
                    spawn_t = carla.Transform(
                        carla.Location(x=cx, y=cy, z=cz),
                        carla.Rotation(yaw=yaw))
                    try:
                        actor = self.world.spawn_actor(self.bp, spawn_t)
                        # Crucial: On garde la physique pour que les capteurs (LiDAR/Caméra) fonctionnent !
                        actor.set_simulate_physics(True)
                        actor.set_autopilot(False) # Pas de Traffic Manager !
                        
                        self.vehicles[sid] = {
                            'actor'     : actor,
                            'prev'      : (cx, cy, cz, yaw),
                            'target'    : (cx, cy, cz, yaw),
                            'prev_speed': speed,
                            'speed'     : speed,
                            'step'      : STEPS,
                        }
                        self.get_logger().info(f'[{sid}] Spawné aux coordonnées absolues GPS ({cx:.2f}, {cy:.2f})')
                    except Exception as e:
                        self.get_logger().error(f'Spawn failed: {e}')
                    return

            # 2. Mise à jour des cibles pour l'interpolation (Exécuté à chaque nouvelle CAM)
            vdata = self.vehicles[sid]
            loc   = vdata['actor'].get_location()
            
            # L'ancienne position de départ devient la position actuelle réelle dans CARLA
            vdata['prev']        = (loc.x, loc.y, loc.z, vdata['target'][3])
            vdata['target']      = (cx, cy, cz, yaw)
            vdata['prev_speed']  = vdata['speed']
            vdata['speed']       = speed
            vdata['step']        = 0  # Reset du compteur d'interpolation pour lancer le mouvement vers la cible

        except Exception as e:
            self.get_logger().error(f'cam_callback error: {e}')

    def world_tick(self):
        """Boucle principale synchrone cadencée à 20Hz."""
        with self._spawn_lock:
            for sid, vdata in self.vehicles.items():
                actor = vdata['actor']
                if not actor.is_alive:
                    continue

                step = vdata['step']
                if step >= STEPS:
                    # On a déjà atteint la cible CAM, on attend la suivante sans bouger l'acteur
                    continue

                # Extraire les données de trajectoire
                px, py, pz, pyaw = vdata['prev']
                tx, ty, tz, tyaw = vdata['target']
                prev_speed       = vdata['prev_speed']
                speed            = vdata['speed']

                # t passe de 0.5 à 1.0 sur 2 steps (car TICK_HZ // CAM_HZ = 2)
                t = (step + 1) / STEPS  

                # Interpolation linéaire de la position spatiale
                ix = px + (tx - px) * t
                iy = py + (ty - py) * t
                iz = pz + (tz - pz) * t

                # Interpolation du Heading (prise en compte du passage de 0 à 360°)
                dyaw = ((tyaw - pyaw + 180) % 360) - 180
                iyaw = pyaw + dyaw * t

                # Application stricte de la transformation (Forçage de position)
                actor.set_transform(carla.Transform(
                    carla.Location(x=ix, y=iy, z=iz),
                    carla.Rotation(yaw=iyaw)))

                # Génération de commandes de contrôle de surface purement esthétiques 
                # (Permet l'inclinaison de la suspension et la rotation visuelle des roues)
                ispeed = prev_speed + (speed - prev_speed) * t
                control = carla.VehicleControl()
                control.throttle = min(ispeed / 30.0, 1.0) if ispeed > 0 else 0.0
                control.steer    = max(-1.0, min(1.0, -dyaw / 30.0))
                control.reverse  = ispeed < 0
                actor.apply_control(control)

                vdata['step'] += 1

        # ── AVANCER LE SIMULATEUR D'UN PAQUET DE TEMPS ──
        # Tous les calculs cinématiques ROS 2 sont faits, on dit à CARLA de générer la frame
        self.world.tick()

    def destroy(self):
        """Nettoyage propre à l'arrêt du script."""
        # Désactivation du mode synchrone pour restaurer CARLA à son état normal
        settings = self.world.get_settings()
        settings.synchronous_mode    = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)

        # Destruction de tous les acteurs spawnés
        with self._spawn_lock:
            for sid, vdata in self.vehicles.items():
                a = vdata['actor']
                if a.is_alive:
                    a.apply_control(carla.VehicleControl()) # Stop les contrôles
                    a.destroy()
        self.get_logger().info('Mode asynchrone restauré — Véhicules du jumeau numérique nettoyés.')


def main():
    rclpy.init()
    node = CamBridgeHybrid()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
