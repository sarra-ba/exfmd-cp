"""
cam_bridge.py — Synchronous TM Bridge
- Mode synchrone CARLA + world.tick() à 20Hz
- TM pilote la voiture sur la route via le xodr
- CAMs réels du véhicule Origin DS7 → conversion GPS→CARLA
- Même approche que generate_traffic.py
"""
import carla
import math
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from etsi_its_cam_msgs.msg import CAM

TM_PORT  = 8002
TICK_HZ  = 20
TICK_DT  = 1.0 / TICK_HZ

# Référence géographique UPHF (centre de la carte CARLA)
REF_LAT  = 50.31841
REF_LON  = 3.51045


def gps_to_carla(lat, lon):
    """WGS84 → coordonnées locales CARLA (mètres, axe Y inversé)."""
    R = 6371000.0
    x =  R * math.cos(math.radians(REF_LAT)) * math.radians(lon - REF_LON)
    y = -R * math.radians(lat - REF_LAT)
    return x, y


class CamBridge(Node):
    def __init__(self):
        super().__init__('cam_bridge')

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(60.0)
        self.world  = self.client.get_world()
        self.amap   = self.world.get_map()
        self.tm     = self.client.get_trafficmanager(TM_PORT)

        self.get_logger().info(f'Carte: {self.world.get_map().name}')

        # ── Mode synchrone ──
        settings = self.world.get_settings()
        settings.synchronous_mode    = True
        settings.fixed_delta_seconds = TICK_DT
        self.world.apply_settings(settings)
        self.tm.set_synchronous_mode(True)
        self.get_logger().info(f'Mode synchrone activé — {TICK_HZ}Hz')

        bp_lib  = self.world.get_blueprint_library()
        self.bp = bp_lib.find('vehicle.tesla.model3')
        self.bp.set_attribute('color', '255,0,0')

        self._spawn_lock = threading.Lock()
        self.vehicles    = {}

        self.cb_group = ReentrantCallbackGroup()

        self.sub = self.create_subscription(
            CAM, '/its/cam_received', self.cam_callback, 10,
            callback_group=self.cb_group)

        self.tick_timer = self.create_timer(
            TICK_DT, self.world_tick,
            callback_group=self.cb_group)

        self.get_logger().info(
            f'Bridge démarré — synchrone {TICK_HZ}Hz / TM:{TM_PORT} '
            f'— mode GPS réel (REF {REF_LAT}, {REF_LON})')

    # ── Décodage CAM réel (GPS standard ETSI) ───────────────────────────────
    @staticmethod
    def decode(msg):
        hfc = msg.cam.cam_parameters \
                  .high_frequency_container \
                  .basic_vehicle_container_high_frequency

        # Position GPS réelle (×1e7 → degrés)
        lat = msg.cam.cam_parameters.basic_container \
                  .reference_position.latitude.value / 1e7
        lon = msg.cam.cam_parameters.basic_container \
                  .reference_position.longitude.value / 1e7

        # Conversion GPS → CARLA
        cx, cy = gps_to_carla(lat, lon)

        # Heading ETSI (1/10°, Nord=0 CW) → CARLA yaw (Est=0 CCW)
        yaw = (90.0 - hfc.heading.heading_value.value / 10.0) % 360.0

        # Vitesse ETSI (1/100 m/s) → m/s
        speed = hfc.speed.speed_value.value / 100.0

        sid = msg.header.station_id.value

        return sid, cx, cy, yaw, speed

    # ── Z depuis waypoint routier ────────────────────────────────────────────
    def get_road_z(self, x, y):
        wp = self.amap.get_waypoint(
            carla.Location(x=x, y=y, z=0),
            project_to_road=True,
            lane_type=carla.LaneType.Driving)
        return (wp.transform.location.z + 0.3) if wp else 0.5

    # ── world.tick() à 20Hz ─────────────────────────────────────────────────
    def world_tick(self):
        self.world.tick()

    # ── Callback CAM ────────────────────────────────────────────────────────
    def cam_callback(self, msg):
        try:
            sid, cx, cy, yaw, speed = self.decode(msg)
            cz = self.get_road_z(cx, cy)

            with self._spawn_lock:
                already_alive = (sid in self.vehicles and
                                 self.vehicles[sid]['actor'].is_alive)
                if not already_alive:
                    wp_spawn = self.amap.get_waypoint(
                        carla.Location(x=cx, y=cy, z=0),
                        project_to_road=True,
                        lane_type=carla.LaneType.Driving)
                    if wp_spawn:
                        spawn_t = wp_spawn.transform
                        spawn_t.location.z = cz
                        spawn_t.rotation.yaw = yaw
                    else:
                        spawn_t = carla.Transform(
                            carla.Location(x=cx, y=cy, z=cz),
                            carla.Rotation(yaw=yaw))
                    try:
                        actor = self.world.spawn_actor(self.bp, spawn_t)
                        actor.set_autopilot(True, TM_PORT)
                        self.tm.ignore_lights_percentage(actor, 100)
                        self.tm.ignore_signs_percentage(actor, 100)
                        self.tm.auto_lane_change(actor, False)
                        self.tm.set_desired_speed(actor, speed * 3.6)

                        self.vehicles[sid] = {
                            'actor'      : actor,
                            'cam_buffer' : [carla.Location(x=cx, y=cy, z=cz)],
                            'speed'      : speed,
                        }
                        self.get_logger().info(
                            f'[{sid}] Spawné ({cx:.2f},{cy:.2f},{cz:.2f}) '
                            f'yaw={yaw:.1f}° {speed*3.6:.1f}km/h')
                    except Exception as e:
                        self.get_logger().error(f'Spawn failed: {e}')
                    return

            # ── Update path et vitesse ──
            vdata = self.vehicles[sid]
            vdata['speed'] = speed

            vdata['cam_buffer'].append(carla.Location(x=cx, y=cy, z=cz))
            if len(vdata['cam_buffer']) > 5:
                vdata['cam_buffer'] = vdata['cam_buffer'][-5:]

            self.tm.set_path(vdata['actor'], vdata['cam_buffer'], True)
            self.tm.set_desired_speed(vdata['actor'], speed * 3.6)

            self.get_logger().debug(
                f'[{sid}] GPS→CARLA ({cx:.2f},{cy:.2f}) '
                f'yaw={yaw:.1f}° spd={speed:.2f}m/s')

        except Exception as e:
            self.get_logger().error(f'cam_callback: {e}')

    # ── Cleanup ─────────────────────────────────────────────────────────────
    def destroy(self):
        settings = self.world.get_settings()
        settings.synchronous_mode    = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)

        for vdata in self.vehicles.values():
            a = vdata['actor']
            if a.is_alive:
                a.set_autopilot(False, TM_PORT)
                a.destroy()
        self.get_logger().info('Mode asynchrone restauré — véhicules détruits')


def main():
    rclpy.init()
    node = CamBridge()
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
