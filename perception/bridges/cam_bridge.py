"""
cam_bridge.py — Synchronous TM Bridge
- Mode synchrone CARLA + world.tick() à 20Hz
- TM pilote la voiture sur la route via le xodr
- CAMs fournissent le path et la vitesse au TM
- Même approche que generate_traffic.py
"""
import carla
import math
import threading
import csv
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from etsi_its_cam_msgs.msg import CAM

TM_PORT      = 8000
TICK_HZ      = 20       # fréquence simulation
TICK_DT      = 1.0 / TICK_HZ
CSV_PATH     = '/home/syfra/workspace/moad_workspace/uphf_trajectory.csv'


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
        settings.synchronous_mode   = True
        settings.fixed_delta_seconds = TICK_DT
        self.world.apply_settings(settings)
        self.tm.set_synchronous_mode(True)
        self.get_logger().info(f'Mode synchrone activé — {TICK_HZ}Hz')

        bp_lib  = self.world.get_blueprint_library()
        self.bp = bp_lib.find('vehicle.tesla.model3')
        self.bp.set_attribute('color', '255,0,0')

        self._z_cache    = {}
        self._spawn_lock = threading.Lock()
        self.vehicles    = {}

        # Préchauffer cache Z
        self.get_logger().info('Préconstruction cache Z...')
        self._preheat_z_cache()
        self.get_logger().info(f'Cache Z prêt : {len(self._z_cache)} entrées')

        self.cb_group = ReentrantCallbackGroup()

        self.sub = self.create_subscription(
            CAM, '/its/cam_received', self.cam_callback, 10,
            callback_group=self.cb_group)

        # Timer world.tick() à 20Hz
        self.tick_timer = self.create_timer(
            TICK_DT, self.world_tick,
            callback_group=self.cb_group)

        self.get_logger().info(
            f'Bridge démarré — synchrone {TICK_HZ}Hz / TM:{TM_PORT}')

    # ── Cache Z ─────────────────────────────────────────────────────────────
    def _preheat_z_cache(self):
        try:
            count = 0
            with open(CSV_PATH) as f:
                reader = csv.DictReader(f)
                for row in reader:
                    self.get_road_z(float(row['carla_x']), float(row['carla_y']))
                    count += 1
                    if count % 5000 == 0:
                        self.get_logger().info(f'Cache Z : {count} points...')
        except Exception as e:
            self.get_logger().warn(f'Preheat échoué: {e}')

    def get_road_z(self, x, y):
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

    # ── Décodage CAM ────────────────────────────────────────────────────────
    @staticmethod
    def decode(msg):
        hfc   = msg.cam.cam_parameters \
                    .high_frequency_container \
                    .basic_vehicle_container_high_frequency
        cx    = msg.cam.cam_parameters.basic_container \
                    .reference_position.altitude.altitude_value.value / 100.0
        cy    = hfc.yaw_rate.yaw_rate_value.value / 100.0
        yaw   = (90.0 - hfc.heading.heading_value.value / 10.0) % 360.0
        speed = hfc.speed.speed_value.value / 100.0
        sid   = msg.header.station_id.value
        return sid, cx, cy, yaw, speed

    # ── world.tick() à 20Hz ─────────────────────────────────────────────────
    def world_tick(self):
        self.world.tick()

    # ── Callback CAM (10Hz) ─────────────────────────────────────────────────
    def cam_callback(self, msg):
        try:
            sid, cx, cy, yaw, speed = self.decode(msg)
            cz = self.get_road_z(cx, cy)

            with self._spawn_lock:
                already_alive = (sid in self.vehicles and
                                 self.vehicles[sid]['actor'].is_alive)
                if not already_alive:
                    # Spawner sur le waypoint routier
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
                            'actor'       : actor,
                            'target_x'    : cx,
                            'target_y'    : cy,
                            'target_z'    : cz,
                            'target_yaw'  : yaw,
                            'target_speed': speed,
                            'cam_buffer'  : [carla.Location(x=cx, y=cy, z=cz)],
                        }
                        self.get_logger().info(
                            f'[{sid}] Spawné ({cx:.2f},{cy:.2f}) '
                            f'yaw={yaw:.1f}° {speed*3.6:.1f}km/h — TM ON')
                    except Exception as e:
                        self.get_logger().error(f'Spawn failed: {e}')
                    return

            # ── Update path et vitesse depuis le CAM ──
            vdata = self.vehicles[sid]
            actor = vdata['actor']

            vdata['target_x']     = cx
            vdata['target_y']     = cy
            vdata['target_z']     = cz
            vdata['target_yaw']   = yaw
            vdata['target_speed'] = speed

            # Accumuler positions dans le buffer (5 points lookahead)
            vdata['cam_buffer'].append(carla.Location(x=cx, y=cy, z=cz))
            if len(vdata['cam_buffer']) > 5:
                vdata['cam_buffer'] = vdata['cam_buffer'][-5:]

            # TM suit ce path sur la route
            self.tm.set_path(actor, vdata['cam_buffer'], True)
            self.tm.set_desired_speed(actor, speed * 3.6)

        except Exception as e:
            self.get_logger().error(f'cam_callback: {e}')

    # ── Cleanup ─────────────────────────────────────────────────────────────
    def destroy(self):
        # Désactiver mode synchrone
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
