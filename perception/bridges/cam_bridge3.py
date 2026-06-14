"""
cam_bridge2.py — Smooth Transform Bridge
- Souscrit à /its/cam_received (vrais CAMs du cube C-ITS)
- Décode lat/lon GPS → coordonnées CARLA via gps_to_carla()
- set_transform() interpolé à 20Hz → suit fidèlement la trajectoire réelle
- set_simulate_physics(True) → LiDAR, caméras, collisions actifs
- apply_control() → roues tournent, suspension active
"""
import carla
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from etsi_its_cam_msgs.msg import CAM

TM_PORT   = 8001
INTERP_HZ = 20
CAM_HZ    = 10
STEPS     = INTERP_HZ // CAM_HZ  # 2 steps entre deux CAMs

REF_LAT = 50.318415
REF_LON = 3.510451


def gps_to_carla(lat, lon):
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

        self.get_logger().info(f'Carte: {self.world.get_map().name}')

        bp_lib  = self.world.get_blueprint_library()
        self.bp = bp_lib.find('vehicle.tesla.model3')
        self.bp.set_attribute('color', '255,0,0')

        self._spawn_lock = threading.Lock()
        self._z_cache    = {}

        # {station_id: {
        #   'actor'     : carla.Actor,
        #   'prev'      : (x, y, z, yaw),
        #   'target'    : (x, y, z, yaw),
        #   'prev_speed': float,
        #   'speed'     : float,
        #   'step'      : int,
        # }}
        self.vehicles = {}

        self.cb_group = ReentrantCallbackGroup()

        self.sub = self.create_subscription(
            CAM, '/its/cam_received', self.cam_callback, 10,
            callback_group=self.cb_group)

        self.interp_timer = self.create_timer(
            1.0 / INTERP_HZ, self.interp_tick,
            callback_group=self.cb_group)

        self.get_logger().info(
            f'Bridge démarré — {INTERP_HZ}Hz interp / {CAM_HZ}Hz CAM '
            f'— REF ({REF_LAT}, {REF_LON})')

    # ── Z depuis waypoint routier (caché) ────────────────────────────────────
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

    # ── Callback CAM (10Hz) ─────────────────────────────────────────────────
    def cam_callback(self, msg):
        try:
            sid, cx, cy, yaw, speed = self.decode(msg)
            cz = self.get_road_z(cx, cy)

            with self._spawn_lock:
                already_alive = (sid in self.vehicles and
                                 self.vehicles[sid]['actor'].is_alive)
                if not already_alive:
                    spawn_t = carla.Transform(
                        carla.Location(x=cx, y=cy, z=cz),
                        carla.Rotation(yaw=yaw))
                    try:
                        actor = self.world.spawn_actor(self.bp, spawn_t)
                        actor.set_simulate_physics(True)
                        actor.set_autopilot(False)
                        self.vehicles[sid] = {
                            'actor'     : actor,
                            'prev'      : (cx, cy, cz, yaw),
                            'target'    : (cx, cy, cz, yaw),
                            'prev_speed': speed,
                            'speed'     : speed,
                            'step'      : STEPS,
                        }
                        self.get_logger().info(
                            f'[{sid}] Spawné ({cx:.2f},{cy:.2f},{cz:.2f}) '
                            f'yaw={yaw:.1f}°')
                    except Exception as e:
                        self.get_logger().error(f'Spawn failed: {e}')
                    return

            # ── Mettre à jour prev/target ──
            vdata = self.vehicles[sid]
            loc   = vdata['actor'].get_location()
            vdata['prev']       = (loc.x, loc.y, loc.z, vdata['target'][3])
            vdata['target']     = (cx, cy, cz, yaw)
            vdata['prev_speed'] = vdata['speed']
            vdata['speed']      = speed
            vdata['step']       = 0

        except Exception as e:
            self.get_logger().error(f'cam_callback: {e}')

    # ── Interpolation à 20Hz ────────────────────────────────────────────────
    def interp_tick(self):
        for sid, vdata in self.vehicles.items():
            actor = vdata['actor']
            if not actor.is_alive:
                continue

            step = vdata['step']
            if step >= STEPS:
                continue

            px, py, pz, pyaw = vdata['prev']
            tx, ty, tz, tyaw = vdata['target']
            prev_speed        = vdata['prev_speed']
            speed             = vdata['speed']

            t = (step + 1) / STEPS  # ∈ (0, 1]

            ix = px + (tx - px) * t
            iy = py + (ty - py) * t
            iz = pz + (tz - pz) * t

            dyaw = ((tyaw - pyaw + 180) % 360) - 180
            iyaw = pyaw + dyaw * t

            ispeed = prev_speed + (speed - prev_speed) * t

            # set_transform → position exacte
            actor.set_transform(carla.Transform(
                carla.Location(x=ix, y=iy, z=iz),
                carla.Rotation(yaw=iyaw)))

            # apply_control → roues + suspension réalistes
            MAX_SPEED = 30.0
            if ispeed > prev_speed:
                throttle = min(ispeed / MAX_SPEED, 1.0)
                brake    = 0.0
            elif ispeed < prev_speed - 0.2:
                throttle = 0.0
                brake    = min((prev_speed - ispeed) / MAX_SPEED, 1.0)
            else:
                throttle = min(ispeed / MAX_SPEED, 1.0)
                brake    = 0.0

            dyaw_ctrl = ((tyaw - pyaw + 180) % 360) - 180
            steer     = max(-1.0, min(1.0, -dyaw_ctrl / 30.0))

            control            = carla.VehicleControl()
            control.throttle   = throttle
            control.steer      = steer
            control.brake      = brake
            control.hand_brake = False
            control.reverse    = ispeed < 0
            actor.apply_control(control)

            vdata['step'] += 1

    # ── Cleanup ─────────────────────────────────────────────────────────────
    def destroy(self):
        for vdata in self.vehicles.values():
            a = vdata['actor']
            if a.is_alive:
                a.apply_control(carla.VehicleControl())
                a.destroy()
        self.get_logger().info('Véhicules détruits')


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
