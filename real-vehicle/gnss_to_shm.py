"""
gnss_to_shm.py
Souscrit aux topics /vehicle{n}/gnss et /vehicle{n}/imu
et écrit les données dans /dev/shm/agent_{n-1} pour CarlaCaService Artery.

Remplace la lecture directe depuis l'API CARLA dans art_spawn.py.
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import NavSatFix, Imu
import struct
import math
import time

HERO_STATE_FORMAT = 'ddddddd'
SHM_SIZE = struct.calcsize(HERO_STATE_FORMAT)

VEHICLES = [1, 2, 3]


class GnssToShm(Node):
    def __init__(self):
        super().__init__('gnss_to_shm')

        # État courant pour chaque véhicule
        self._state = {}
        for idx in VEHICLES:
            self._state[idx] = {
                'lat': 0.0, 'lon': 0.0,
                'heading': 0.0, 'speed': 0.0,
                'accel': 0.0, 'yaw_rate': 0.0,
                'prev_lat': None, 'prev_lon': None, 'prev_time': None,
            }
            # Initialiser SHM à zéro
            shm_path = f'/dev/shm/agent_{idx - 1}'
            try:
                with open(shm_path, 'r+b') as f:
                    f.write(b'\x00' * SHM_SIZE)
            except FileNotFoundError:
                with open(shm_path, 'wb') as f:
                    f.write(b'\x00' * SHM_SIZE)
            self.get_logger().info(f'SHM initialisé : {shm_path}')

        # Subscribers GNSS
        for idx in VEHICLES:
            self.create_subscription(
                NavSatFix,
                f'/vehicle{idx}/gnss',
                lambda msg, i=idx: self._on_gnss(msg, i),
                10)
            self.get_logger().info(f'Souscrit à /vehicle{idx}/gnss')

        # Subscribers IMU
        for idx in VEHICLES:
            self.create_subscription(
                Imu,
                f'/vehicle{idx}/imu',
                lambda msg, i=idx: self._on_imu(msg, i),
                10)
            self.get_logger().info(f'Souscrit à /vehicle{idx}/imu')

        self.get_logger().info('gnss_to_shm démarré')

    def _on_gnss(self, msg: NavSatFix, idx: int):
        s = self._state[idx]
        lat = msg.latitude
        lon = msg.longitude
        now = time.time()

        # Calcul vitesse depuis déplacement GPS
        if s['prev_lat'] is not None and s['prev_time'] is not None:
            dt = now - s['prev_time']
            if dt > 0.01:
                R = 6371000.0
                dx = R * math.cos(math.radians(s['prev_lat'])) \
                       * math.radians(lon - s['prev_lon'])
                dy = R * math.radians(lat - s['prev_lat'])
                s['speed'] = math.sqrt(dx**2 + dy**2) / dt

                # Calcul heading depuis déplacement
                if abs(dx) > 0.001 or abs(dy) > 0.001:
                    heading_rad = math.atan2(dx, dy)
                    s['heading'] = (math.degrees(heading_rad)) % 360.0

        s['lat'] = lat
        s['lon'] = lon
        s['prev_lat'] = lat
        s['prev_lon'] = lon
        s['prev_time'] = now

        self._write_shm(idx)

    def _on_imu(self, msg: Imu, idx: int):
        s = self._state[idx]

        # Accélération longitudinale
        s['accel'] = msg.linear_acceleration.x

        # Yaw rate depuis angular velocity z
        s['yaw_rate'] = msg.angular_velocity.z

        # Heading depuis quaternion
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        s['heading'] = (90.0 - yaw) % 360.0

        self._write_shm(idx)

    def _write_shm(self, idx: int):
        s = self._state[idx]
        shm_path = f'/dev/shm/agent_{idx - 1}'
        try:
            data = struct.pack(
                HERO_STATE_FORMAT,
                s['lat'], s['lon'], s['heading'], s['speed'],
                s['accel'], s['yaw_rate'], time.time()
            )
            with open(shm_path, 'r+b') as f:
                f.write(data)
        except Exception as e:
            self.get_logger().warn(f'SHM write error [{shm_path}]: {e}')


def main():
    rclpy.init()
    node = GnssToShm()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
