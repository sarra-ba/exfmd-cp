"""
art_spawn.py
Spawne 3 véhicules (Tesla Model3) avec TM autopilot en mode synchrone.
Lit les coordonnées GPS depuis le GNSS sensor CARLA (data.latitude/longitude).
Ecrit les positions dans /dev/shm/agent_{idx} pour CarlaCaService Artery.
"""
import carla
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
import math
import time
import random
import struct
import os

from pyproj import Transformer
_transformer_carla_to_gps = Transformer.from_crs(
    '+proj=tmerc +lat_0=50.3184145 +lon_0=3.510451 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs',
    'EPSG:4326', always_xy=True)

def carla_to_gps(x, y):
    lon, lat = _transformer_carla_to_gps.transform(x, -y)
    return lat, lon

TM_PORT  = 8002
GNSS_HZ  = 10.0
IMU_HZ   = 10.0

VEHICLES_CONFIG = [
    (1, 1000000001, '0,0,255'),
    (2, 1000000002, '0,255,0'),
    (3, 1000000003, '255,165,0'),
]

HERO_STATE_FORMAT = 'ddddddd'
SHM_SIZE = struct.calcsize(HERO_STATE_FORMAT)


class VehicleAgent:
    def __init__(self, node, world, amap, tm, idx, station_id, color):
        self.node       = node
        self.world      = world
        self.amap       = amap
        self.tm         = tm
        self.idx        = idx
        self.station_id = station_id
        self.color      = color

        self.vehicle     = None
        self.gnss_sensor = None

        self._lat      = None
        self._lon      = None
        self._alt      = 0.0
        self._speed_ms = 0.0
        self._prev_lat  = None
        self._prev_lon  = None
        self._prev_time = None

        self._shm_path = f'/dev/shm/agent_{idx - 1}'

        try:
            with open(self._shm_path, 'wb') as f:
                f.write(b'\x00' * SHM_SIZE)
            node.get_logger().info(f'SHM initialisé : {self._shm_path}')
        except Exception as e:
            node.get_logger().warn(f'SHM init error: {e}')

        self.gnss_pub = node.create_publisher(NavSatFix, f'/vehicle{idx}/gnss', 10)
        self.imu_pub  = node.create_publisher(Imu,       f'/vehicle{idx}/imu',  10)

        node.get_logger().info(f'Agent {idx} (station_id={station_id}) initialisé')

    def write_shm(self, lat, lon, heading, speed, accel, yaw_rate):
        try:
            data = struct.pack(HERO_STATE_FORMAT,
                               lat, lon, heading, speed,
                               accel, yaw_rate, time.time())
            with open(self._shm_path, 'r+b') as f:
                f.write(data)
        except Exception as e:
            self.node.get_logger().warn(f'SHM write error [{self._shm_path}]: {e}')

    def spawn(self, spawn_points):
        bp_lib = self.world.get_blueprint_library()
        bp = bp_lib.find('vehicle.tesla.model3')
        bp.set_attribute('color', self.color)
        spawn_t = random.choice(spawn_points)
        try:
            self.vehicle = self.world.spawn_actor(bp, spawn_t)
            self.vehicle.set_autopilot(True, TM_PORT)
            self.tm.ignore_lights_percentage(self.vehicle, 100)
            self.tm.ignore_signs_percentage(self.vehicle, 100)
            self.tm.auto_lane_change(self.vehicle, False)
            self.tm.vehicle_percentage_speed_difference(self.vehicle, 0)
            self.node.get_logger().info(
                f'Véhicule {self.idx} spawné (carla_id={self.vehicle.id}) '
                f'— station_id={self.station_id}')
            self._attach_gnss()
            return True
        except Exception as e:
            self.node.get_logger().error(f'Spawn véhicule {self.idx} failed: {e}')
            return False

    def _attach_gnss(self):
        bp_lib  = self.world.get_blueprint_library()
        gnss_bp = bp_lib.find('sensor.other.gnss')
        gnss_bp.set_attribute('sensor_tick', str(1.0 / GNSS_HZ))
        transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.8))
        self.gnss_sensor = self.world.spawn_actor(
            gnss_bp, transform, attach_to=self.vehicle)
        self.gnss_sensor.listen(self._on_gnss)

    def _on_gnss(self, data):
        if not self.vehicle or not self.vehicle.is_alive:
            return

        now = self.node.get_clock().now().nanoseconds * 1e-9
        # Conversion CARLA → GPS via pyproj (Y inversé)
        loc = self.vehicle.get_location()
        lat, lon = carla_to_gps(loc.x, loc.y)
        alt = data.altitude

        # Calcul vitesse depuis déplacement GPS
        if self._prev_lat is not None and self._prev_time is not None:
            dt = now - self._prev_time
            if dt > 0.01:
                R  = 6371000.0
                dx = R * math.cos(math.radians(self._prev_lat)) \
                       * math.radians(lon - self._prev_lon)
                dy = R * math.radians(lat - self._prev_lat)
                self._speed_ms = math.sqrt(dx**2 + dy**2) / dt

        self._lat      = lat
        self._lon      = lon
        self._alt      = alt
        self._prev_lat  = lat
        self._prev_lon  = lon
        self._prev_time = now

        # Écrire dans SHM pour Artery
        try:
            t             = self.vehicle.get_transform()
            ang_vel       = self.vehicle.get_angular_velocity()
            accel_v       = self.vehicle.get_acceleration()
            heading_geo   = (90.0 - t.rotation.yaw) % 360.0
            yaw_rate_rads = math.radians(ang_vel.z)
            self.write_shm(lat, lon, heading_geo, self._speed_ms,
                           accel_v.x, yaw_rate_rads)
        except Exception as e:
            self.node.get_logger().warn(f'SHM update error: {e}')

        # Publier NavSatFix ROS2
        msg = NavSatFix()
        msg.header.stamp    = self.node.get_clock().now().to_msg()
        msg.header.frame_id = f'vehicle{self.idx}/gnss'
        msg.status.status   = NavSatStatus.STATUS_FIX
        msg.status.service  = NavSatStatus.SERVICE_GPS
        msg.latitude        = lat
        msg.longitude       = lon
        msg.altitude        = alt
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance = [1.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0,
                                   0.0, 0.0, 4.0]
        self.gnss_pub.publish(msg)

    def publish_imu(self):
        if not self.vehicle or not self.vehicle.is_alive:
            return
        try:
            t       = self.vehicle.get_transform()
            ang_vel = self.vehicle.get_angular_velocity()
            accel   = self.vehicle.get_acceleration()

            yaw_rad   = math.radians(t.rotation.yaw)
            pitch_rad = math.radians(t.rotation.pitch)
            roll_rad  = math.radians(t.rotation.roll)

            cy = math.cos(yaw_rad   * 0.5)
            sy = math.sin(yaw_rad   * 0.5)
            cp = math.cos(pitch_rad * 0.5)
            sp = math.sin(pitch_rad * 0.5)
            cr = math.cos(roll_rad  * 0.5)
            sr = math.sin(roll_rad  * 0.5)

            msg = Imu()
            msg.header.stamp    = self.node.get_clock().now().to_msg()
            msg.header.frame_id = f'vehicle{self.idx}/imu'
            msg.orientation.w = cr * cp * cy + sr * sp * sy
            msg.orientation.x = sr * cp * cy - cr * sp * sy
            msg.orientation.y = cr * sp * cy + sr * cp * sy
            msg.orientation.z = cr * cp * sy - sr * sp * cy
            msg.angular_velocity.x = math.radians(ang_vel.x)
            msg.angular_velocity.y = math.radians(ang_vel.y)
            msg.angular_velocity.z = math.radians(ang_vel.z)
            msg.linear_acceleration.x = accel.x
            msg.linear_acceleration.y = accel.y
            msg.linear_acceleration.z = accel.z
            self.imu_pub.publish(msg)
        except Exception as e:
            self.node.get_logger().warn(f'IMU {self.idx} error: {e}')

    def destroy(self):
        if self.gnss_sensor and self.gnss_sensor.is_alive:
            self.gnss_sensor.stop()
        time.sleep(0.3)
        if self.gnss_sensor and self.gnss_sensor.is_alive:
            self.gnss_sensor.destroy()
        if self.vehicle and self.vehicle.is_alive:
            self.vehicle.set_autopilot(False, TM_PORT)
            self.vehicle.destroy()
        try:
            os.remove(self._shm_path)
        except:
            pass
        self.node.get_logger().info(f'Véhicule {self.idx} détruit proprement')


class MultiVehicleSpawner(Node):
    def __init__(self):
        super().__init__('multi_vehicle_spawner')

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(60.0)
        self.world  = self.client.get_world()
        self.amap   = self.world.get_map()
        self.tm     = self.client.get_trafficmanager(TM_PORT)

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)
        self.tm.set_synchronous_mode(True)
        self.tm.set_global_distance_to_leading_vehicle(5.0)
        self.tm.set_respawn_dormant_vehicles(False)

        self.get_logger().info(f'Carte: {self.world.get_map().name}')

        self.cb_group = ReentrantCallbackGroup()
        self.agents   = []

        spawn_points = self.amap.get_spawn_points()
        available    = spawn_points[1:]

        for idx, station_id, color in VEHICLES_CONFIG:
            agent = VehicleAgent(
                self, self.world, self.amap, self.tm,
                idx, station_id, color)
            if agent.spawn(available):
                self.agents.append(agent)
            time.sleep(0.5)

        self.get_logger().info(f'{len(self.agents)} véhicules spawned')

        self.create_timer(1.0 / IMU_HZ, self._publish_all_imu,
                          callback_group=self.cb_group)

    def _publish_all_imu(self):
        self.world.tick()
        for agent in self.agents:
            agent.publish_imu()

    def destroy(self):
        for agent in self.agents:
            agent.destroy()
        self.get_logger().info('Tous les véhicules détruits')


def main():
    rclpy.init()
    node = MultiVehicleSpawner()
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
