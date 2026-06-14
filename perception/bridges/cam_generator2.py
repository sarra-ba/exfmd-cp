"""
cam_generator_v2.py
Souscrit aux topics /vehicle2/gnss et /vehicle2/imu
et génère des CAMs ETSI conformes à 10 Hz sur /its/cam/1000000001.

Les champs CAM remplis :
  reference_position.latitude/longitude  ← GNSS
  heading.heading_value                  ← IMU (quaternion → yaw → ETSI)
  speed.speed_value                      ← calculé depuis positions GNSS successives
  yaw_rate.yaw_rate_value                ← IMU (angular_velocity.z)
  longitudinal_acceleration              ← IMU (linear_acceleration.x)
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import NavSatFix, Imu
from etsi_its_cam_msgs.msg import CAM
import math

STATION_ID  = 1000000001
CAM_TOPIC   = f'/its/cam/v{STATION_ID}'
CAM_HZ      = 10.0

# Constantes véhicule (Tesla Model3)
VEHICLE_LENGTH = 47   # 4.7m en 1/10 dm
VEHICLE_WIDTH  = 19   # 1.9m en 1/10 dm

# Référence géographique UPHF (pour conversion GPS → ETSI)
REF_LAT = 50.31841
REF_LON = 3.51045


def quaternion_to_yaw(ox, oy, oz, ow):
    """Quaternion ROS2 → yaw en degrés (géographique, Nord=0 CW)."""
    # Quaternion → yaw Euler (axe Z, convention ROS)
    siny_cosp = 2.0 * (ow * oz + ox * oy)
    cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    yaw_deg = math.degrees(yaw_rad)

    # CARLA yaw (Est=0, CCW) → heading géographique (Nord=0, CW)
    heading_geo = (90.0 - yaw_deg) % 360.0
    return heading_geo


class CamGeneratorV2(Node):
    def __init__(self):
        super().__init__('cam_generator_v2')

        self.cb_group = ReentrantCallbackGroup()

        # Dernières valeurs reçues des capteurs
        self._lat      = None
        self._lon      = None
        self._alt      = None
        self._heading  = 0.0    # degrés géo (Nord=0 CW)
        self._yaw_rate = 0.0    # rad/s
        self._accel_x  = 0.0   # m/s² longitudinal

        # Pour calcul vitesse depuis GNSS
        self._prev_lat  = None
        self._prev_lon  = None
        self._prev_time = None
        self._speed_ms  = 0.0  # m/s

        # Publisher CAM
        self.cam_pub = self.create_publisher(
            CAM, CAM_TOPIC, 10)

        # Souscriptions capteurs
        self.create_subscription(
            NavSatFix, '/vehicle2/gnss',
            self._on_gnss, 10,
            callback_group=self.cb_group)

        self.create_subscription(
            Imu, '/vehicle2/imu',
            self._on_imu, 10,
            callback_group=self.cb_group)

        # Timer publication CAM à 10 Hz
        self.create_timer(
            1.0 / CAM_HZ, self._publish_cam,
            callback_group=self.cb_group)

        self.get_logger().info(
            f'CAM Generator V2 démarré — station_id={STATION_ID} '
            f'→ {CAM_TOPIC}')

    # ── Callbacks capteurs ───────────────────────────────────────────────────
    def _on_gnss(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9

        self._lat = msg.latitude
        self._lon = msg.longitude
        self._alt = msg.altitude

        # Calcul vitesse depuis déplacement GNSS
        if self._prev_lat is not None and self._prev_time is not None:
            dt = now - self._prev_time
            if dt > 0.01:
                R  = 6371000.0
                dx = R * math.cos(math.radians(self._prev_lat)) \
                       * math.radians(self._lon - self._prev_lon)
                dy = R * math.radians(self._lat - self._prev_lat)
                dist = math.sqrt(dx**2 + dy**2)
                self._speed_ms = dist / dt

        self._prev_lat  = self._lat
        self._prev_lon  = self._lon
        self._prev_time = now

    def _on_imu(self, msg):
        # Heading depuis quaternion
        self._heading = quaternion_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w)

        # Yaw rate (rad/s → ETSI 1/100 deg/s)
        self._yaw_rate = msg.angular_velocity.z

        # Accélération longitudinale (m/s²)
        self._accel_x = msg.linear_acceleration.x

    # ── Publication CAM ──────────────────────────────────────────────────────
    def _publish_cam(self):
        if self._lat is None:
            return   # GNSS pas encore reçu

        cam = CAM()

        # ── Header ──
        cam.header.protocol_version = 2
        cam.header.message_id       = 2
        cam.header.station_id.value = STATION_ID

        # ── Basic container ──
        cam.cam.cam_parameters.basic_container \
            .station_type.value = 5   # passenger car

        # Position GPS en format ETSI (×1e7)
        cam.cam.cam_parameters.basic_container \
            .reference_position.latitude.value  = int(self._lat * 1e7)
        cam.cam.cam_parameters.basic_container \
            .reference_position.longitude.value = int(self._lon * 1e7)

        # Altitude en cm (ETSI : valeur en 1/100 m, référence WGS84)
        alt_cm = int(self._alt * 100.0)
        alt_cm = max(-100000, min(800001, alt_cm))
        cam.cam.cam_parameters.basic_container \
            .reference_position.altitude.altitude_value.value = alt_cm

        # ── High frequency container ──
        hfc = cam.cam.cam_parameters.high_frequency_container \
                  .basic_vehicle_container_high_frequency

        # Heading ETSI : Nord=0 CW, 1/10° [0..3600]
        heading_etsi = int(round(self._heading * 10.0)) % 3601
        hfc.heading.heading_value.value    = heading_etsi
        hfc.heading.heading_confidence.value = 126   # confiance ~3.6°

        # Vitesse ETSI : 1/100 m/s [0..16383]
        speed_etsi = int(round(self._speed_ms * 100.0))
        speed_etsi = max(0, min(16383, speed_etsi))
        hfc.speed.speed_value.value    = speed_etsi
        hfc.speed.speed_confidence.value = 52   # confiance ~0.52 m/s

        # Direction de conduite (0=forward, 1=backward, 2=unavailable)
        hfc.drive_direction.value = 0

        # Dimensions véhicule
        hfc.vehicle_length.vehicle_length_value.value = VEHICLE_LENGTH
        hfc.vehicle_width.value                        = VEHICLE_WIDTH

        # Accélération longitudinale ETSI : 1/10 m/s² [-160..161]
        accel_etsi = int(round(self._accel_x * 10.0))
        accel_etsi = max(-160, min(161, accel_etsi))
        hfc.longitudinal_acceleration \
            .longitudinal_acceleration_value.value = accel_etsi
        hfc.longitudinal_acceleration \
            .longitudinal_acceleration_confidence.value = 102

        # Yaw rate ETSI : 1/100 deg/s [-32766..32766], 32767=unavailable
        yaw_rate_degs  = math.degrees(self._yaw_rate)
        yaw_rate_etsi  = int(round(yaw_rate_degs * 100.0))
        yaw_rate_etsi  = max(-32766, min(32766, yaw_rate_etsi))
        hfc.yaw_rate.yaw_rate_value.value    = yaw_rate_etsi
        hfc.yaw_rate.yaw_rate_confidence.value = 8

        # Courbure (unavailable)
        hfc.curvature.curvature_value.value    = 1023
        hfc.curvature.curvature_confidence.value = 7
        hfc.curvature_calculation_mode.value   = 2

        self.cam_pub.publish(cam)

        self.get_logger().debug(
            f'CAM publié | '
            f'lat={self._lat:.6f} lon={self._lon:.6f} | '
            f'hdg={self._heading:.1f}° | '
            f'spd={self._speed_ms:.2f} m/s | '
            f'yaw_rate={math.degrees(self._yaw_rate):.2f} deg/s')


def main():
    rclpy.init()
    node = CamGeneratorV2()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
