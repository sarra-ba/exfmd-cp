"""
fake_cam_publisher.py
Publie des CAMs depuis uphf_trajectory.csv à 10Hz.
Encode carla_x/carla_y dans les champs altitude + yaw_rate
pour éviter toute conversion GPS dans le bridge.
"""
import rclpy
from rclpy.node import Node
from etsi_its_cam_msgs.msg import CAM
import csv
import math


class FakeCamPublisher(Node):
    def __init__(self):
        super().__init__('fake_cam_publisher')
        self.pub = self.create_publisher(CAM, '/its/cam_received', 10)

        self.rows = []
        with open('/home/syfra/workspace/moad_workspace/uphf_trajectory.csv') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.rows.append({
                    'lat':     float(row['lat']),
                    'lon':     float(row['lon']),
                    'carla_x': float(row['carla_x']),
                    'carla_y': float(row['carla_y']),
                    'z':       float(row['z']),
                    'heading': float(row['heading']),  # degrés géo, peut être négatif
                    'time':    float(row['time']),
                })

        # Filtrer points quasi-statiques (< 0.3m)
        filtered = [self.rows[0]]
        for row in self.rows[1:]:
            prev = filtered[-1]
            dx = row['carla_x'] - prev['carla_x']
            dy = row['carla_y'] - prev['carla_y']
            if math.sqrt(dx**2 + dy**2) > 0.3:
                filtered.append(row)
        self.rows = filtered

        self.idx = 0
        self.timer = self.create_timer(0.1, self.publish_cam)  # 10 Hz
        self.get_logger().info(
            f'FakeCamPublisher: {len(self.rows)} points après filtrage')

    def publish_cam(self):
        if self.idx >= len(self.rows):
            self.get_logger().info('Trajectoire terminée')
            self.timer.cancel()
            return

        row = self.rows[self.idx]

        cam = CAM()
        cam.header.protocol_version = 2
        cam.header.message_id = 2
        cam.header.station_id.value = 427949455

        # ── Position GPS (référence standard ETSI ×1e7) ──────────────────
        cam.cam.cam_parameters.basic_container \
            .reference_position.latitude.value  = int(row['lat'] * 1e7)
        cam.cam.cam_parameters.basic_container \
            .reference_position.longitude.value = int(row['lon'] * 1e7)

        # ── Heading ETSI : Nord=0, sens horaire, 1/10° [0..3600] ─────────
        # heading CSV est en degrés géo (Nord=0 CW), peut être négatif
        heading_geo = row['heading'] % 360.0          # ramener dans [0, 360)
        heading_etsi = int(round(heading_geo * 10.0)) % 3601
        cam.cam.cam_parameters.high_frequency_container \
            .basic_vehicle_container_high_frequency \
            .heading.heading_value.value = heading_etsi

        # ── Vitesse ETSI : 1/100 m/s ─────────────────────────────────────
        if self.idx < len(self.rows) - 1:
            nxt = self.rows[self.idx + 1]
            dx   = nxt['carla_x'] - row['carla_x']
            dy   = nxt['carla_y'] - row['carla_y']
            dist = math.sqrt(dx**2 + dy**2)
            dt   = max((nxt['time'] - row['time']), 0.001)
            speed_ms  = dist / dt
        else:
            speed_ms = 0.0
        speed_etsi = int(round(speed_ms * 100.0))
        speed_etsi = max(0, min(speed_etsi, 16383))   # clamp ETSI max
        cam.cam.cam_parameters.high_frequency_container \
            .basic_vehicle_container_high_frequency \
            .speed.speed_value.value = speed_etsi

        # ── Encoder carla_x / carla_y pour le bridge ─────────────────────
        # On utilise altitude_value (×100 cm) et yaw_rate (×100) comme
        # canaux de transport — le bridge les lira directement.
        # carla_x et carla_y sont en mètres, on stocke ×100 pour avoir
        # la précision centimétrique dans un int32.
        cam.cam.cam_parameters.basic_container \
            .reference_position.altitude.altitude_value.value = \
            int(round(row['carla_x'] * 100.0))   # carla_x ×100

        cam.cam.cam_parameters.high_frequency_container \
            .basic_vehicle_container_high_frequency \
            .yaw_rate.yaw_rate_value.value = \
            int(round(row['carla_y'] * 100.0))   # carla_y ×100

        self.pub.publish(cam)
        self.get_logger().info(
            f'CAM #{self.idx}/{len(self.rows)-1} | '
            f'carla=({row["carla_x"]:.2f}, {row["carla_y"]:.2f}) | '
            f'hdg={heading_geo:.1f}° | spd={speed_ms:.2f} m/s')
        self.idx += 1


def main():
    rclpy.init()
    node = FakeCamPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
