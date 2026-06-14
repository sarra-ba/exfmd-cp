import math
from typing import List, Tuple

import etsi_its_cam_msgs.msg as cam_msg
import rclpy
from rclpy.node import Node


TrajectoryPoint = Tuple[float, float]


class CamProviderStaticPath(Node):
    def __init__(self):
        super().__init__('cam_provider_static_path')
        self.get_logger().info(f'Node "{self.get_name()}" started')

        self.cam_publisher = self.create_publisher(cam_msg.CAM, '/its/cam_provided', 1)
        self.station_id = 12001
        self.timer_period = 1.0
        self.trajectory: List[TrajectoryPoint] = [
            (50.35864, 3.52595),
            (50.35879, 3.52640),
            (50.35858, 3.52692),
            (50.35814, 3.52710),
            (50.35776, 3.52688),
            (50.35757, 3.52635),
            (50.35775, 3.52579),
            (50.35820, 3.52559),
        ]
        self.current_index = 0
        self.create_timer(self.timer_period, self.publish)

    @staticmethod
    def _clamp(value: int, minimum: int, maximum: int) -> int:
        return max(minimum, min(maximum, value))

    @staticmethod
    def _to_cam_latitude(latitude_deg: float) -> int:
        return CamProviderStaticPath._clamp(int(round(latitude_deg * 1e7)), -900000000, 900000001)

    @staticmethod
    def _to_cam_longitude(longitude_deg: float) -> int:
        return CamProviderStaticPath._clamp(int(round(longitude_deg * 1e7)), -1800000000, 1800000001)

    @staticmethod
    def _bearing_deg(start: TrajectoryPoint, end: TrajectoryPoint) -> float:
        lat1 = math.radians(start[0])
        lat2 = math.radians(end[0])
        dlon = math.radians(end[1] - start[1])
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

    @staticmethod
    def _distance_m(start: TrajectoryPoint, end: TrajectoryPoint) -> float:
        earth_radius_m = 6371000.0
        lat1 = math.radians(start[0])
        lat2 = math.radians(end[0])
        dlat = lat2 - lat1
        dlon = math.radians(end[1] - start[1])
        a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        return earth_radius_m * c

    def _generate_cam(self, point: TrajectoryPoint, next_point: TrajectoryPoint) -> cam_msg.CAM:
        heading_deg = self._bearing_deg(point, next_point)
        heading_value = self._clamp(int(round(heading_deg * 10.0)), 0, 3600)
        speed_mps = self._distance_m(point, next_point) / self.timer_period
        speed_value = self._clamp(int(round(speed_mps * 100.0)), 0, 16382)

        msg = cam_msg.CAM()
        msg.header.station_id.value = self.station_id
        msg.cam.cam_parameters.basic_container.station_type.value = 5
        ref = msg.cam.cam_parameters.basic_container.reference_position
        ref.latitude.value = self._to_cam_latitude(point[0])
        ref.longitude.value = self._to_cam_longitude(point[1])
        ref.position_confidence_ellipse.semi_major_confidence.value = 20
        ref.position_confidence_ellipse.semi_minor_confidence.value = 20
        ref.altitude.altitude_value.value = 0
        ref.altitude.altitude_confidence.value = 15

        hf = msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency
        hf.heading.heading_value.value = heading_value
        hf.heading.heading_confidence.value = 127
        hf.speed.speed_value.value = speed_value
        hf.speed.speed_confidence.value = 127
        hf.drive_direction.value = 0
        hf.vehicle_length.vehicle_length_value.value = 450
        hf.vehicle_length.vehicle_length_confidence_indication.value = 1
        hf.vehicle_width.value = 180
        hf.longitudinal_acceleration.longitudinal_acceleration_value.value = 161
        hf.longitudinal_acceleration.longitudinal_acceleration_confidence.value = 102
        hf.curvature.curvature_value.value = 30001
        hf.curvature.curvature_confidence.value = 7
        hf.curvature_calculation_mode.value = 0
        hf.yaw_rate.yaw_rate_value.value = 32767
        hf.yaw_rate.yaw_rate_confidence.value = 8
        return msg

    def publish(self) -> None:
        point = self.trajectory[self.current_index]
        next_index = (self.current_index + 1) % len(self.trajectory)
        next_point = self.trajectory[next_index]

        cam = self._generate_cam(point, next_point)
        self.cam_publisher.publish(cam)
        self.get_logger().info(
            f'CAM provided at lat={point[0]:.6f}, lon={point[1]:.6f}, idx={self.current_index}'
        )
        self.current_index = next_index


def main(args=None):
    rclpy.init(args=args)
    node = CamProviderStaticPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
