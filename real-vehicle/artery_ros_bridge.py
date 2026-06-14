"""
artery_ros2_bridge.py
Reçoit depuis Artery via UDP :
  - ports 9001/9002/9003 : données JSON → etsi_its_cam_msgs/CAM
  - ports 9301/9302/9303 : bytes bruts GeoNet+BTP+CAM → std_msgs/ByteMultiArray

Publie sur :
  - /its/cam/v{station_id}           → etsi_its_cam_msgs/CAM
  - /simulator/vehicle{idx}/cam      → std_msgs/ByteMultiArray (GeoNet+BTP+CAM encodé)
"""
import socket
import json
import threading
import math
import traceback
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from etsi_its_cam_msgs.msg import CAM
from std_msgs.msg import ByteMultiArray, MultiArrayDimension

# Ports JSON (données décodées)
UDP_JSON_PORTS = [9001, 9002, 9003]

# Ports RAW (GeoNet+BTP+CAM encodé)
UDP_RAW_PORTS  = [9301, 9302, 9303]

VEHICLE_LENGTH = 459  # DS7 4.59m
VEHICLE_WIDTH  = 189  # DS7 1.89m


class ArteryRos2Bridge(Node):
    def __init__(self):
        super().__init__('artery_ros2_bridge')
        self._cam_pubs = {}
        self._raw_pubs = {}
        self._lock = threading.Lock()

        # Pré-créer les publishers raw pour vehicle1, vehicle2, vehicle3
        for idx in range(len(UDP_RAW_PORTS)):
            topic = f'/simulator/vehicle{idx + 1}/cam'
            self._raw_pubs[idx] = self.create_publisher(ByteMultiArray, topic, 10)
            self.get_logger().info(f'Publisher RAW: {topic}')

        # Threads UDP JSON
        for port in UDP_JSON_PORTS:
            t = threading.Thread(target=self._udp_json_listener, args=(port,), daemon=True)
            t.start()

        # Threads UDP RAW
        for i, port in enumerate(UDP_RAW_PORTS):
            t = threading.Thread(target=self._udp_raw_listener, args=(port, i), daemon=True)
            t.start()

        self.get_logger().info('Artery → ROS2 bridge démarré')

    def _get_cam_publisher(self, station_id):
        with self._lock:
            if station_id not in self._cam_pubs:
                topic = f'/its/cam/v{station_id}'
                self._cam_pubs[station_id] = self.create_publisher(CAM, topic, 10)
                self.get_logger().info(f'Publisher CAM: {topic}')
            return self._cam_pubs[station_id]

    # ── UDP JSON listener ────────────────────────────────────────────────────
    def _udp_json_listener(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', port))
        self.get_logger().info(f'UDP JSON listener port {port}')
        while rclpy.ok():
            try:
                data, _ = sock.recvfrom(1024)
                d = json.loads(data.decode())
                self._publish_cam(d)
            except json.JSONDecodeError as e:
                self.get_logger().warn(f'JSON error port {port}: {e}')
            except Exception as e:
                traceback.print_exc()
                self.get_logger().warn(f'UDP JSON error port {port}: {e}')

    # ── UDP RAW listener ─────────────────────────────────────────────────────
    def _udp_raw_listener(self, port, vehicle_idx):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', port))
        self.get_logger().info(f'UDP RAW listener port {port} → vehicle{vehicle_idx + 1}')
        while rclpy.ok():
            try:
                data, _ = sock.recvfrom(4096)
                self._publish_raw(data, vehicle_idx)
            except Exception as e:
                self.get_logger().warn(f'UDP RAW error port {port}: {e}')

    # ── Publier CAM décodé ───────────────────────────────────────────────────
    def _publish_cam(self, d):
        try:
            lat        = float(d['lat'])
            lon        = float(d['lon'])
            heading    = float(d['heading'])
            speed      = float(d['speed'])
            accel      = float(d.get('accel', 0.0))
            yaw_rate   = float(d.get('yaw_rate', 0.0))
            station_id = int(d['station_id'])

            cam = CAM()
            cam.header.protocol_version = 2
            cam.header.message_id       = 2
            cam.header.station_id.value = station_id
            cam.cam.cam_parameters.basic_container.station_type.value = 5
            cam.cam.cam_parameters.basic_container \
                .reference_position.latitude.value  = int(lat * 1e7)
            cam.cam.cam_parameters.basic_container \
                .reference_position.longitude.value = int(lon * 1e7)

            hfc = cam.cam.cam_parameters.high_frequency_container \
                      .basic_vehicle_container_high_frequency
            hfc.heading.heading_value.value      = int(round(heading * 10.0)) % 3601
            hfc.heading.heading_confidence.value = 126
            hfc.speed.speed_value.value          = max(0, min(16383, int(round(speed * 100.0))))
            hfc.speed.speed_confidence.value     = 52
            hfc.drive_direction.value            = 0 if speed >= 0 else 1
            hfc.vehicle_length.vehicle_length_value.value = VEHICLE_LENGTH
            hfc.vehicle_width.value              = VEHICLE_WIDTH
            hfc.longitudinal_acceleration \
                .longitudinal_acceleration_value.value = max(-160, min(161, int(round(accel * 10.0))))
            hfc.longitudinal_acceleration \
                .longitudinal_acceleration_confidence.value = 102
            hfc.yaw_rate.yaw_rate_value.value      = max(-32766, min(32766,
                int(round(math.degrees(yaw_rate) * 100.0))))
            hfc.yaw_rate.yaw_rate_confidence.value = 8
            hfc.curvature.curvature_value.value      = 1023
            hfc.curvature.curvature_confidence.value = 7
            hfc.curvature_calculation_mode.value     = 2

            pub = self._get_cam_publisher(station_id)
            pub.publish(cam)
            self.get_logger().debug(
                f'[{station_id}] CAM → lat={lat:.6f} lon={lon:.6f}')

        except Exception as e:
            traceback.print_exc()
            self.get_logger().warn(f'publish_cam error: {e}')

    # ── Publier bytes bruts GeoNet+BTP+CAM ──────────────────────────────────
    def _publish_raw(self, data: bytes, vehicle_idx: int):
        try:
            msg = ByteMultiArray()
            dim = MultiArrayDimension()
            dim.label  = 'bytes'
            dim.size   = len(data)
            dim.stride = len(data)
            msg.layout.dim.append(dim)
            msg.data = [bytes([b]) for b in data]

            pub = self._raw_pubs[vehicle_idx]
            pub.publish(msg)
            self.get_logger().debug(
                f'vehicle{vehicle_idx + 1} RAW CAM → {len(data)} bytes')

        except Exception as e:
            traceback.print_exc()
            self.get_logger().warn(f'publish_raw error: {e}')


def main():
    rclpy.init()
    node = ArteryRos2Bridge()
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
