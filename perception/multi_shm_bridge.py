#!/usr/bin/env python3
"""
multi_shm_bridge.py — Bridge ROS2 → /dev/shm/agent_N
Généralise cam_bridge.py pour N véhicules.
Lit /vehicle_N/gnss/fix + /vehicle_N/imu/data + /vehicle_N/vehicle_status
Écrit /dev/shm/agent_N (7 doubles : lat, lon, heading, speed, accel, yaw_rate, timestamp)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistStamped
import struct, mmap, math, os, time

SHM_FORMAT = "ddddddd"  # 7 doubles
SHM_SIZE   = struct.calcsize(SHM_FORMAT)  # 56 bytes

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))

def yaw_to_heading(yaw_deg):
    return (90.0 - yaw_deg) % 360.0


class AgentBridge(Node):
    def __init__(self, agent_index: int):
        super().__init__(f'agent_bridge_{agent_index}')
        self.agent_index = agent_index
        ns = f'vehicle_{agent_index + 1}'
        shm_path = f'/dev/shm/agent_{agent_index}'

        # Init shared memory
        with open(shm_path, 'wb') as f:
            f.write(b'\x00' * SHM_SIZE)
        self.shm_fd = open(shm_path, 'r+b')
        self.shm = mmap.mmap(self.shm_fd.fileno(), SHM_SIZE)

        self.lat = self.lon = self.alt = 0.0
        self.heading = self.speed = self.accel = self.yaw_rate = 0.0

        self.create_subscription(NavSatFix,    f'/{ns}/gnss/fix',       self.gnss_cb,   SENSOR_QOS)
        self.create_subscription(Imu,          f'/{ns}/imu/data',       self.imu_cb,    SENSOR_QOS)
        self.create_subscription(TwistStamped, f'/{ns}/vehicle_status', self.status_cb, 10)

        self.get_logger().info(f"[agent_{agent_index}] bridge started → {shm_path}")

    def gnss_cb(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.alt = msg.altitude
        self.write()

    def imu_cb(self, msg):
        yaw = quat_to_yaw(msg.orientation.x, msg.orientation.y,
                          msg.orientation.z, msg.orientation.w)
        self.heading  = yaw_to_heading(yaw)
        self.yaw_rate = msg.angular_velocity.z
        self.accel    = msg.linear_acceleration.x
        self.write()

    def status_cb(self, msg):
        # vehicle_status: linear.x = speed_kmh
        self.speed = msg.twist.linear.x / 3.6  # km/h → m/s
        self.write()

    def write(self):
        data = struct.pack(SHM_FORMAT,
            self.lat, self.lon, self.heading,
            self.speed, self.accel, self.yaw_rate,
            time.time()
        )
        self.shm.seek(0)
        self.shm.write(data)

    def destroy_node(self):
        self.shm.close()
        self.shm_fd.close()
        super().destroy_node()


def main():
    rclpy.init()
    from rclpy.executors import MultiThreadedExecutor

    n_agents = int(os.environ.get('N_AGENTS', '3'))
    executor = MultiThreadedExecutor(num_threads=n_agents + 1)

    nodes = []
    for i in range(n_agents):
        node = AgentBridge(i)
        nodes.append(node)
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
