#!/usr/bin/env python3
"""
perception_shm_bridge.py — ROS2 /carla/hero/perceived_objects → /dev/shm/perceived_objects
Reçoit le JSON publié par carla_perception_bridge et l'écrit en shm
pour qu'Artery puisse le lire sans overhead fichier.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

SHM_PATH = '/dev/shm/perceived_objects'
MAX_SIZE = 65536  # 64KB largement suffisant

class PerceptionShmBridge(Node):
    def __init__(self):
        super().__init__('perception_shm_bridge')

        # Init shm avec des zéros
        with open(SHM_PATH, 'wb') as f:
            f.write(b'\x00' * MAX_SIZE)

        self.create_subscription(
            String,
            '/carla/hero/perceived_objects',
            self.on_perceived,
            10
        )
        self.get_logger().info(f'Perception SHM bridge started → {SHM_PATH}')

    JSON_PATH = '/home/syfra/artery_fresh/scenarios/F2MD_Highway_CPM/carla_perceived_objects.json'

    def on_perceived(self, msg):
        data = msg.data.encode('utf-8')
        # Écrire longueur (4 bytes) + JSON dans SHM
        size = len(data)
        if size + 4 > MAX_SIZE:
            self.get_logger().warn(f'Payload trop grand: {size} bytes')
            return
        with open(SHM_PATH, 'r+b') as f:
            f.write(size.to_bytes(4, 'little'))
            f.write(data)
        # Écrire aussi dans fichier JSON pour Artery
        with open(self.JSON_PATH, 'w') as f:
            f.write(msg.data)
        self.get_logger().debug(f'SHM updated: {size} bytes, objects written')

def main():
    rclpy.init()
    node = PerceptionShmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
