import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class DroneRosBridge(Node):
    def __init__(self):
        super().__init__('drone_ros_bridge')
        self.mode_pub = self.create_publisher(String, '/drone/mode', 10)
        self.status_pub = self.create_publisher(String, '/drone/status', 10)
        self.create_subscription(Int32, '/drone/set_mode', self.set_mode, 10)
        self.mode_file = Path(os.environ.get('DRONE_MODE_FILE', '/tmp/drone_mode.txt'))
        self.create_timer(0.2, self.publish_state)

    def set_mode(self, message: Int32):
        if message.data not in (1, 2):
            self.get_logger().error('Mode must be 1 (GPS) or 2 (vision)')
            return
        self.mode_file.write_text(str(message.data), encoding='ascii')
        self.status_pub.publish(String(data=f'mode_command_sent:{message.data}'))

    def publish_state(self):
        mode = 'unknown'
        if self.mode_file.exists():
            mode = self.mode_file.read_text(encoding='ascii').strip()
        self.mode_pub.publish(String(data=mode))


def main(args=None):
    rclpy.init(args=args)
    node = DroneRosBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
