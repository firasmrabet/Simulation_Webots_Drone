#!/usr/bin/env python3
"""ROS 2 Jazzy adapter for the existing Webots drone controller."""

import importlib.util
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String


_original_path = Path(__file__).with_name('drone_controller.py')
_spec = importlib.util.spec_from_file_location('drone_controller_original', _original_path)
_original = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_original)


class RosDroneProtection(_original.DroneProtection):
    def __init__(self):
        rclpy.init(args=None)
        self.ros_node = rclpy.create_node('webots_drone_controller')
        self.pose_pub = self.ros_node.create_publisher(PoseStamped, '/drone/pose', 10)
        self.status_pub = self.ros_node.create_publisher(String, '/drone/status', 10)
        self.ros_node.create_subscription(Int32, '/drone/set_mode', self._mode_callback, 10)
        super().__init__()

    def _mode_callback(self, message):
        if message.data == 1:
            self.switch_mode(ord('1'))
        elif message.data == 2:
            self.switch_mode(ord('2'))
        else:
            self.ros_node.get_logger().error('Mode must be 1 (GPS) or 2 (vision)')

    def _publish_ros_state(self):
        position = self.drone_pos_field.getSFVec3f()
        pose = PoseStamped()
        pose.header.stamp = self.ros_node.get_clock().now().to_msg()
        pose.header.frame_id = 'webots_world'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        self.pose_pub.publish(pose)
        mode = 'gps' if self.flight_mode == self.MODE_AUTO else 'vision'
        self.status_pub.publish(String(data=mode))

    def run(self):
        labels = {self.MODE_AUTO: 'GPS Automatique', self.MODE_VISION: 'Intelligence Artificielle'}
        print('_' * 50)
        print('  ROS 2 Jazzy actif : /drone/pose, /drone/status, /drone/set_mode')
        print(f'  Pilotage actuel : {labels[self.flight_mode]}')
        print('_' * 50)

        try:
            while self.step(self.dt) != -1:
                rclpy.spin_once(self.ros_node, timeout_sec=0.0)
                key = self.keyboard.getKey()
                self.tick += 1
                if key in [ord('1'), ord('2')]:
                    self.switch_mode(key)

                roll, pitch, yaw = self.imu.getRollPitchYaw()
                x, y, z = self.gps.getValues()
                r_acc, p_acc, _ = self.gyro.getValues()
                if self.flight_mode == self.MODE_AUTO:
                    self.execute_auto_mode(roll, pitch, yaw, x, y, z, r_acc, p_acc)
                else:
                    self.execute_vision_mode(roll, pitch, yaw, x, y, z, r_acc, p_acc)
                if self.tick % 5 == 0:
                    self._publish_ros_state()
        finally:
            self.ros_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    RosDroneProtection().run()
