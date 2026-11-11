# ROS 2 Jazzy integration

The world now uses `controllers/drone_controller/drone_controller_ros2.py`. The original controller is loaded unchanged and wrapped with a ROS 2 node, so the simulation still supports the keyboard shortcuts while exposing ROS topics.

## Build and launch

Install ROS 2 Jazzy, Webots, and the `webots_ros2_driver` package first.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch drone_webots_ros2 simulation.launch.py
```

The launch file starts `worlds/drone_parapluie.wbt`, which selects the ROS-enabled drone controller.

## Topics

- `/drone/pose` (`geometry_msgs/msg/PoseStamped`): drone position in the `webots_world` frame.
- `/drone/status` (`std_msgs/msg/String`): current mode, `gps` or `vision`.
- `/drone/set_mode` (`std_msgs/msg/Int32`): command `1` for GPS mode or `2` for vision mode.

Example:

```bash
ros2 topic echo /drone/pose
ros2 topic pub --once /drone/set_mode std_msgs/msg/Int32 '{data: 1}'
ros2 topic pub --once /drone/set_mode std_msgs/msg/Int32 '{data: 2}'
```

If ROS 2 is not sourced, use the original Webots controller/world configuration as before. ROS mode requires the Python environment used by Webots to be able to import `rclpy`, `geometry_msgs`, and `std_msgs`.
