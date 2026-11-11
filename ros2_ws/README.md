# ROS 2 Jazzy integration

This workspace adds a ROS 2 Jazzy bridge for the Webots drone simulation.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch drone_webots_ros2 simulation.launch.py
```

Set the drone mode with `1` for GPS or `2` for vision:

```bash
ros2 topic pub --once /drone/set_mode std_msgs/msg/Int32 '{data: 1}'
```
