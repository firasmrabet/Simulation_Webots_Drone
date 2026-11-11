from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    repository_root = Path(__file__).resolve().parents[4]
    world = repository_root / 'worlds' / 'drone_parapluie.wbt'
    webots = WebotsLauncher(world=str(world))
    bridge = Node(
        package='drone_webots_ros2',
        executable='ros_bridge',
        output='screen',
    )
    return LaunchDescription([webots, bridge, webots._on_exit])
