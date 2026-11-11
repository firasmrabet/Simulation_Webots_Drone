from pathlib import Path

from launch import LaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    repository_root = Path(__file__).resolve().parents[4]
    world = repository_root / 'worlds' / 'drone_parapluie.wbt'
    webots = WebotsLauncher(world=str(world))
    return LaunchDescription([webots, webots._on_exit])
