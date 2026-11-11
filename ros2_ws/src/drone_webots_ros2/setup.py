from setuptools import find_packages, setup

package_name = 'drone_webots_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simulation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': ['ros_bridge = drone_webots_ros2.ros_bridge:main']},
)
