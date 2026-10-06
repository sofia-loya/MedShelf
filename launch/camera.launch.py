"""Bridge the MedShelf shelf camera from Gazebo to ROS 2. Run alongside gazebo.launch.py."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge_config = os.path.join(
        get_package_share_directory('my_arm_bringup'), 'config', 'camera_bridge.yaml')

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([camera_bridge])