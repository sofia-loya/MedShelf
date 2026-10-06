"""Start the MedShelf world in Gazebo, spawn the UR5 + Robotiq arm, and bring up its controllers."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('my_arm_bringup')
    world = os.path.join(pkg_share, 'worlds', 'medshelf.world')
    xacro_file = os.path.join(pkg_share, 'description', 'urdf', 'my_arm.urdf.xacro')

    # Lets Gazebo find package:// meshes for our world, the UR5, and the gripper
    resource_paths = os.pathsep.join(
        os.path.dirname(get_package_share_directory(pkg))
        for pkg in ('my_arm_bringup', 'ur_description', 'robotiq_description')
    )
    set_resource_path = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_paths)

    # Robot base: on the lower tabletop, just behind the raised delivery tray
    spawn_args = [
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.12'),
        DeclareLaunchArgument('z', default_value='1.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
        'use_sim_time': True,
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': f'-r -v 3 {world}', 'on_exit_shutdown': 'true'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_arm',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # Gazebo's sim clock -> ROS /clock, so every node runs on sim time
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'], output='screen')
    arm_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller'], output='screen')
    gripper_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_controller'], output='screen')

    # Bin attach/detach bridge; also releases all bins right after spawn
    grasp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'grasp.launch.py')))

    # Order: robot spawned -> joint states + grasp bridge -> arm + gripper controllers
    start_broadcaster = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_broadcaster, grasp]))
    start_controllers = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster,
                      on_exit=[arm_controller, gripper_controller]))

    return LaunchDescription(spawn_args + [
        set_resource_path,
        gazebo,
        robot_state_publisher,
        clock_bridge,
        spawn_robot,
        start_broadcaster,
        start_controllers,
    ])