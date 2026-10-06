"""MoveIt (move_group + RViz) for the robot already running in Gazebo.

Run AFTER `ros2 launch my_arm_bringup gazebo.launch.py`. Does not start
robot_state_publisher or ros2_control: Gazebo already provides both.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config():
    # Use the exact same robot description Gazebo spawned
    xacro = Path(get_package_share_directory("my_arm_bringup")) / "description" / "urdf" / "my_arm.urdf.xacro"
    return (
        MoveItConfigsBuilder("my_arm", package_name="my_arm_moveit_config")
        .robot_description(file_path=str(xacro))
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .moveit_cpp(file_path="config/moveit_py.yaml")
        .to_moveit_configs()
    )


def generate_launch_description():
    moveit_config = build_moveit_config()
    sim_time = {"use_sim_time": True}

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), sim_time],
    )

    rviz_config = str(Path(get_package_share_directory("my_arm_moveit_config")) / "config" / "moveit.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            sim_time,
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        move_group,
        rviz,
    ])
