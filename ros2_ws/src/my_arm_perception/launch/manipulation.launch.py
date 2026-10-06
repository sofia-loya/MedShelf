"""Real manipulation node (MoveItPy). Replaces mock_manipulation.

Run after gazebo.launch.py (and optionally moveit_gazebo.launch.py for RViz).
Robot base position must match the gazebo.launch.py spawn args (x, y, z).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    xacro = Path(get_package_share_directory("my_arm_bringup")) / "description" / "urdf" / "my_arm.urdf.xacro"
    moveit_config = (
        MoveItConfigsBuilder("my_arm", package_name="my_arm_moveit_config")
        .robot_description(file_path=str(xacro))
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .moveit_cpp(file_path="config/moveit_py.yaml")
        .to_moveit_configs()
    )

    args = [DeclareLaunchArgument(n, default_value=v) for n, v in
            (("base_x", "0.0"), ("base_y", "0.12"), ("base_z", "1.05"))]

    # No `name=` here on purpose: MoveItPy creates its own internal node ("moveit_py")
    # and a launch-level name remap would rename every node in the process.
    manipulation = Node(
        package="my_arm_perception",
        executable="medshelf_manipulation",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "base_x": ParameterValue(LaunchConfiguration("base_x"), value_type=float),
                "base_y": ParameterValue(LaunchConfiguration("base_y"), value_type=float),
                "base_z": ParameterValue(LaunchConfiguration("base_z"), value_type=float),
            },
        ],
    )
    return LaunchDescription(args + [manipulation])
