from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    seed_argument = DeclareLaunchArgument(
        "seed",
        default_value="0",
        description="Random seed used to shuffle bin colors across the 3x3 shelf.",
    )

    requested_bin_argument = DeclareLaunchArgument(
        "requested_bin",
        default_value="gauze",
        description="Requested supply ID, for example gauze or gloves.",
    )

    shelf_publisher = Node(
        package="my_arm_perception",
        executable="shelf_test_publisher",
        name="shelf_test_publisher",
        output="screen",
        parameters=[{
            "seed": LaunchConfiguration("seed"),
            "image_topic": "/medshelf/camera/image_raw",
        }],
    )

    bin_detector = Node(
        package="my_arm_perception",
        executable="bin_detector",
        name="bin_detector",
        output="screen",
        parameters=[{
            "requested_bin": LaunchConfiguration("requested_bin"),
            "image_topic": "/medshelf/camera/image_raw",
            "world_frame": "world",
        }],
    )

    return LaunchDescription([
        seed_argument,
        requested_bin_argument,
        shelf_publisher,
        bin_detector,
    ])
