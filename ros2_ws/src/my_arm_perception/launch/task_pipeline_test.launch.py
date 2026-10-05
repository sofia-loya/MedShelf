from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    seed_argument = DeclareLaunchArgument(
        "seed",
        default_value="0",
        description="Random seed used to shuffle bin colors.",
    )

    requested_bin_argument = DeclareLaunchArgument(
        "requested_bin",
        default_value="gauze",
        description="Requested supply ID.",
    )

    mock_result_argument = DeclareLaunchArgument(
        "mock_result",
        default_value="SUCCEEDED",
        description="Mock manipulation result, either SUCCEEDED or FAILED.",
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
            "required_stable_frames": 3,
        }],
    )

    task_manager = Node(
        package="my_arm_perception",
        executable="task_manager",
        name="task_manager",
        output="screen",
        parameters=[{
            "requested_bin": LaunchConfiguration("requested_bin"),
        }],
    )

    mock_manipulation = Node(
        package="my_arm_perception",
        executable="mock_manipulation",
        name="mock_manipulation",
        output="screen",
        parameters=[{
            "result": LaunchConfiguration("mock_result"),
            "execution_delay_s": 2.0,
        }],
    )

    return LaunchDescription([
        seed_argument,
        requested_bin_argument,
        mock_result_argument,
        shelf_publisher,
        bin_detector,
        task_manager,
        mock_manipulation,
    ])
