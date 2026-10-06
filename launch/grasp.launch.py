"""Bridge the bin attach/detach topics to Gazebo and release every bin at startup.

Included by gazebo.launch.py right after the robot spawns.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

BINS = ['gauze', 'gloves', 'syringes', 'masks', 'tape', 'wipes',
        'dressings', 'saline', 'specimen_cups']


def generate_launch_description():
    bridge_args = []
    for b in BINS:
        bridge_args += [
            f'/medshelf/attach/tray_{b}@std_msgs/msg/Empty]gz.msgs.Empty',
            f'/medshelf/detach/tray_{b}@std_msgs/msg/Empty]gz.msgs.Empty',
            f'/medshelf/attach_state/tray_{b}@std_msgs/msg/String[gz.msgs.StringMsg',
        ]

    grasp_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='grasp_bridge',
        arguments=bridge_args,
        output='screen',
    )

    # Gazebo attaches every DetachableJoint when the robot spawns, so detach
    # all bins once the bridge is up (sent 3 times in case one is dropped).
    release_all = [
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--times', '3', '--rate', '2',
                 f'/medshelf/detach/tray_{b}', 'std_msgs/msg/Empty', '{}'],
            output='log',
        )
        for b in BINS
    ]

    return LaunchDescription([
        grasp_bridge,
        TimerAction(period=2.0, actions=release_all),
    ])