from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="minimal_fsm_demo",
                executable="yellow_line_fsm_demo",
                name="yellow_line_fsm_demo",
                output="screen",
            ),
        ]
    )
