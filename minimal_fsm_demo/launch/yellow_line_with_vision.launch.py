from launch import LaunchDescription
from launch.actions import ExecuteProcess
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
            ExecuteProcess(
                cmd=[
                    "python3",
                    "-m",
                    "minimal_fsm_demo.ball_color_detector",
                    "--ros-args",
                    "-p",
                    "image_topic:=/rgb_camera/image_raw",
                ],
                output="screen",
            ),
        ]
    )
