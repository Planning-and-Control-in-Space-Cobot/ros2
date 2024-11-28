from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="attitude_controller",
                executable="attitude_controller_main",
                name="attitude_controller",
                output="screen",
                parameters=[
                    {"param_name": "param_value"},
                    {"param_name1": "param_value"},
                ],
            )
        ]
    )
