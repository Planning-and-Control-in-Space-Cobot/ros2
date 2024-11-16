from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the bridge node to relay topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/space_cobot0/motor/actuator_cmd@actuator_msgs/msg/Actuators@gz.msgs.Actuators'
        ],
        output='screen'
    )

    return LaunchDescription([
        bridge,
    ])
