from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
            parameters=[{
                'camera_id': 0,  # Optional parameter, customize as needed
                'frame_rate': 30
            }]
        ),
        Node(
            package='perception',
            executable='camera_subscriber',
            name='camera_subscriber',
            output='screen',
            parameters=[{
                'camera_id': 0,  # Optional parameter, customize as needed
                'frame_rate': 30
            }]
        )

    ])
