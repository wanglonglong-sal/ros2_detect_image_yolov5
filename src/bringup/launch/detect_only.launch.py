from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_video_arg = DeclareLaunchArgument(
        'output_video_path',
        default_value='/mnt/d/Dataset/Output/detect_output.mp4'
    )

    yolo_node = Node(
        package='detect',
        executable='mask_detect',
        name='mask_detect',
        output='screen',
        parameters=[{'output_video_path': LaunchConfiguration('output_video_path')}],
    )

    sub_node = Node(
        package='perception',
        executable='camera_subscriber',
        name='camera_subscriber',
        output='screen',
    )

    return LaunchDescription([output_video_arg, yolo_node, sub_node])
