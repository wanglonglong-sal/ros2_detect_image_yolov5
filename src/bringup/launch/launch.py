# bringup/launch/launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument('image_topic', default_value='/image_raw')
    output_video_arg = DeclareLaunchArgument(
        'output_video_path', default_value='/mnt/d/Dataset/Output/tracked_output.mp4'
    )

    yolo_node = Node(
        package='detect',
        executable='mask_detect',
        name='mask_detect',
        output='screen',
    )

    tracker_node = Node(
        package='trackor',
        executable='object_detect',
        name='object_tracker',
        output='screen',
        parameters=[{'output_video_path': LaunchConfiguration('output_video_path')}],
    )

    sub_node = Node(
        package='perception',
        executable='camera_subscriber',
        name='camera_subscriber',
        output='screen',
    )

    return LaunchDescription([image_topic_arg, output_video_arg, yolo_node, tracker_node, sub_node])
