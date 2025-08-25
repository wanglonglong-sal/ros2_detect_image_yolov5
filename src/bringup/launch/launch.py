# bringup/launch/launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument('image_topic', default_value='/image_raw')

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
    )

    sub_node = Node(
        package='perception',
        executable='camera_subscriber',
        name='camera_subscriber',
        output='screen',
    )

    return LaunchDescription([image_topic_arg, yolo_node, tracker_node, sub_node])
