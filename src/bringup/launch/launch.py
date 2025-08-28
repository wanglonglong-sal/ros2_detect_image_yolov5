# bringup/launch/launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
try:
    from ament_index_python.packages import get_package_share_directory
    _has_ament_index = True
except Exception:
    _has_ament_index = False

def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument('image_topic', default_value='/image_raw')
    output_video_arg = DeclareLaunchArgument(
        'output_video_path', default_value='/mnt/d/Dataset/Output/tracked_output.mp4'
    )

    yolo_node = Node(
        package='detect',
        executable='yolov5_video_detector',
        name='yolov5_video_detector',
        output='screen',
    )

    # Tracker params: load packaged YAML and allow launch arg to override output path
    try:
        tracker_cfg = os.path.join(
            get_package_share_directory('trackor'),
            'config',
            'object_tracker.yaml',
        )
        tracker_params = [tracker_cfg, {'output_video_path': LaunchConfiguration('output_video_path')}]
    except Exception:
        tracker_params = [{'output_video_path': LaunchConfiguration('output_video_path')}]

    tracker_node = Node(
        package='trackor',
        executable='object_tracker',
        name='object_tracker',
        output='screen',
        parameters=tracker_params,
    )

    # Try to include action_recognizer if available; otherwise skip gracefully
    action_node = None
    try:
        action_params = [{'backend': 'stub'}]
        if _has_ament_index:
            action_cfg = os.path.join(
                get_package_share_directory('action_recognizer'),
                'config',
                'action_recognizer.yaml',
            )
            # Use YAML params if package is discoverable
            action_params = [action_cfg]

        action_node = Node(
            package='action_recognizer',
            executable='action_recognizer_node',
            name='action_recognizer',
            output='screen',
            parameters=action_params,
        )
    except Exception:
        action_node = None

    sub_node = Node(
        package='perception',
        executable='camera_subscriber',
        name='camera_subscriber',
        output='screen',
    )

    actions = [
        image_topic_arg,
        output_video_arg,
        yolo_node,
        tracker_node,
        sub_node,
    ]
    if action_node is not None:
        actions.insert(4, action_node)
    return LaunchDescription(actions)
