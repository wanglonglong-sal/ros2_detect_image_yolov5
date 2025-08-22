# bringup/launch/launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    device_arg      = DeclareLaunchArgument('device',      default_value='/dev/video0')
    image_topic_arg = DeclareLaunchArgument('image_topic', default_value='/image_raw')

    cam_yaml = PathJoinSubstitution([FindPackageShare('bringup'), 'config', 'cam.yaml'])

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        # 先加载 YAML（类型是对的），再可选覆盖某些参数
        parameters=[cam_yaml, {'video_device': LaunchConfiguration('device')}],
        remappings=[('image_raw', LaunchConfiguration('image_topic'))],
    )

    yolo_node = Node(
        package='detect',
        executable='mask_detect',
        name='mask_detect',
        output='screen',
        remappings=[('/image_raw', LaunchConfiguration('image_topic'))],
    )

    sub_node = Node(
        package='perception',
        executable='camera_subscriber',
        name='camera_subscriber',
        output='screen',
        remappings=[('/image_raw', LaunchConfiguration('image_topic'))],
    )

    return LaunchDescription([device_arg, image_topic_arg, camera_node, yolo_node, sub_node])
