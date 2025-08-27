from setuptools import find_packages, setup
import sys

package_name = 'detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/yolov5_video_detector.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wanglonglong',
    maintainer_email='wanglonglong02@gmail.com',
    description='YOLOv5-based object detection node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov5_video_detector = detect.yolov5_video_detector:main'
        ],
    },
    options={
        'build_scripts': {
            'executable': '/home/wanglonglong/ros2_env/bin/python3',  # 你的虚拟环境Python路径
        },
    },
)
