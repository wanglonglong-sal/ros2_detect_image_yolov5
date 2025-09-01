from setuptools import find_packages, setup

package_name = 'action_recognizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/action_recognizer.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wanglonglong',
    maintainer_email='wanglonglong02@gmail.com',
    description='Action recognition node consuming tracked objects and images',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_recognizer_node = action_recognizer.node:main',
        ],
    },
    # 新增：强制指定可执行文件的Python解释器路径
    options={
        'build_scripts': {
            'executable': '/home/wanglonglong/ros2_env/bin/python3',  # 你的虚拟环境Python路径
        },
    },    
)

