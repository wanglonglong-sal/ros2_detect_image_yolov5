from setuptools import find_packages, setup
import sys

package_name = 'trackor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'deep-sort-realtime'],
    zip_safe=True,
    maintainer='wanglonglong',
    maintainer_email='wanglonglong02@gmail.com',
    description='Tracking detected objects using Deep SORT',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detect = trackor.object_detect:main'
        ],
    },
    # 新增：强制指定可执行文件的Python解释器路径
    options={
        'build_scripts': {
            'executable': sys.executable,
        },
    },
    
)
