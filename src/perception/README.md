# 20250730 
Today, I am going to rewrite a whole project with a standard catalog structure, as follows:
---------------------------------------------------------------

ros2_ws/
├── src/
│
│   ├── bringup/                      # 🔧 启动模块：统一启动所有功能包
│   │   ├── launch/
│   │   │   ├── bringup.launch.py     # 启动全部节点
│   │   │   └── rviz_config.rviz
│   │   ├── config/
│   │   │   └── robot_params.yaml     # 全局参数配置
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── README.md
│
│   ├── interfaces/                   # 📦 自定义接口定义（消息、服务、动作）
│   │   ├── msg/
│   │   │   └── ObjectInfo.msg
│   │   ├── srv/
│   │   │   └── SetMode.srv
│   │   ├── action/
│   │   │   └── MoveTo.action
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── README.md
│
│   ├── perception/                  # 👁️ 感知模块：视觉、识别、SLAM
│   │   ├── include/perception/
│   │   │   └── object_detector.hpp
│   │   ├── src/
│   │   │   ├── camera_node.cpp
│   │   │   ├── object_detector.cpp
│   │   │   └── slam_node.cpp
│   │   ├── launch/
│   │   │   └── perception.launch.py
│   │   ├── config/
│   │   │   └── perception_params.yaml
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── README.md
│
│   ├── navigation/                  # 🧭 导航模块：地图、定位、路径规划
│   │   ├── include/navigation/
│   │   ├── src/
│   │   │   ├── map_server.cpp
│   │   │   ├── localization_node.cpp
│   │   │   └── planner.cpp
│   │   ├── launch/
│   │   │   └── navigation.launch.py
│   │   ├── config/
│   │   │   └── nav_params.yaml
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── README.md
│
│   ├── control/                     # ⚙️ 控制模块：速度/位置控制、PID
│   │   ├── include/control/
│   │   │   └── pid_controller.hpp
│   │   ├── src/
│   │   │   ├── velocity_controller.cpp
│   │   │   └── pid_controller.cpp
│   │   ├── launch/
│   │   │   └── control.launch.py
│   │   ├── config/
│   │   │   └── control_params.yaml
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── README.md
│
│   ├── robot_description/          # 🤖 机器人模型（URDF、网格文件等）
│   │   ├── urdf/
│   │   │   └── robot.urdf.xacro
│   │   ├── meshes/
│   │   │   └── base_link.stl
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── README.md
│
│   ├── robot_state/                # 📡 TF 广播、状态发布
│   │   ├── src/
│   │   │   └── state_publisher.cpp
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── README.md
│
│   ├── service_nodes/              # 🛠️ 服务节点集合（如重置、状态获取）
│   │   ├── src/
│   │   │   ├── diagnostics_server.cpp
│   │   │   └── reset_pose_server.cpp
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── README.md
│
│   ├── tools/                      # 🧪 工具包：测试、仿真、调试脚本
│   │   ├── dummy_publisher.cpp
│   │   ├── rviz_plugins/
│   │   ├── bag_tools/
│   │   ├── launch/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── README.md
│
│   └── utils/                      # 🔧 公共工具函数 / 通用类库
│       ├── include/utils/
│       │   └── math_utils.hpp
│       ├── src/
│       │   └── math_utils.cpp
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── README.md
│
├── install/                        # ⚙️ colcon 构建生成目录（自动生成）
├── build/
├── log/
└── colcon.meta                     # （可选）colcon 构建配置文件

-------------------------------
When I rewrite the perception's camera publisher, I found that we use cv::VideoCapture to gain the images from the camera. The process will generate a image with opencv formation. And if you want to publish them by using the rclcpp(ros2), you have to change the opencv image formation to ros2 image formation, which is the <sensor_msgs::msg::Image> by using cv_brdige. The code is:
auto msg = cv_brige::CvImage(header, "bgr8", image).toImageMsg();
I just perplex about the usage of the cv_brdige. After searche, I found it is a brdige between opencv and ros2 image. Finally, I found that it is no neccessary to gain the frame from camera by opencv's VideoCpature. Only uusing the driver of camera from ros2 is ok. 
-----------------------
So, now I would like to try install the tool of v4l2. When I success to configurate the ros2 camera driver, there is no necessary to write a camera publisher programma.
-----------------------

This is the step of configuration of the ros2 camera driver:
这是 v4l-utils 提供的工具，可以用来操作和检测 V4L2 设备。

运行：
v4l2-ctl --version
如果输出版本号（如 utils version 1.22.1），说明已安装。

如果提示 command not found：
那就安装它：

sudo apt update
sudo apt install v4l-utils

✅ 2. 查看摄像头是否被识别（设备文件）
运行：
bash
复制
编辑
ls /dev/video*
输出示例：

bash
复制
编辑
/dev/video0
如果有 /dev/video0、/dev/video1 等，说明摄像头设备已被系统识别，且支持 V4L2。

3. install the tool
sudo apt update
sudo apt install ros-jazzy-v4l2-camera

4. ros2 run v4l2_camera v4l2_camera_node
这会启动一个摄像头节点，默认发布话题：/image_raw



✅ 检查话题发布是否成功：
bash
复制
编辑
ros2 topic list
你应该会看到 /image_raw，然后可以再查看消息：

bash
复制
编辑
ros2 topic echo /image_raw
或者用 rqt_image_view 可视化（如果已安装）：

bash
复制
编辑
ros2 run rqt_image_view rqt_image_view

# 20250730 try to use the launch file to start the node of v4l2_camera_node, there are some comments:

1. create launch file had better use the function of create package. the command is:
ros2 pkg create bringup --build-type ament_cmake --dependencies launch_ros

2. make a standard folder by hand. 
    cd ~/Learn_ROS2/03.my_robot
    cd src
    cd bringup
    mkdir launch
Finally, new a file of launch.py

3. edit the launch file:
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_ctl_camera',
            executable='v4l2_ctl_camera_node',
            name='v4l2_ctl_camera',
            output='screen',
            parameters=[{
                'camera_id': 0,  # Optional parameter, customize as needed
                'frame_rate': 30
            }]
        )
    ])

4. cofigurate the CMakeList.txt
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

5. cofigurate the package.xml
  <exec_depend>launch_ros</exec_depend>
  delete other description of launch_ros

6. rebuild the project

7. ros2 launch bringup launch.py

# After this, we found another problem, which is v4l2 as a 3rd tool need to be compile to the 03.my_robot project (better)

# 2025-07-31

1. Though the website of gitlab to download the source of v4l2(Video4Linux2)
你可以通过以下方式将源码克隆到 src/ 内部并编译：

bash
复制
编辑
cd ~/Learn_ROS2/03.my_robot/src
git clone --branch jazzy https://gitlab.com/boldhearts/ros2_v4l2_camera.git v4l2_camera

2. 然后继续执行：

bash
复制
编辑
cd ~/Learn_ROS2/03.my_robot
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash


rosdep install --from-paths src --ignore-src -r -y means try to download all the dependencies 

3. check the image/raw is on track by ros2 topic list or ros2 run rqt_image_view rqt_image_view



