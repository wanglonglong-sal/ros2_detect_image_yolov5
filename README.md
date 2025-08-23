# ros2_detect_image_yolov5
This is a ros2 structure project and involve yolo-v5 to detect image of whether people wear a mask

# The data flow with each node
node:v4l2_camera_node
-scriber: the raw carema images
-publisher: /image_raw

node:mask_detect
-scriber: /image_raw
-publisher: 

node:mask_traker
-scriber: /detections : vision_msgs/Detection2DArray
-publisher: 

# Common commands about ros2
build the whole project: colcon build

run launch: ros2 launch bringup launch.py
-launch.py will activate the node of v4l2 and camera_subscriber(this node has no further usage)

enter python virtual environment: source ~/ros2_env/bin/activate
run python node: ros2 run detect mask_detect

create a new package: 
-cd src
-ros2 pkg create trackor --build-type ament_python --dependencies rclpy --node-name object_detect 

# git commands
# 从远程拉取所有分支和更新（不合并）
git fetch --all

# 查看所有本地分支
git branch

# 查看所有远程分支
git branch -r

# 查看本地分支和远程分支的绑定关系
git branch -vv

# 切换到本地已有分支
git checkout <分支名>

# 基于远程分支新建并切换到本地分支
git checkout -b <本地分支名> origin/<远程分支名>

# 示例：基于远程 feat/vision-msgs 新建本地分支
git checkout -b feat/vision-msgs origin/feat/vision-msgs

# 查看当前分支是否有关联远程分支
git status

# 如果没关联，可以手动建立 tracking
git branch --set-upstream-to=origin/<远程分支名> <本地分支名>

# 或在新建时直接建立 tracking（简写：-u = --track）
git checkout -b <本地分支名> -u origin/<远程分支名>


