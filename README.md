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

# Common commands
build the whole project: colcon build
run launch: ros2 launch bringup launch.py
-launch.py will activate the node of v4l2 and camera_subscriber(this node has no further usage)
enter python virtual environment: source ~/ros2_env/bin/activate
run python node: ros2 run detect mask_detect
