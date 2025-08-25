import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import onnxruntime as ort
import numpy as np
import cv2
import os
import sys
from geometry_msgs.msg import PoseWithCovariance
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Pose2D,
)


CLASS_NAMES = ['no_mask', 'mask']  # 0->no_mask, 1->mask（按你的训练集顺序来）

VIDEO_PATH = '/mnt/d/Dataset/City/CityWay_part2.mp4'


class YoloV5OnnxSubscriber(Node):
    """Read frames from a fixed video file and publish detections."""

    def __init__(self):
        super().__init__('yolov5_onnx_subscriber')
        self.bridge = CvBridge()

        # 模型路径
        model_path = os.path.expanduser('~/Learn_ROS2/03.my_robot/src/detect/model/best.onnx')
        model_path = os.path.abspath(model_path)

        if not os.path.exists(model_path):
            self.get_logger().fatal(f"模型文件不存在: {model_path}")
            sys.exit(1)

        # 加载 ONNX 模型
        try:
            self.session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
            self.get_logger().info(f'ONNX 模型加载成功: {model_path}')
        except Exception as e:
            self.get_logger().fatal(f'ONNX 模型加载失败: {str(e)}')
            sys.exit(1)

        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info(f"模型输入名称: {self.input_name}")

        # 视频读取与发布器
        self.cap = cv2.VideoCapture(VIDEO_PATH)
        if not self.cap.isOpened():
            self.get_logger().fatal(f'无法打开视频文件: {VIDEO_PATH}')
            sys.exit(1)

        # 声明输出视频路径参数并初始化写入器
        self.declare_parameter(
            'output_video_path', '/mnt/d/Dataset/Output/detect_output.mp4'
        )
        output_path = (
            self.get_parameter('output_video_path').get_parameter_value().string_value
        )
        fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

        self.image_pub = self.create_publisher(Image, '/image_raw', 10)
        self.det_pub = self.create_publisher(Detection2DArray, '/detections', 10)

        # 定时器循环读取视频帧
        self.timer = self.create_timer(1 / 30.0, self.timer_callback)

    def preprocess(self, image):
        img_resized = cv2.resize(image, (640, 640))
        img_input = img_resized[:, :, ::-1].transpose(2, 0, 1)
        img_input = np.expand_dims(img_input, 0).astype(np.float32) / 255.0
        return img_input, img_resized

    def postprocess(self, outputs, img_shape, orig_shape, conf_thres=0.25, iou_thres=0.45):
        preds = outputs[0][0]
        boxes, scores, class_ids = [], [], []

        for pred in preds:
            conf = pred[4]
            if conf < conf_thres:
                continue
            class_conf = pred[5:]
            cls_id = np.argmax(class_conf)
            score = conf * class_conf[cls_id]
            if score < conf_thres:
                continue

            x, y, w, h = pred[0:4]
            x1 = (x - w / 2) * orig_shape[1] / img_shape[1]
            y1 = (y - h / 2) * orig_shape[0] / img_shape[0]
            x2 = (x + w / 2) * orig_shape[1] / img_shape[1]
            y2 = (y + h / 2) * orig_shape[0] / img_shape[0]

            boxes.append([int(x1), int(y1), int(x2), int(y2)])
            scores.append(float(score))
            class_ids.append(cls_id)

        idxs = cv2.dnn.NMSBoxes(boxes, scores, conf_thres, iou_thres)
        final_boxes, final_scores, final_classes = [], [], []
        if len(idxs) > 0:
            for i in idxs.flatten():
                final_boxes.append(boxes[i])
                final_scores.append(scores[i])
                final_classes.append(class_ids[i])

        return final_boxes, final_scores, final_classes

    def draw_detections(self, image, boxes, scores, class_ids):
        for (box, score, cls_id) in zip(boxes, scores, class_ids):
            x1, y1, x2, y2 = box
            label = CLASS_NAMES[cls_id] if 0 <= cls_id < len(CLASS_NAMES) else str(cls_id)
            color = (0, 0, 255) if label == 'no_mask' else (0, 255, 0)
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            y_text = max(0, y1 - 5)
            cv2.putText(image, f"{label}:{score:.2f}", (x1, y_text),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return image

    def publish_detections(self, boxes, scores, class_ids, img_shape):
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'

        for box, score, cls_id in zip(boxes, scores, class_ids):
            detection = Detection2D()

            x1, y1, x2, y2 = box
            center2d = Pose2D()
            center2d.position.x = (x1 + x2) / 2.0
            center2d.position.y = (y1 + y2) / 2.0
            center2d.theta = 0.0
            detection.bbox.center = center2d

            detection.bbox.size_x = float(x2 - x1)
            detection.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesis()
            label = CLASS_NAMES[cls_id] if 0 <= cls_id < len(CLASS_NAMES) else str(cls_id)
            hyp.class_id = label
            hyp.score = float(score)

            ohwp = ObjectHypothesisWithPose()
            ohwp.hypothesis = hyp

            pwc = PoseWithCovariance()
            pwc.pose.position.x = center2d.position.x
            pwc.pose.position.y = center2d.position.y
            pwc.pose.position.z = 0.0
            pwc.pose.orientation.w = 1.0
            ohwp.pose = pwc

            detection.results.append(ohwp)
            msg.detections.append(detection)

        self.det_pub.publish(msg)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('视频读取完毕')
            if hasattr(self, 'writer'):
                self.writer.release()
            self.destroy_node()
            rclpy.shutdown()
            return

        orig_shape = frame.shape[:2]
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(img_msg)

        try:
            img_input, img_resized = self.preprocess(frame)
            outputs = self.session.run(None, {self.input_name: img_input})
            boxes, scores, class_ids = self.postprocess(outputs, img_resized.shape, orig_shape)
            draw_img = self.draw_detections(frame.copy(), boxes, scores, class_ids)
            if hasattr(self, 'writer'):
                self.writer.write(draw_img)
            self.publish_detections(boxes, scores, class_ids, orig_shape)
        except Exception as e:
            self.get_logger().error(f'处理视频帧时出错: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloV5OnnxSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if hasattr(node, 'cap'):
            node.cap.release()
        if hasattr(node, 'writer'):
            node.writer.release()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
