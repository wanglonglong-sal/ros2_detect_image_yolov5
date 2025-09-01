import os
import sys
import yaml
import cv2
import numpy as np
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseWithCovariance
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Pose2D,
)


# COCO class names used to map class IDs to human-readable labels
COCO_CLASS_NAMES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
    'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
    'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
    'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
    'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
    'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
    'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
    'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
    'toothbrush'
]

# BGR colors for selected classes
CLASS_COLORS = {
    'person': (0, 255, 0),
    'bicycle': (255, 0, 0),
    'car': (0, 0, 255),
    'motorcycle': (255, 0, 255),
    'bus': (0, 255, 255),
    'truck': (255, 255, 0),
}


class YoloV5OnnxSubscriber(Node):
    """Read frames from a video and publish detections and images."""

    def __init__(self):
        super().__init__('yolov5_onnx_subscriber')
        self.bridge = CvBridge()

        def open_video(path: str):
            """Try multiple OpenCV backends to open a video file."""
            backends = [
                (cv2.CAP_FFMPEG, 'CAP_FFMPEG'),
                (cv2.CAP_GSTREAMER, 'CAP_GSTREAMER'),
                (cv2.CAP_ANY, 'CAP_ANY'),
            ]
            for backend, name in backends:
                cap = cv2.VideoCapture(path, backend)
                if cap.isOpened():
                    self.get_logger().info(f"Opened video with backend {name}: {path}")
                    return cap
                cap.release()
            raise RuntimeError(f'Failed to open video file: {path}')

        # Load config file
        default_cfg_path = os.path.join(
            get_package_share_directory('detect'),
            'config',
            'yolov5_video_detector.yaml',
        )
        self.declare_parameter('config_path', default_cfg_path)
        cfg_path = self.get_parameter('config_path').get_parameter_value().string_value
        try:
            with open(cfg_path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().fatal(f"Failed to load config: {cfg_path} ({exc})")
            sys.exit(1)

        # Parameters
        self.declare_parameter('model_path', cfg.get('model_path', ''))
        self.declare_parameter('ignore_ratio', cfg.get('ignore_ratio', 0.25))
        self.declare_parameter('enable_output_video', cfg.get('enable_output_video', True))
        self.declare_parameter('allowed_classes', cfg.get('allowed_classes', []))

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.ignore_ratio = float(self.get_parameter('ignore_ratio').value)
        self.enable_output_video = bool(self.get_parameter('enable_output_video').value)

        # Allowed classes mapping (names -> COCO ids)
        allowed_param = self.get_parameter('allowed_classes').value
        if isinstance(allowed_param, (list, tuple)):
            allowed_names = [str(x).strip().lower() for x in allowed_param]
        elif isinstance(allowed_param, str):
            allowed_names = [s.strip().lower() for s in allowed_param.split(',') if s.strip()]
        else:
            allowed_names = []
        self.allowed_class_ids = set()
        if allowed_names:
            name_to_id = {n.lower(): i for i, n in enumerate(COCO_CLASS_NAMES)}
            for n in allowed_names:
                if n in name_to_id:
                    self.allowed_class_ids.add(name_to_id[n])
                else:
                    self.get_logger().warn(f"Unrecognized class name '{n}', ignore this entry")
            self.get_logger().info(f"Enabled class filter (IDs): {sorted(list(self.allowed_class_ids))}")

        # Prepare model
        model_path = os.path.abspath(os.path.expanduser(model_path))
        if not os.path.exists(model_path):
            self.get_logger().fatal(f"Model file not found: {model_path}")
            sys.exit(1)
        try:
            self.session = ort.InferenceSession(
                model_path,
                providers=['CUDAExecutionProvider', 'CPUExecutionProvider'],
            )
            self.get_logger().info(f'ONNX model loaded: {model_path}')
            self.get_logger().info(f"ONNX Runtime providers: {self.session.get_providers()}")
        except Exception as e:
            self.get_logger().fatal(f'Failed to load ONNX model: {str(e)}')
            sys.exit(1)

        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info(f"Model input name: {self.input_name}")

        # Open video
        video_path = cfg.get('input_video_path')
        if not video_path:
            self.get_logger().fatal('Missing input_video_path in config')
            sys.exit(1)
        try:
            self.cap = open_video(video_path)
        except Exception as exc:
            self.get_logger().fatal(str(exc))
            sys.exit(1)

        # Optional writer
        self.writer = None
        if self.enable_output_video:
            output_path = cfg.get('output_video_path')
            if not output_path:
                self.get_logger().fatal('Missing output_video_path in config')
                sys.exit(1)
            fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
            width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
            if not self.writer.isOpened():
                self.get_logger().error(f"Failed to create output video: {output_path}")

        # Publishers
        self.image_pub = self.create_publisher(Image, '/image_raw', 10)
        self.det_pub = self.create_publisher(Detection2DArray, '/detections', 10)

        # Timer
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
            cls_id = int(np.argmax(class_conf))
            score = float(conf * class_conf[cls_id])
            if score < conf_thres:
                continue

            x, y, w, h = pred[0:4]
            x1 = (x - w / 2) * orig_shape[1] / img_shape[1]
            y1 = (y - h / 2) * orig_shape[0] / img_shape[0]
            x2 = (x + w / 2) * orig_shape[1] / img_shape[1]
            y2 = (y + h / 2) * orig_shape[0] / img_shape[0]

            boxes.append([int(x1), int(y1), int(x2), int(y2)])
            scores.append(score)
            class_ids.append(cls_id)

        # NMS
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
            label = COCO_CLASS_NAMES[cls_id] if 0 <= cls_id < len(COCO_CLASS_NAMES) else str(cls_id)
            color = CLASS_COLORS.get(label, (255, 255, 255))
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            y_text = max(0, y1 - 5)
            cv2.putText(image, f"{label}:{score:.2f}", (x1, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
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
            label = COCO_CLASS_NAMES[cls_id] if 0 <= cls_id < len(COCO_CLASS_NAMES) else str(cls_id)
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
            self.get_logger().info('Video stream finished')
            if getattr(self, 'writer', None) is not None:
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

            # Filter detections in the bottom area and by allowed classes (if configured)
            h = frame.shape[0]
            filtered = []
            for box, score, cls in zip(boxes, scores, class_ids):
                if box[1] > h * (1 - self.ignore_ratio):
                    continue
                if self.allowed_class_ids and cls not in self.allowed_class_ids:
                    continue
                filtered.append((box, score, cls))
            boxes, scores, class_ids = (map(list, zip(*filtered)) if filtered else ([], [], []))

            draw_img = self.draw_detections(frame.copy(), boxes, scores, class_ids)
            if getattr(self, 'writer', None) is not None and self.enable_output_video:
                self.writer.write(draw_img)
            self.publish_detections(boxes, scores, class_ids, orig_shape)
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')


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
        if getattr(node, 'writer', None) is not None:
            node.writer.release()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

