import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import onnxruntime as ort
import numpy as np
import cv2
import os
import sys

CLASS_NAMES = ['no_mask', 'mask']  # 0->no_mask, 1->mask（按你的训练集顺序来）

class YoloV5OnnxSubscriber(Node):
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

        # 获取输入输出信息
        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info(f"模型输入名称: {self.input_name}")

        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            109
        )

    def preprocess(self, image):
        """Resize + 归一化 + NCHW"""
        img_resized = cv2.resize(image, (640, 640))
        img_input = img_resized[:, :, ::-1].transpose(2, 0, 1)  # BGR->RGB, HWC->CHW
        img_input = np.expand_dims(img_input, 0).astype(np.float32) / 255.0
        return img_input, img_resized

    def postprocess(self, outputs, img_shape, orig_shape, conf_thres=0.25, iou_thres=0.45):
        """YOLOv5 ONNX 输出后处理"""
        # 输出一般 shape = (1, num_boxes, 85)，最后85=xywh+obj+num_classes
        preds = outputs[0][0]  # (num_boxes, 85)
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

            # xywh -> xyxy
            x, y, w, h = pred[0:4]
            x1 = (x - w / 2) * orig_shape[1] / img_shape[1]
            y1 = (y - h / 2) * orig_shape[0] / img_shape[0]
            x2 = (x + w / 2) * orig_shape[1] / img_shape[1]
            y2 = (y + h / 2) * orig_shape[0] / img_shape[0]

            boxes.append([int(x1), int(y1), int(x2), int(y2)])
            scores.append(float(score))
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

            # 可选：不同类别不同颜色
            label = CLASS_NAMES[cls_id] if 0 <= cls_id < len(CLASS_NAMES) else str(cls_id)
            color = (0, 0, 255) if label == 'no_mask' else (0, 255, 0)  # 红色=未戴口罩，绿色=戴口罩

            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            y_text = max(0, y1 - 5)
            cv2.putText(image, f"{label}:{score:.2f}",
                        (x1, y_text), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, color, 2)
        return image

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            orig_shape = cv_image.shape[:2]  # 原始图像尺寸

            # 前处理
            img_input, img_resized = self.preprocess(cv_image)

            # 推理
            outputs = self.session.run(None, {self.input_name: img_input})

            # 后处理
            boxes, scores, class_ids = self.postprocess(outputs, img_resized.shape, orig_shape)

            # 可视化
            result_img = self.draw_detections(cv_image.copy(), boxes, scores, class_ids)

            cv2.imshow("YOLOv5 ONNX Detection", result_img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloV5OnnxSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
