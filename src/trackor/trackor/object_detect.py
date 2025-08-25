import rclpy
from rclpy.node import Node
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)
from geometry_msgs.msg import PoseWithCovariance
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from deep_sort_realtime.deepsort_tracker import DeepSort

class ObjectTrackerNode(Node):
    def __init__(self):
        super().__init__('object_tracker')
        self.tracker = DeepSort(max_age=30, n_init=3, nn_budget=100)
        self.bridge = CvBridge()
        self.last_image = None

        self.declare_parameter(
            'output_video_path', '/mnt/d/Dataset/Output/tracked_output.mp4'
        )
        self.declare_parameter('output_fps', 30.0)
        self.output_path = (
            self.get_parameter('output_video_path').get_parameter_value().string_value
        )
        self.output_fps = (
            self.get_parameter('output_fps').get_parameter_value().double_value
        )
        self.writer = None

        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            Detection2DArray,
            '/tracked_objects',
            10
        )

        self.get_logger().info('ObjectTrackerNode 已启动，等待检测结果...')

    def image_callback(self, msg: Image):
        self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def detection_callback(self, msg: Detection2DArray):
        dets = []
        det_info = []

        for det in msg.detections:
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y
            w = det.bbox.size_x
            h = det.bbox.size_y
            score = det.results[0].hypothesis.score if det.results else 1.0
            label = det.results[0].hypothesis.class_id if det.results else ''
            x1 = x - w / 2
            y1 = y - h / 2
            x2 = x1 + w
            y2 = y1 + h
            bbox = [x1, y1, x2, y2]
            dets.append((bbox, score, label))
            det_info.append((bbox, label, score))

        if len(dets) == 0 or self.last_image is None:
            return

        tracks = self.tracker.update_tracks(dets, frame=self.last_image)

        tracked_msg = Detection2DArray()
        tracked_msg.header = msg.header

        draw_img = self.last_image.copy() if self.last_image is not None else None

        for track in tracks:
            if not track.is_confirmed():
                continue
            x1, y1, x2, y2 = track.to_ltrb()
            track_id = track.track_id

            # match detection to track via IoU
            best_iou = 0.0
            best_label = ''
            best_score = 0.0
            for det_box, label, score in det_info:
                dx1, dy1, dx2, dy2 = det_box
                inter_x1 = max(x1, dx1)
                inter_y1 = max(y1, dy1)
                inter_x2 = min(x2, dx2)
                inter_y2 = min(y2, dy2)
                inter_area = max(0.0, inter_x2 - inter_x1) * max(0.0, inter_y2 - inter_y1)
                box_area = (x2 - x1) * (y2 - y1)
                det_area = (dx2 - dx1) * (dy2 - dy1)
                union = box_area + det_area - inter_area + 1e-6
                iou = inter_area / union
                if iou > best_iou:
                    best_iou = iou
                    best_label = label
                    best_score = score

            bbox = Detection2D()

            bbox.bbox.center.position.x = float((x1 + x2) / 2)
            bbox.bbox.center.position.y = float((y1 + y2) / 2)
            bbox.bbox.center.theta = 0.0
            bbox.bbox.size_x = float(x2 - x1)
            bbox.bbox.size_y = float(y2 - y1)


            hyp_cls = ObjectHypothesis()
            hyp_cls.class_id = best_label
            hyp_cls.score = float(best_score)
            ohwp_cls = ObjectHypothesisWithPose()
            ohwp_cls.hypothesis = hyp_cls


            pwc = PoseWithCovariance()
            pwc.pose.position.x = bbox.bbox.center.position.x
            pwc.pose.position.y = bbox.bbox.center.position.y

            pwc.pose.orientation.w = 1.0
            ohwp_cls.pose = pwc

            bbox.results.append(ohwp_cls)

            hyp_id = ObjectHypothesis()
            hyp_id.class_id = str(int(track_id))
            hyp_id.score = float(best_score)
            ohwp_id = ObjectHypothesisWithPose()
            ohwp_id.hypothesis = hyp_id
            ohwp_id.pose = pwc
            bbox.results.append(ohwp_id)


            tracked_msg.detections.append(bbox)

            if draw_img is not None:
                cv2.rectangle(draw_img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                text = f"ID:{int(track_id)} {best_label}:{best_score:.2f}"
                cv2.putText(draw_img, text, (int(x1), max(0, int(y1) - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        self.publisher.publish(tracked_msg)
        self.get_logger().info(f'发布跟踪目标数量: {len(tracked_msg.detections)}')
        if draw_img is not None:
            if self.writer is None:
                h, w = draw_img.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.writer = cv2.VideoWriter(
                    self.output_path, fourcc, self.output_fps, (w, h)
                )
            self.writer.write(draw_img)

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
