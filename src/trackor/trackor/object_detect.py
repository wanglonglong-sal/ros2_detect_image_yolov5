import rclpy
from rclpy.node import Node
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)
from geometry_msgs.msg import PoseWithCovariance
from trackor.sort import Sort
import numpy as np

class ObjectTrackerNode(Node):
    def __init__(self):
        super().__init__('object_tracker')
        self.tracker = Sort()

        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.publisher = self.create_publisher(
            Detection2DArray,
            '/tracked_objects',
            10
        )

        self.get_logger().info('ObjectTrackerNode 已启动，等待检测结果...')

    def detection_callback(self, msg: Detection2DArray):
        dets = []

        for det in msg.detections:
            x = det.bbox.center.x
            y = det.bbox.center.y
            w = det.bbox.size_x
            h = det.bbox.size_y
            score = det.results[0].score if det.results else 1.0
            x1 = x - w / 2
            y1 = y - h / 2
            x2 = x + w / 2
            y2 = y + h / 2
            dets.append([x1, y1, x2, y2, score])

        if len(dets) == 0:
            return

        # 转换为 numpy 输入到 tracker
        dets_np = np.array(dets)
        tracks = self.tracker.update(dets_np)

        # 发布跟踪后的结果
        tracked_msg = Detection2DArray()
        tracked_msg.header = msg.header

        for track in tracks:
            x1, y1, x2, y2, track_id = track
            bbox = Detection2D()
            bbox.bbox.center.x = float((x1 + x2) / 2)
            bbox.bbox.center.y = float((y1 + y2) / 2)
            bbox.bbox.center.theta = 0.0
            bbox.bbox.size_x = float(x2 - x1)
            bbox.bbox.size_y = float(y2 - y1)

            hypothesis = ObjectHypothesis()
            hypothesis.id = int(track_id)
            hypothesis.score = 1.0  # 跟踪置信度可设为 1.0 或从检测中继承

            ohwp = ObjectHypothesisWithPose()
            ohwp.hypothesis = hypothesis

            pwc = PoseWithCovariance()
            pwc.pose.position.x = bbox.bbox.center.x
            pwc.pose.position.y = bbox.bbox.center.y
            pwc.pose.orientation.w = 1.0
            ohwp.pose = pwc

            bbox.results.append(ohwp)
            tracked_msg.detections.append(bbox)

        self.publisher.publish(tracked_msg)
        self.get_logger().info(f'发布跟踪目标数量: {len(tracked_msg.detections)}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
