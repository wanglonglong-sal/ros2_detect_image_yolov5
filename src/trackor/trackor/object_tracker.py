import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.callback_groups import ReentrantCallbackGroup
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
import threading
from deep_sort_realtime.deepsort_tracker import DeepSort
from typing import Optional, Tuple, List
import numpy as np
from .sort import Sort
import time


# 不同交通参与者的颜色映射（BGR）
CLASS_COLORS = {
    'person': (0, 255, 0),
    'bicycle': (255, 0, 0),
    'car': (0, 0, 255),
    'motorcycle': (255, 0, 255),
    'bus': (0, 255, 255),
    'truck': (255, 255, 0),
}

class ObjectTrackerNode(Node):
    def __init__(self):
        super().__init__('object_tracker')
        # Allow callbacks to run concurrently within this node
        self.cb_group = ReentrantCallbackGroup()
        # Backend selection: 'deepsort' (default) or 'sort'
        self.declare_parameter('tracker_backend', 'deepsort')
        self.tracker_backend = self.get_parameter('tracker_backend').get_parameter_value().string_value
        if self.tracker_backend.lower() == 'sort':
            self.tracker = Sort(max_age=15, min_hits=3, iou_threshold=0.3)
            self.get_logger().info('Tracker backend: SORT (no re-id)')
        else:
            # DeepSort configuration (try GPU/half if supported)
            try:
                self.tracker = DeepSort(max_age=30, n_init=3, nn_budget=50, embedder_gpu=True, half=True)
            except TypeError:
                self.tracker = DeepSort(max_age=30, n_init=3, nn_budget=50)
            self.get_logger().info('Tracker backend: DeepSort')
        self.bridge = CvBridge()
        self.last_image = None
        self._prev_img_time_sec = None
        self._estimated_fps = 0.0

        self.declare_parameter(
            'output_video_path', '/mnt/d/Dataset/Output/tracked_output.mp4'
        )
        self.declare_parameter('output_fps', 30.0)
        self.declare_parameter('process_hz', 10.0)
        self.declare_parameter('min_det_score', 0.40)
        self.declare_parameter('top_k', 10)
        self.declare_parameter('allowed_labels', ['person','bicycle','car','motorcycle','bus','truck'])
        self.output_path = (
            self.get_parameter('output_video_path').get_parameter_value().string_value
        )
        self.output_fps = (
            self.get_parameter('output_fps').get_parameter_value().double_value
        )
        self.process_hz = (
            self.get_parameter('process_hz').get_parameter_value().double_value
            if hasattr(self.get_parameter('process_hz'), 'get_parameter_value') else 10.0
        )
        self.min_det_score = float(self.get_parameter('min_det_score').get_parameter_value().double_value if hasattr(self.get_parameter('min_det_score'), 'get_parameter_value') else 0.40)
        self.top_k = int(self.get_parameter('top_k').get_parameter_value().integer_value if hasattr(self.get_parameter('top_k'), 'get_parameter_value') else 10)
        try:
            self.allowed_labels = [str(x) for x in self.get_parameter('allowed_labels').value]
        except Exception:
            self.allowed_labels = ['person','bicycle','car','motorcycle','bus','truck']
        # Detailed logging switch
        self.declare_parameter('log_detail', True)
        try:
            self.log_detail = bool(self.get_parameter('log_detail').get_parameter_value().bool_value)
        except Exception:
            self.log_detail = True
        self.writer = None
        self.declare_parameter('enable_output_video', True)
        self.enable_output_video = (
            self.get_parameter('enable_output_video').get_parameter_value().bool_value
            if hasattr(self.get_parameter('enable_output_video'), 'get_parameter_value')
            else True
        )

        # QoS: for real-time, keep detection queue shallow to avoid latency buildup
        det_qos = QoSProfile(depth=10)
        det_qos.history = QoSHistoryPolicy.KEEP_LAST
        det_qos.reliability = QoSReliabilityPolicy.RELIABLE

        img_qos = qos_profile_sensor_data

        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            det_qos,
            callback_group=self.cb_group,
        )
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            img_qos,
            callback_group=self.cb_group,
        )

        pub_qos = QoSProfile(depth=100)
        pub_qos.history = QoSHistoryPolicy.KEEP_LAST
        pub_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.publisher = self.create_publisher(
            Detection2DArray,
            '/tracked_objects',
            pub_qos,
        )

        self.get_logger().info('ObjectTrackerNode 已启动，等待检测结果...')
        # Protect VideoWriter from concurrent access
        self._writer_lock = threading.Lock()
        # Protect DeepSort internal state from concurrent access
        self._tracker_lock = threading.Lock()

        # Debug/heartbeat counters
        self._img_count = 0
        self._det_count = 0
        self._det_skipped_no_image = 0
        self._pub_count = 0
        self._det_overwrite_count = 0

        # Heartbeat timer for connectivity & rates overview
        self._hb_timer = self.create_timer(2.0, self._heartbeat)

        # Latest detections buffer (drop-oldest, keep-latest)
        self._latest_dets: Optional[Tuple[List[Tuple[List[float], float, str]], List[Tuple[List[float], str, float]], object]] = None
        self._pending_lock = threading.Lock()
        # Processing timer at fixed rate to avoid backlog
        period = 1.0 / max(0.1, float(self.process_hz))
        self._proc_timer = self.create_timer(period, self._process_tick, callback_group=self.cb_group)
        # In-flight guard to avoid overlapping processing
        self._proc_inflight = False

    def image_callback(self, msg: Image):
        # Update last image and estimate FPS from image timestamps
        self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._img_count += 1
        if self.log_detail and (self._img_count % 30 == 1):
            self.get_logger().info(f'[img] received #{self._img_count}: size={self.last_image.shape[1]}x{self.last_image.shape[0]}')
        try:
            stamp = msg.header.stamp
            t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            if self._prev_img_time_sec is not None:
                dt = t - self._prev_img_time_sec
                if dt > 1e-3:
                    inst_fps = 1.0 / dt
                    if self._estimated_fps <= 0:
                        self._estimated_fps = inst_fps
                    else:
                        # simple EMA to smooth FPS estimate
                        self._estimated_fps = 0.9 * self._estimated_fps + 0.1 * inst_fps
            self._prev_img_time_sec = t
        except Exception:
            pass

    def detection_callback(self, msg: Detection2DArray):
        self._det_count += 1
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
            bbox = [x1, y1, w, h]
            dets.append((bbox, score, label))
            det_info.append(([x1, y1, x2, y2], label, score))

        # Cache latest detections and return quickly; heavy work in timer
        with self._pending_lock:
            if self._latest_dets is not None:
                self._det_overwrite_count += 1
            self._latest_dets = (dets, det_info, msg.header)
        if self.log_detail:
            self.get_logger().info(f'[det] msg with {len(dets)} detections received; overwrites:{self._det_overwrite_count}')
        return

        # Require an image to proceed; allow empty detections to still publish/write
        if self.last_image is None:
            self._det_skipped_no_image += 1
            if (self._det_skipped_no_image % 50) == 1:
                self.get_logger().warn('检测到达但尚未收到图像，已跳过多次。请检查 /image_raw 频率与 QoS.')
            return

        # DeepSort is not thread-safe; guard updates
        with self._tracker_lock:
            if self.tracker_backend.lower() == 'sort':
                # SORT expects ndarray Nx5 [x1,y1,x2,y2,score]
                if dets:
                    det_np = np.array([[b[0], b[1], b[0]+b[2], b[1]+b[3], s] for (b,s,_) in dets], dtype=np.float32)
                else:
                    det_np = np.empty((0,5), dtype=np.float32)
                tracks_mat = self.tracker.update(dets=det_np)
                # Convert to unified list format for downstream drawing/output
                tracks = []
                for t in tracks_mat:
                    x1, y1, x2, y2, tid = t
                    class TrackLike:
                        def __init__(self, l, t, r, b, tid):
                            self._ltrb = (float(l), float(t), float(r), float(b))
                            self.track_id = int(tid)
                        def is_confirmed(self):
                            return True
                        def to_ltrb(self):
                            return self._ltrb
                    tracks.append(TrackLike(x1, y1, x2, y2, tid))
            else:
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
                color = CLASS_COLORS.get(best_label, (255, 255, 255))
                cv2.rectangle(draw_img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                text = f"ID:{int(track_id)} {best_label}:{best_score:.2f}"
                cv2.putText(
                    draw_img,
                    text,
                    (int(x1), max(0, int(y1) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )

        self.publisher.publish(tracked_msg)
        self._pub_count += 1
        self.get_logger().info(f'发布跟踪目标数量: {len(tracked_msg.detections)}')
        if self.enable_output_video and draw_img is not None:
            with self._writer_lock:
                if self.writer is None:
                    h, w = draw_img.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    fps = float(self.output_fps) if float(self.output_fps) > 0 else (
                        float(self._estimated_fps) if self._estimated_fps > 0 else 30.0
                    )
                    self.writer = cv2.VideoWriter(self.output_path, fourcc, fps, (w, h))
                    if not self.writer.isOpened():
                        self.get_logger().error(f"Failed to open tracker output video: {self.output_path}")
                        self.writer = None
                if self.writer is not None:
                    self.writer.write(draw_img)

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
        super().destroy_node()

    def _heartbeat(self):
        try:
            img_pubs = self.image_sub.get_publisher_count() if self.image_sub else 0
            det_pubs = self.subscription.get_publisher_count() if self.subscription else 0
            tracked_subs = self.publisher.get_subscription_count() if self.publisher else 0
        except Exception:
            img_pubs = det_pubs = tracked_subs = 0

        self.get_logger().info(
            (
                f'心跳 | images:{self._img_count} det_msgs:{self._det_count} '
                f'skipped(no_image):{self._det_skipped_no_image} published:{self._pub_count} '
                f'est_fps:{self._estimated_fps:.2f} '
                f'pubs(/image_raw):{img_pubs} pubs(/detections):{det_pubs} '
                f'subs(/tracked_objects):{tracked_subs}'
            )
        )

    def _process_tick(self):
        # Fixed-rate processing; always tick to age-out tracks
        if self.last_image is None:
            return
        if self._proc_inflight:
            if self.log_detail:
                self.get_logger().info('[tick] previous processing still running; skip this tick')
            return
        self._proc_inflight = True
        with self._pending_lock:
            latest = self._latest_dets
            self._latest_dets = None
        if latest is None:
            dets, det_info, header = [], [], None
            if self.log_detail:
                self.get_logger().info('[tick] no pending detections; aging tracks only')
        else:
            dets, det_info, header = latest
            if self.log_detail:
                self.get_logger().info(f'[tick] processing pending detections: count_in={len(dets)}')

        # Filter detections by score/label and keep top-k
        dets_in = len(dets)
        if dets:
            idx_sorted = sorted(range(len(dets)), key=lambda i: dets[i][1], reverse=True)
            filtered = []
            filtered_info = []
            for i in idx_sorted:
                box, score, label = dets[i]
                if score < self.min_det_score:
                    continue
                if self.allowed_labels and label and label not in self.allowed_labels:
                    continue
                filtered.append((box, score, label))
                filtered_info.append(det_info[i])
                if len(filtered) >= max(1, self.top_k):
                    break
            dets, det_info = filtered, filtered_info
        if self.log_detail:
            self.get_logger().info(f'[tick] filtered detections: in={dets_in} out={len(dets)} (min_score={self.min_det_score}, top_k={self.top_k})')

        # DeepSort update with latency logging
        t0 = time.time()
        with self._tracker_lock:
            tracks = self.tracker.update_tracks(dets, frame=self.last_image)
        proc_ms = (time.time() - t0) * 1000.0
        if self.log_detail:
            # detection to tick latency if header available
            dt_ms = None
            try:
                if header is not None:
                    dt_ms = (self.get_clock().now().nanoseconds - (header.stamp.sec * 10**9 + header.stamp.nanosec)) / 1e6
            except Exception:
                dt_ms = None
            self.get_logger().info(f'[tick] deepsort proc={proc_ms:.1f}ms tracks={len(tracks)} det_to_tick_ms={dt_ms if dt_ms is not None else "n/a"}')

        tracked_msg = Detection2DArray()
        if header is not None:
            tracked_msg.header = header
        else:
            tracked_msg.header.stamp = self.get_clock().now().to_msg()
            tracked_msg.header.frame_id = 'camera_frame'

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
                color = CLASS_COLORS.get(best_label, (255, 255, 255))
                cv2.rectangle(draw_img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                text = f"ID:{int(track_id)} {best_label}:{best_score:.2f}"
                cv2.putText(
                    draw_img,
                    text,
                    (int(x1), max(0, int(y1) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )

        self.publisher.publish(tracked_msg)
        self._pub_count += 1
        if self.log_detail:
            self.get_logger().info(f'[pub] tracked objects: {len(tracked_msg.detections)}')
        self._proc_inflight = False
        if self.enable_output_video and draw_img is not None:
            with self._writer_lock:
                if self.writer is None:
                    h, w = draw_img.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    fps = float(self.output_fps) if float(self.output_fps) > 0 else (
                        float(self._estimated_fps) if self._estimated_fps > 0 else 30.0
                    )
                    self.writer = cv2.VideoWriter(self.output_path, fourcc, fps, (w, h))
                    if not self.writer.isOpened():
                        self.get_logger().error(f"Failed to open tracker output video: {self.output_path}")
                        self.writer = None
                if self.writer is not None:
                    self.writer.write(draw_img)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
