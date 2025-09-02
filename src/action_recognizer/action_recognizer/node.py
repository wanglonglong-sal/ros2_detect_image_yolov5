import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from typing import Dict, Deque, Tuple, Optional
from collections import defaultdict, deque
import numpy as np
import cv2
import threading

from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)
from cv_bridge import CvBridge

from .backends import get_backend


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ActionRecognizerNode(Node):
    def __init__(self):
        super().__init__('action_recognizer')
        self.bridge = CvBridge()
        # Allow callbacks to run concurrently within this node
        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('backend', 'stub')
        self.declare_parameter('clip_len', 16)
        self.declare_parameter('sample_rate', 2)
        self.declare_parameter('inference_stride', 8)
        self.declare_parameter('min_box_size', 8)
        self.declare_parameter('allowed_actors', [
            'person', 'bicycle', 'motorcycle', 'car', 'bus', 'truck'
        ])
        self.declare_parameter('labels_path', '')
        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('multi_label', True)
        self.declare_parameter('alpha', 4)
        self.declare_parameter('input_size', 224)
        self.declare_parameter('output_topic', '/actions')
        self.declare_parameter('output_video_path', '/mnt/d/Dataset/Output/actions_output.mp4')
        self.declare_parameter('output_fps', 30.0)
        self.declare_parameter('enable_output_video', True)

        backend_name = self.get_parameter('backend').get_parameter_value().string_value
        self.clip_len = int(self.get_parameter('clip_len').get_parameter_value().integer_value)
        self.sample_rate = int(self.get_parameter('sample_rate').get_parameter_value().integer_value)
        self.inference_stride = int(self.get_parameter('inference_stride').get_parameter_value().integer_value)
        self.min_box_size = int(self.get_parameter('min_box_size').get_parameter_value().integer_value)
        self.allowed_actors = [
            str(s) for s in self.get_parameter('allowed_actors').value
        ]
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.output_video_path = self.get_parameter('output_video_path').get_parameter_value().string_value
        self.output_fps = float(self.get_parameter('output_fps').get_parameter_value().double_value)
        self.enable_output_video = (
            self.get_parameter('enable_output_video').get_parameter_value().bool_value
        )
        self.labels_path = self.get_parameter('labels_path').get_parameter_value().string_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.multi_label = bool(self.get_parameter('multi_label').get_parameter_value().bool_value)
        self.alpha = int(self.get_parameter('alpha').get_parameter_value().integer_value)
        self.input_size = int(self.get_parameter('input_size').get_parameter_value().integer_value)

        # Backend (stub for now, can expand to slowfast)
        try:
            self.backend = get_backend(
                backend_name,
                clip_len=self.clip_len,
                sample_rate=self.sample_rate,
                model_path=self.model_path,
                labels_path=self.labels_path,
                device=self.device,
                multi_label=self.multi_label,
                alpha=self.alpha,
                input_size=self.input_size,
            )
            self.get_logger().info(f"Action backend initialized: {backend_name}")
        except Exception as e:
            self.get_logger().error(f"Failed to init backend '{backend_name}': {e}. Fallback to 'stub'.")
            self.backend = get_backend('stub', clip_len=self.clip_len, sample_rate=self.sample_rate)

        # State
        self.last_image = None  # np.ndarray
        # Full-frame buffer for AVA-style backend (keep most recent clip_len frames)
        from collections import deque
        self.frame_buffer: Deque[np.ndarray] = deque(maxlen=self.clip_len)
        self.global_step: int = 0
        self.track_buffers: Dict[int, Deque[np.ndarray]] = defaultdict(lambda: deque(maxlen=self.clip_len * max(1, self.sample_rate)))
        self.track_steps: Dict[int, int] = defaultdict(int)
        self.track_last_action: Dict[int, Tuple[str, float]] = {}
        self.writer: Optional[cv2.VideoWriter] = None
        self._writer_lock = threading.Lock()
        # FPS estimation from incoming image timestamps (used if output_fps <= 0)
        self._prev_img_time_sec: Optional[float] = None
        self._estimated_fps: float = 0.0

        # Simple color mapping consistent with detector/tracker
        self.class_colors = {
            'person': (0, 255, 0),
            'bicycle': (255, 0, 0),
            'car': (0, 0, 255),
            'motorcycle': (255, 0, 255),
            'bus': (0, 255, 255),
            'truck': (255, 255, 0),
        }

        # Subscriptions
        img_qos = qos_profile_sensor_data
        tracks_qos = QoSProfile(depth=200)
        tracks_qos.history = QoSHistoryPolicy.KEEP_LAST
        tracks_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            img_qos,
            callback_group=self.cb_group,
        )
        self.tracks_sub = self.create_subscription(
            Detection2DArray,
            '/tracked_objects',
            self.tracks_callback,
            tracks_qos,
            callback_group=self.cb_group,
        )

        # Publisher
        actions_qos = QoSProfile(depth=200)
        actions_qos.history = QoSHistoryPolicy.KEEP_LAST
        actions_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.actions_pub = self.create_publisher(
            Detection2DArray,
            self.output_topic,
            actions_qos,
        )

        self.get_logger().info('ActionRecognizerNode started; waiting for tracks and images')

    def image_callback(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_image = img
            # Maintain full-frame buffer for AVA backend
            self.frame_buffer.append(img)
            # Estimate FPS from image timestamps
            try:
                stamp = msg.header.stamp
                t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
                if self._prev_img_time_sec is not None:
                    dt = max(1e-6, t - self._prev_img_time_sec)
                    inst_fps = 1.0 / dt
                    if self._estimated_fps <= 0:
                        self._estimated_fps = inst_fps
                    else:
                        self._estimated_fps = 0.9 * self._estimated_fps + 0.1 * inst_fps
                self._prev_img_time_sec = t
            except Exception:
                pass
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')

    def _parse_track(self, det: Detection2D) -> Tuple[Optional[int], Optional[str], Optional[float]]:
        track_id: Optional[int] = None
        actor_label: Optional[str] = None
        actor_score: Optional[float] = None
        for res in det.results:
            cid = res.hypothesis.class_id
            if cid is None:
                continue
            if cid.isdigit():
                try:
                    track_id = int(cid)
                except Exception:
                    pass
            else:
                # assume this is the semantic class label from detector (person, car, ...)
                if actor_label is None:
                    actor_label = cid
                    try:
                        actor_score = float(res.hypothesis.score)
                    except Exception:
                        actor_score = None
        return track_id, actor_label, actor_score

    def _crop_from_image(self, image: np.ndarray, det: Detection2D) -> Optional[np.ndarray]:
        h, w = image.shape[:2]
        cx = float(det.bbox.center.position.x)
        cy = float(det.bbox.center.position.y)
        bw = float(det.bbox.size_x)
        bh = float(det.bbox.size_y)
        if bw < self.min_box_size or bh < self.min_box_size:
            return None
        x1 = int(clamp(cx - bw / 2, 0, w - 1))
        y1 = int(clamp(cy - bh / 2, 0, h - 1))
        x2 = int(clamp(cx + bw / 2, 0, w - 1))
        y2 = int(clamp(cy + bh / 2, 0, h - 1))
        if x2 <= x1 or y2 <= y1:
            return None
        crop = image[y1:y2, x1:x2].copy()
        if crop.size == 0:
            return None
        # normalize crop size (helps backend stability)
        crop = cv2.resize(crop, (224, 224))
        return crop

    def tracks_callback(self, msg: Detection2DArray):
        if self.last_image is None:
            return

        image = self.last_image
        out = Detection2DArray()
        out.header = msg.header

        draw_img = image.copy()

        # If backend supports AVA-style full-frame inference, use it.
        use_ava = hasattr(self.backend, 'predict_full')
        boxes_px = []
        tracks = []  # (track_id, actor_label, actor_score, det)
        for det in msg.detections:
            track_id, actor_label, actor_score = self._parse_track(det)
            if track_id is None or actor_label is None:
                continue
            if self.allowed_actors and actor_label not in self.allowed_actors:
                continue
            tracks.append((track_id, actor_label, actor_score, det))
            # For overlay later; boxes needed in both modes
            cx = float(det.bbox.center.position.x)
            cy = float(det.bbox.center.position.y)
            bw = float(det.bbox.size_x)
            bh = float(det.bbox.size_y)
            x1 = clamp(cx - bw / 2, 0, image.shape[1] - 1)
            y1 = clamp(cy - bh / 2, 0, image.shape[0] - 1)
            x2 = clamp(cx + bw / 2, 0, image.shape[1] - 1)
            y2 = clamp(cy + bh / 2, 0, image.shape[0] - 1)
            boxes_px.append([x1, y1, x2, y2])

        did_infer = False
        if use_ava:
            # Full-frame clip buffer must be ready; infer on stride only when we have boxes
            if len(self.frame_buffer) >= self.clip_len and boxes_px:
                # advance stride only on frames with boxes so empty frames don't delay inference
                self.global_step += 1
                if self.global_step % max(1, self.inference_stride) == 0:
                    preds = self.backend.predict_full(list(self.frame_buffer), np.array(boxes_px, dtype=np.float32), image.shape[:2])
                else:
                    preds = []
                # Update cache and build outputs
                for (track_id, actor_label, actor_score, det), pred in zip(tracks, preds):
                    if pred is None:
                        continue
                    action_label, action_score = pred
                    self.track_last_action[track_id] = (action_label, float(action_score))

                    out_det = Detection2D()
                    out_det.bbox = det.bbox
                    # actor
                    hyp_actor = ObjectHypothesis()
                    hyp_actor.class_id = actor_label
                    hyp_actor.score = 1.0
                    oh_actor = ObjectHypothesisWithPose()
                    oh_actor.hypothesis = hyp_actor
                    oh_actor.pose.pose.position.x = det.bbox.center.position.x
                    oh_actor.pose.pose.position.y = det.bbox.center.position.y
                    oh_actor.pose.pose.orientation.w = 1.0
                    out_det.results.append(oh_actor)
                    # action
                    hyp_action = ObjectHypothesis()
                    hyp_action.class_id = f"action:{action_label}"
                    hyp_action.score = float(action_score)
                    oh_action = ObjectHypothesisWithPose()
                    oh_action.hypothesis = hyp_action
                    oh_action.pose.pose.position.x = det.bbox.center.position.x
                    oh_action.pose.pose.position.y = det.bbox.center.position.y
                    oh_action.pose.pose.orientation.w = 1.0
                    out_det.results.append(oh_action)
                    # track id
                    hyp_id = ObjectHypothesis()
                    hyp_id.class_id = str(int(track_id))
                    hyp_id.score = float(action_score)
                    oh_id = ObjectHypothesisWithPose()
                    oh_id.hypothesis = hyp_id
                    oh_id.pose.pose.position.x = det.bbox.center.position.x
                    oh_id.pose.pose.position.y = det.bbox.center.position.y
                    oh_id.pose.pose.orientation.w = 1.0
                    out_det.results.append(oh_id)
                    out.detections.append(out_det)
                did_infer = True
        else:
            # Fallback to crop-based classification per track (original path)
            for (track_id, actor_label, actor_score, det) in tracks:
                crop = self._crop_from_image(image, det)
                if crop is not None:
                    buf = self.track_buffers[track_id]
                    buf.append(crop)
                    self.track_steps[track_id] += 1

                    should_infer = len(buf) >= self.clip_len and (
                        self.track_steps[track_id] % max(1, self.inference_stride) == 0
                    )
                    if should_infer:
                        action_label, action_score = self.backend.predict(list(buf), actor_label)
                        self.track_last_action[track_id] = (action_label, float(action_score))

                        out_det = Detection2D()
                        out_det.bbox = det.bbox
                        hyp_actor = ObjectHypothesis()
                        hyp_actor.class_id = actor_label
                        hyp_actor.score = 1.0
                        oh_actor = ObjectHypothesisWithPose()
                        oh_actor.hypothesis = hyp_actor
                        oh_actor.pose.pose.position.x = det.bbox.center.position.x
                        oh_actor.pose.pose.position.y = det.bbox.center.position.y
                        oh_actor.pose.pose.orientation.w = 1.0
                        out_det.results.append(oh_actor)
                        hyp_action = ObjectHypothesis()
                        hyp_action.class_id = f"action:{action_label}"
                        hyp_action.score = float(action_score)
                        oh_action = ObjectHypothesisWithPose()
                        oh_action.hypothesis = hyp_action
                        oh_action.pose.pose.position.x = det.bbox.center.position.x
                        oh_action.pose.pose.position.y = det.bbox.center.position.y
                        oh_action.pose.pose.orientation.w = 1.0
                        out_det.results.append(oh_action)
                        hyp_id = ObjectHypothesis()
                        hyp_id.class_id = str(int(track_id))
                        hyp_id.score = float(action_score)
                        oh_id = ObjectHypothesisWithPose()
                        oh_id.hypothesis = hyp_id
                        oh_id.pose.pose.position.x = det.bbox.center.position.x
                        oh_id.pose.pose.position.y = det.bbox.center.position.y
                        oh_id.pose.pose.orientation.w = 1.0
                        out_det.results.append(oh_id)
                        out.detections.append(out_det)
                        did_infer = True

        # Draw overlay using last known action (or unknown) for all tracks
        for (track_id, actor_label, actor_score, det) in tracks:
            x_c = float(det.bbox.center.position.x)
            y_c = float(det.bbox.center.position.y)
            w_b = float(det.bbox.size_x)
            h_b = float(det.bbox.size_y)
            x1 = int(clamp(x_c - w_b / 2, 0, draw_img.shape[1] - 1))
            y1 = int(clamp(y_c - h_b / 2, 0, draw_img.shape[0] - 1))
            x2 = int(clamp(x_c + w_b / 2, 0, draw_img.shape[1] - 1))
            y2 = int(clamp(y_c + h_b / 2, 0, draw_img.shape[0] - 1))

            color = self.class_colors.get(actor_label, (255, 255, 255))
            cv2.rectangle(draw_img, (x1, y1), (x2, y2), color, 2)

            a_lbl, a_scr = self.track_last_action.get(track_id, ("unknown", 0.0))
            actor_scr = actor_score if actor_score is not None else 1.0
            text = f"ID:{track_id} {actor_label}:{actor_scr:.2f} act:{a_lbl}:{a_scr:.2f}"
            cv2.putText(
                draw_img,
                text,
                (x1, max(0, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )

        if out.detections:
            self.actions_pub.publish(out)

        # Initialize writer lazily and write frame
        if self.enable_output_video and self.output_video_path:
            with self._writer_lock:
                if self.writer is None:
                    h, w = draw_img.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    fps = float(self.output_fps) if float(self.output_fps) > 0 else (
                        float(self._estimated_fps) if self._estimated_fps > 0 else 30.0
                    )
                    self.writer = cv2.VideoWriter(self.output_video_path, fourcc, fps, (w, h))
                    if not self.writer.isOpened():
                        self.get_logger().error(f"Failed to open action output video: {self.output_video_path}")
                        self.writer = None
                if self.writer is not None:
                    self.writer.write(draw_img)


def main(args=None):
    rclpy.init(args=args)
    node = ActionRecognizerNode()
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
