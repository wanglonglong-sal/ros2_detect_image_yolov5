#!/usr/bin/env python3
import argparse
from collections import deque
import sys
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


class TopicRateMonitor(Node):
    def __init__(self, topics, window: int = 50, period: float = 1.0):
        super().__init__('topic_rate_monitor')
        self.window = max(2, int(window))
        self.period = max(0.1, float(period))
        self.topics = topics

        # state per topic
        self._subs = {}
        self._buffers: Dict[str, deque] = {t: deque(maxlen=self.window) for t in topics}
        self._types: Dict[str, Optional[str]] = {t: None for t in topics}

        # Try initial subscription and schedule periodic retry for missing types
        for t in topics:
            self._ensure_subscription(t)

        self._retry_timer = self.create_timer(2.0, self._retry_missing)
        self._print_timer = self.create_timer(self.period, self._print_status)

    def _ensure_subscription(self, topic: str):
        if topic in self._subs and self._subs[topic] is not None:
            return
        # resolve type
        tlist = dict(self.get_topic_names_and_types())
        if topic not in tlist or not tlist[topic]:
            return
        type_str = tlist[topic][0]
        self._types[topic] = type_str
        try:
            msg_type = get_message(type_str)
        except Exception as e:
            self.get_logger().warn(f"Failed to load message type for {topic}: {type_str} ({e})")
            return

        # QoS: image-like topics use sensor QoS; others reliable with modest depth
        if 'image' in topic.lower():
            qos = qos_profile_sensor_data
        else:
            qos = QoSProfile(depth=100)
            qos.history = QoSHistoryPolicy.KEEP_LAST
            qos.reliability = QoSReliabilityPolicy.RELIABLE

        def cb(msg, tname=topic):
            now = self.get_clock().now().nanoseconds * 1e-9
            self._buffers[tname].append(now)

        self._subs[topic] = self.create_subscription(msg_type, topic, cb, qos)

    def _retry_missing(self):
        for t in self.topics:
            if self._subs.get(t) is None:
                self._ensure_subscription(t)

    def _compute_hz(self, times: deque) -> Optional[float]:
        if len(times) < 2:
            return None
        dt = times[-1] - times[0]
        if dt <= 0:
            return None
        return (len(times) - 1) / dt

    def _print_status(self):
        # Clear screen and move cursor home
        sys.stdout.write('\033[2J\033[H')
        sys.stdout.write('Topic Rate Monitor\n')
        sys.stdout.write('-------------------\n')
        for t in self.topics:
            buf = self._buffers[t]
            hz = self._compute_hz(buf)
            hz_str = f"{hz:6.2f} Hz" if hz is not None else "   n/a"
            typ = self._types.get(t) or "<waiting>"
            pub_count = 0
            sub = self._subs.get(t)
            if sub is not None:
                try:
                    pub_count = sub.get_publisher_count()
                except Exception:
                    pass
            sys.stdout.write(f"{t:24s} | {hz_str} | pubs: {pub_count:2d} | {typ}\n")
        sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description='Monitor ROS 2 topic rates with periodic refresh')
    parser.add_argument('--topics', nargs='+', default=['/image_raw', '/detections', '/tracked_objects', '/actions'], help='List of topic names to monitor')
    parser.add_argument('--window', type=int, default=50, help='Sliding window size for Hz estimation')
    parser.add_argument('--period', type=float, default=1.0, help='Refresh period in seconds')
    args = parser.parse_args()

    rclpy.init()
    node = TopicRateMonitor(args.topics, window=args.window, period=args.period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

