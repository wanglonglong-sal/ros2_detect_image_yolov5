#!/usr/bin/env python3
import argparse
from collections import deque
import re
import sys
from typing import Dict, Optional, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


class TopicRateMonitor(Node):
    def __init__(self, topics: Optional[List[str]], window: int = 50, period: float = 1.0, include: Optional[str] = None, exclude: Optional[str] = None, no_clear: bool = False):
        super().__init__('topic_rate_monitor')
        self.window = max(2, int(window))
        self.period = max(0.1, float(period))
        self.no_clear = bool(no_clear)
        self.dynamic = topics is None or len(topics) == 0
        self.include_re = re.compile(include) if include else None
        self.exclude_re = re.compile(exclude) if exclude else None

        # state per topic
        self._subs: Dict[str, object] = {}
        self._buffers: Dict[str, deque] = {}
        self._types: Dict[str, Optional[str]] = {}

        # Initialize topics list
        if self.dynamic:
            self._refresh_topics()
            self._discover_timer = self.create_timer(2.0, self._refresh_topics)
        else:
            for t in topics:
                self._ensure_subscription(t)

        self._print_timer = self.create_timer(self.period, self._print_status)

    def _filter_topic(self, name: str, type_list: List[str]) -> bool:
        if name == '' or name.startswith('/rosout'):
            return False
        if self.include_re and not self.include_re.search(name):
            return False
        if self.exclude_re and self.exclude_re.search(name):
            return False
        return True

    def _refresh_topics(self):
        names_and_types = self.get_topic_names_and_types()
        cur = set(self._subs.keys())
        wanted = []
        for name, types in names_and_types:
            if not self._filter_topic(name, types):
                continue
            wanted.append(name)
            if name not in self._subs:
                self._ensure_subscription(name)
        # Optionally drop subscriptions to removed topics
        for name in cur - set(wanted):
            sub = self._subs.pop(name, None)
            self._buffers.pop(name, None)
            self._types.pop(name, None)
            if sub is not None:
                try:
                    self.destroy_subscription(sub)
                except Exception:
                    pass

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
        topic_lower = topic.lower()
        if 'image' in topic_lower or type_str.endswith('/Image'):
            qos = qos_profile_sensor_data
        else:
            qos = QoSProfile(depth=100)
            qos.history = QoSHistoryPolicy.KEEP_LAST
            qos.reliability = QoSReliabilityPolicy.RELIABLE

        self._buffers.setdefault(topic, deque(maxlen=self.window))

        def cb(msg, tname=topic):
            now = self.get_clock().now().nanoseconds * 1e-9
            self._buffers[tname].append(now)

        self._subs[topic] = self.create_subscription(msg_type, topic, cb, qos)

    @staticmethod
    def _compute_hz(times: deque) -> Optional[float]:
        if len(times) < 2:
            return None
        dt = times[-1] - times[0]
        if dt <= 0:
            return None
        return (len(times) - 1) / dt

    def _print_status(self):
        if not self.no_clear:
            # Clear screen and move cursor home
            sys.stdout.write('\033[2J\033[H')
        sys.stdout.write('Topic Rate Monitor\n')
        sys.stdout.write('-------------------\n')

        # stable ordering: by name
        for t in sorted(self._buffers.keys()):
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
            sys.stdout.write(f"{t:40s} | {hz_str} | pubs: {pub_count:2d} | {typ}\n")
        sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description='Monitor ROS 2 topic rates with periodic refresh')
    parser.add_argument('--topics', nargs='*', default=None, help='Explicit topic names to monitor; omit to auto-monitor all topics')
    parser.add_argument('--window', type=int, default=50, help='Sliding window size for Hz estimation')
    parser.add_argument('--period', type=float, default=1.0, help='Refresh period in seconds')
    parser.add_argument('--include', type=str, default=None, help='Regex to include topics')
    parser.add_argument('--exclude', type=str, default=None, help='Regex to exclude topics')
    parser.add_argument('--no-clear', action='store_true', help='Do not clear screen between refreshes')
    args = parser.parse_args()

    rclpy.init()
    node = TopicRateMonitor(args.topics, window=args.window, period=args.period, include=args.include, exclude=args.exclude, no_clear=args.no_clear)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
