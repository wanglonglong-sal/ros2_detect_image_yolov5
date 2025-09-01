from collections import deque
from typing import List, Tuple
import numpy as np
import cv2


class StubBackend:
    """
    A lightweight, dependency-free backend that infers a coarse action
    based on inter-frame motion inside the crop.

    - For 'person': walking vs standing
    - For vehicles: moving vs stopped
    """

    def __init__(self, clip_len: int = 16, sample_rate: int = 2, **kwargs):
        self.clip_len = clip_len
        self.sample_rate = sample_rate

    def predict(self, clip: List[np.ndarray], actor_label: str) -> Tuple[str, float]:
        if len(clip) < 2:
            return ("unknown", 0.0)

        # Uniformly sample according to sample_rate
        frames = clip[:: self.sample_rate] if self.sample_rate > 1 else clip
        frames = frames[-self.clip_len :]

        # Compute simple motion magnitude across consecutive frames
        motion = []
        for i in range(1, len(frames)):
            a = cv2.cvtColor(frames[i - 1], cv2.COLOR_BGR2GRAY)
            b = cv2.cvtColor(frames[i], cv2.COLOR_BGR2GRAY)
            # resize to reduce noise/compute cost
            a = cv2.resize(a, (64, 64))
            b = cv2.resize(b, (64, 64))
            diff = cv2.absdiff(a, b)
            motion.append(float(diff.mean()))

        mean_motion = float(np.mean(motion)) if motion else 0.0

        if actor_label in ("person",):
            # Thresholds are heuristic
            if mean_motion > 3.0:
                return ("walking", min(1.0, mean_motion / 10.0))
            else:
                return ("standing", max(0.3, 1.0 - mean_motion / 3.0))
        elif actor_label in ("bicycle", "motorcycle", "car", "bus", "truck"):
            if mean_motion > 3.0:
                return ("moving", min(1.0, mean_motion / 10.0))
            else:
                return ("stopped", max(0.3, 1.0 - mean_motion / 3.0))

        return ("unknown", 0.1)
