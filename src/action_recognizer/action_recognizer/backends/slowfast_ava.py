from typing import List, Tuple, Optional
import numpy as np


class SlowFastAVABackend:
    """
    AVA-style action backend using a TorchScript SlowFast detection head.

    - Expects full-frame clip inputs and a set of person boxes on the keyframe.
    - Model signature (TorchScript): forward(inputs, boxes)
      where inputs = [slow, fast], boxes = (N, 5) with first column being frame index (0),
      and following 4 columns being (x1, y1, x2, y2) in resized pixel coords.
    - Outputs per-box logits (N, num_classes). Postprocess with sigmoid (multi-label).

    Parameters:
      clip_len: number of frames per clip (e.g., 64)
      sample_rate: temporal sampling rate (e.g., 2)
      model_path: path to TorchScript .pt/.pth
      labels_path: path to txt with one label per line
      device: 'cpu' or 'cuda:0'
      alpha: SlowFast alpha (e.g., 4)
      input_size: spatial size to which frames are resized (e.g., 256)
      multi_label: use sigmoid on logits
    """

    def __init__(
        self,
        clip_len: int = 64,
        sample_rate: int = 2,
        model_path: str = '',
        labels_path: str = '',
        device: str = 'cpu',
        alpha: int = 4,
        input_size: int = 256,
        multi_label: bool = True,
    ):
        try:
            import torch  # noqa: F401
        except Exception as e:
            raise ImportError('PyTorch is required for SlowFast AVA backend') from e

        # Import torchvision to register custom ops (e.g., torchvision::roi_align) used by TorchScript.
        try:
            import torchvision  # noqa: F401
        except Exception as e:
            raise ImportError(
                'TorchVision is required at runtime to register ops (e.g., roi_align) used in the TorchScript model. '
                'Please install a torchvision version compatible with your torch.'
            ) from e

        import torch
        self.torch = torch
        use_cuda = self.torch.cuda.is_available() and device and device.startswith('cuda')
        self.device = self.torch.device(device if use_cuda else 'cpu')

        self.clip_len = int(clip_len)
        self.sample_rate = int(sample_rate)
        self.alpha = int(alpha)
        self.input_size = int(input_size)
        self.multi_label = bool(multi_label)

        self.labels = None
        if labels_path:
            try:
                with open(labels_path, 'r', encoding='utf-8') as f:
                    self.labels = [line.strip() for line in f if line.strip()]
            except Exception:
                self.labels = None

        if not model_path:
            raise ValueError('SlowFast AVA backend requires a TorchScript model_path')
        self.model_path = model_path
        self.model = self._load_torchscript(model_path)
        self.model.to(self.device)
        self.model.eval()

        # Normalization params (SlowFast convention)
        self.mean = np.array([0.45, 0.45, 0.45], dtype=np.float32)
        self.std = np.array([0.225, 0.225, 0.225], dtype=np.float32)

    def _load_torchscript(self, path: str):
        m = self.torch.jit.load(path, map_location='cpu')
        return m

    def _preprocess_frames(self, frames: List[np.ndarray]):
        """
        frames: list of BGR images, arbitrary size. Returns [slow_t, fast_t] tensors on device.
        """
        import cv2
        T = self.clip_len
        if len(frames) < T:
            frames = frames + [frames[-1]] * (T - len(frames))
        frames = frames[-T:]

        arrs = []
        for img in frames:
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            rgb = cv2.resize(rgb, (self.input_size, self.input_size), interpolation=cv2.INTER_LINEAR)
            x = rgb.astype(np.float32) / 255.0
            x = (x - self.mean) / self.std
            arrs.append(x)
        arr = np.stack(arrs, axis=0)             # (T, H, W, C)
        arr = np.transpose(arr, (3, 0, 1, 2))    # (C, T, H, W)

        fast = np.expand_dims(arr, 0)            # (1, C, T, H, W)
        slow = np.expand_dims(arr[:, :: self.alpha, :, :], 0)
        fast_t = self.torch.from_numpy(fast).to(self.device)
        slow_t = self.torch.from_numpy(slow).to(self.device)
        return [slow_t, fast_t]

    def _scale_boxes(self, boxes_xyxy: np.ndarray, orig_hw: Tuple[int, int]):
        """
        Scale pixel boxes from original image size to resized input_size.
        boxes_xyxy: (N, 4) in original pixel coords
        orig_hw: (H, W)
        Returns: (N, 4) in resized pixel coords
        """
        H, W = orig_hw
        if H <= 0 or W <= 0:
            return np.zeros_like(boxes_xyxy, dtype=np.float32)
        scale_x = self.input_size / float(W)
        scale_y = self.input_size / float(H)
        scaled = boxes_xyxy.astype(np.float32).copy()
        scaled[:, [0, 2]] *= scale_x
        scaled[:, [1, 3]] *= scale_y
        return scaled

    def _forward(self, inputs, boxes_t):
        """
        Run model forward handling both possible TorchScript signatures:
        - forward(inputs, boxes) where inputs = [slow, fast]
        - forward(slow, fast, boxes)
        """
        with self.torch.no_grad():
            try:
                # Some TorchScript exports accept a list [slow, fast] as first arg
                return self.model(inputs, boxes_t)
            except Exception:
                # Others expect separate tensors (slow, fast, boxes)
                if isinstance(inputs, (list, tuple)) and len(inputs) == 2:
                    return self.model(inputs[0], inputs[1], boxes_t)
                raise

    def _postprocess(self, logits_box: 'torch.Tensor') -> List[Tuple[str, float]]:
        x = logits_box
        if hasattr(x, 'detach'):
            x = x.detach().cpu()
        if self.multi_label:
            probs = self.torch.sigmoid(x)
        else:
            probs = self.torch.softmax(x, dim=-1)
        probs_np = probs.numpy()
        results = []
        for p in probs_np:
            idx = int(p.argmax())
            score = float(p[idx])
            if self.labels and 0 <= idx < len(self.labels):
                label = self.labels[idx]
            else:
                label = str(idx)
            results.append((label, score))
        return results

    def predict_full(
        self,
        frames: List[np.ndarray],
        boxes_xyxy: np.ndarray,
        image_shape: Tuple[int, int],
    ) -> List[Tuple[str, float]]:
        """
        Predict actions for provided boxes on the current (key) frame using the full-frame clip.
        - frames: list of recent frames (BGR), length >= clip_len
        - boxes_xyxy: (N, 4) in original pixel coords
        - image_shape: (H, W) of original frames
        Returns: list of (label, score) aligned with boxes order
        """
        if boxes_xyxy is None or len(boxes_xyxy) == 0:
            return []

        inputs = self._preprocess_frames(frames)
        scaled = self._scale_boxes(boxes_xyxy, image_shape)
        # build (N, 5) with zero frame index
        if isinstance(scaled, np.ndarray):
            idx_col = np.zeros((scaled.shape[0], 1), dtype=np.float32)
            boxes5 = np.concatenate([idx_col, scaled.astype(np.float32)], axis=1)
        else:
            raise RuntimeError('boxes scaling failed')

        boxes_t = self.torch.from_numpy(boxes5).to(self.device)
        logits = self._forward(inputs, boxes_t)
        return self._postprocess(logits)
