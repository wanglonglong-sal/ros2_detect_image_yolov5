from typing import List, Tuple, Optional
import numpy as np


class SlowFastBackend:
    """
    SlowFast-style action backend.

    Requirements (provided by your environment):
      - torch
      - torchvision (optional if loading a custom TorchScript)

    Two modes:
      1) TorchScript checkpoint (recommended for AVA): set model_path to a .pt/.pth
         that accepts [slow_pathway, fast_pathway] tensors and returns logits.
      2) Fallback to torchvision SlowFast_R50 (Kinetics-400 classes) if available
         and model_path is empty. This is NOT AVA; labels must match Kinetics.
    """

    def __init__(
        self,
        clip_len: int = 32,
        sample_rate: int = 2,
        model_path: str = '',
        labels_path: str = '',
        device: str = 'cpu',
        alpha: int = 4,
        input_size: int = 224,
        multi_label: bool = True,
    ):
        try:
            import torch  # noqa: F401
        except Exception as e:
            raise ImportError('PyTorch is required for SlowFast backend') from e

        import torch
        self.torch = torch
        self.device = self.torch.device(device if self.torch.cuda.is_available() and device.startswith('cuda') else 'cpu')

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

        self.model = None
        self.model_path = model_path
        if model_path:
            self.model = self._load_torchscript(model_path)
        else:
            self.model = self._load_torchvision_slowfast()
        self.model.to(self.device)
        self.model.eval()

        # Normalization params (ImageNet/SlowFast convention)
        self.mean = np.array([0.45, 0.45, 0.45], dtype=np.float32)
        self.std = np.array([0.225, 0.225, 0.225], dtype=np.float32)

    def _load_torchscript(self, path: str):
        m = self.torch.jit.load(path, map_location='cpu')
        return m

    def _load_torchvision_slowfast(self):
        try:
            from torchvision.models.video import slowfast_r50
            from torchvision.models.video import SlowFast_R50_Weights
            weights = SlowFast_R50_Weights.DEFAULT
            model = slowfast_r50(weights=weights)
            return model
        except Exception as e:
            raise RuntimeError(
                'No model_path provided and torchvision SlowFast could not be loaded. '
                'Provide a TorchScript model_path for AVA.'
            ) from e

    def _preprocess_clip(self, clip: List[np.ndarray]):
        import cv2
        T = self.clip_len
        # ensure enough frames: pad last frame
        if len(clip) < T:
            clip = clip + [clip[-1]] * (T - len(clip))
        clip = clip[-T:]

        frames = []
        for img in clip:
            # BGR -> RGB
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            rgb = cv2.resize(rgb, (self.input_size, self.input_size), interpolation=cv2.INTER_LINEAR)
            x = rgb.astype(np.float32) / 255.0
            x = (x - self.mean) / self.std
            frames.append(x)
        # T, H, W, C -> C, T, H, W
        arr = np.stack(frames, axis=0)  # (T, H, W, C)
        arr = np.transpose(arr, (3, 0, 1, 2))  # (C, T, H, W)
        # Build pathways
        fast = arr  # (C, T, H, W)
        # slow samples every alpha frames
        slow = arr[:, :: self.alpha, :, :]

        # add batch dim
        fast = np.expand_dims(fast, 0)
        slow = np.expand_dims(slow, 0)

        fast_t = self.torch.from_numpy(fast).to(self.device)
        slow_t = self.torch.from_numpy(slow).to(self.device)
        return [slow_t, fast_t]

    def _forward(self, inputs):
        # Try common APIs: TorchScript taking list, torchvision taking dict or list
        with self.torch.no_grad():
            try:
                out = self.model(inputs)
            except Exception:
                try:
                    out = self.model(*inputs)
                except Exception:
                    # torchvision slowfast expects list [slow, fast]
                    out = self.model.forward(inputs)
        return out

    def _postprocess(self, logits) -> Tuple[str, float]:
        x = logits
        if isinstance(x, (list, tuple)):
            x = x[0]
        if hasattr(x, 'detach'):
            x = x.detach().cpu()
        x = x.squeeze()
        if self.multi_label:
            # AVA is multi-label; use sigmoid
            probs = self.torch.sigmoid(x)
        else:
            # single-label; use softmax
            probs = self.torch.softmax(x, dim=-1)
        probs_np = probs.numpy()
        idx = int(probs_np.argmax())
        score = float(probs_np[idx])
        if self.labels and 0 <= idx < len(self.labels):
            label = self.labels[idx]
        else:
            label = str(idx)
        return label, score

    def predict(self, clip: List[np.ndarray], actor_label: str) -> Tuple[str, float]:
        inputs = self._preprocess_clip(clip)
        logits = self._forward(inputs)
        return self._postprocess(logits)

