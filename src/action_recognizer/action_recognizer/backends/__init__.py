from .stub import StubBackend
try:
    from .slowfast import SlowFastBackend  # optional dependency on torch
except Exception:
    SlowFastBackend = None
try:
    from .slowfast_ava import SlowFastAVABackend  # optional dependency on torch
except Exception:
    SlowFastAVABackend = None


def get_backend(name: str, **kwargs):
    name = (name or 'stub').lower()
    if name == 'stub':
        return StubBackend(**kwargs)
    if name == 'slowfast':
        if SlowFastBackend is None:
            raise ImportError('slowfast backend requested but torch/slowfast deps are unavailable')
        return SlowFastBackend(**kwargs)
    if name == 'slowfast_ava':
        if SlowFastAVABackend is None:
            raise ImportError('slowfast_ava backend requested but torch/slowfast deps are unavailable')
        return SlowFastAVABackend(**kwargs)
    raise ValueError(f"Unsupported backend: {name}")
