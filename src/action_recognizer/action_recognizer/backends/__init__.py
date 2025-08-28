from .stub import StubBackend


def get_backend(name: str, **kwargs):
    name = (name or 'stub').lower()
    if name == 'stub':
        return StubBackend(**kwargs)
    raise ValueError(f"Unsupported backend: {name}")

