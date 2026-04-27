"""Dashboard del Radar Kalshi (FastAPI router + UI estática)."""
from .router import build_router, RadarState

__all__ = ["build_router", "RadarState"]
