"""
Cognitive System API Module.

Provides REST and WebSocket endpoints for real-time monitoring
of the Atlas Cognitive Architecture.
"""
from .api import router, include_cognitive_router

__all__ = [
    "router",
    "include_cognitive_router",
]
