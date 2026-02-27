"""Release train: versioning, channel (stable/canary), post-promote CI."""
from __future__ import annotations

from .version import get_version_info

__all__ = ["get_version_info"]
