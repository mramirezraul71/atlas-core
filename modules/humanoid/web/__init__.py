"""Web navigator: Playwright-based browser automation. Policy + audit."""
from __future__ import annotations

from .navigator import (click, close, extract_text, fill, get_missing_deps,
                        is_available, open_url, screenshot, status)

__all__ = [
    "is_available",
    "get_missing_deps",
    "open_url",
    "click",
    "fill",
    "extract_text",
    "screenshot",
    "close",
    "status",
]
