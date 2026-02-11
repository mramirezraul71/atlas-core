"""Web navigator: Playwright-based browser automation. Policy + audit."""
from __future__ import annotations

from .navigator import (
    is_available,
    get_missing_deps,
    open_url,
    click,
    fill,
    extract_text,
    screenshot,
    close,
    status,
)

__all__ = [
    "is_available", "get_missing_deps",
    "open_url", "click", "fill", "extract_text", "screenshot", "close", "status",
]
