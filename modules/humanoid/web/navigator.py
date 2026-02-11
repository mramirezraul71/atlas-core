"""Browser automation: Playwright wrapper. Disabled if Playwright not installed."""
from __future__ import annotations

import logging
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from .policy import can_navigate

_log = logging.getLogger("humanoid.web")
_playwright = None
_browser = None
_page = None
_missing_deps: List[str] = []


def _try_import_playwright() -> bool:
    global _playwright, _missing_deps
    if _playwright is not None:
        return len(_missing_deps) == 0
    try:
        from playwright.sync_api import sync_playwright
        _playwright = sync_playwright
        _missing_deps = []
        return True
    except ImportError:
        _missing_deps = ["playwright"]
        return False


def is_available() -> bool:
    return _try_import_playwright()


def get_missing_deps() -> List[str]:
    _try_import_playwright()
    return list(_missing_deps)


def _audit(action: str, ok: bool, payload: Optional[Dict] = None, error: Optional[str] = None) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("web", "system", "navigator", action, ok, 0, error, payload, None)
    except Exception:
        pass


def _memory_web(kind: str, path_or_uri: str, content_preview: str = "") -> None:
    """Integrar acción web en memoria persistente."""
    try:
        from modules.humanoid.memory_engine import store_artifact
        store_artifact(None, None, f"web_{kind}", path_or_uri[:2048], content_preview[:1024])
    except Exception:
        pass


def open_url(url: str, timeout_ms: int = 30000) -> Dict[str, Any]:
    """Open URL. Returns {ok, error}. Policy + rate limit applied."""
    if not is_available():
        return {"ok": False, "error": "playwright not installed", "missing_deps": _missing_deps}
    allowed, reason = can_navigate(url)
    if not allowed:
        _audit("open_url", False, {"url": url}, reason)
        return {"ok": False, "error": reason}
    global _browser, _page
    t0 = time.perf_counter()
    try:
        pw = _playwright()
        _browser = pw.chromium.launch(headless=True)
        _page = _browser.new_page()
        _page.goto(url, timeout=timeout_ms)
        _audit("open_url", True, {"url": url})
        _memory_web("navigate", url)
        return {"ok": True, "ms": int((time.perf_counter() - t0) * 1000)}
    except Exception as e:
        _audit("open_url", False, {"url": url}, str(e))
        return {"ok": False, "error": str(e), "ms": int((time.perf_counter() - t0) * 1000)}


def click(selector: str, timeout_ms: int = 5000) -> Dict[str, Any]:
    if not is_available() or _page is None:
        return {"ok": False, "error": "no session or playwright missing"}
    try:
        _page.click(selector, timeout=timeout_ms)
        _audit("click", True, {"selector": selector})
        return {"ok": True}
    except Exception as e:
        _audit("click", False, {"selector": selector}, str(e))
        return {"ok": False, "error": str(e)}


def fill(selector: str, value: str, timeout_ms: int = 5000) -> Dict[str, Any]:
    if not is_available() or _page is None:
        return {"ok": False, "error": "no session or playwright missing"}
    try:
        _page.fill(selector, value, timeout=timeout_ms)
        _audit("fill", True, {"selector": selector})
        return {"ok": True}
    except Exception as e:
        _audit("fill", False, {"selector": selector}, str(e))
        return {"ok": False, "error": str(e)}


def extract_text() -> Dict[str, Any]:
    if not is_available() or _page is None:
        return {"ok": False, "text": "", "error": "no session"}
    try:
        from .extractors import extract_text_from_html
        html = _page.content()
        text = extract_text_from_html(html)
        _memory_web("extract", "page_content", text[:1000])
        return {"ok": True, "text": text[:5000]}
    except Exception as e:
        return {"ok": False, "text": "", "error": str(e)}


def screenshot(path: Optional[str] = None) -> Dict[str, Any]:
    if not is_available() or _page is None:
        return {"ok": False, "path": "", "error": "no session"}
    try:
        if path is None:
            path = str(Path(os.getenv("TEMP", ".")) / "atlas_web_screenshot.png")
        _page.screenshot(path=path)
        _audit("screenshot", True, {"path": path})
        _memory_web("screenshot", path)
        return {"ok": True, "path": path}
    except Exception as e:
        return {"ok": False, "path": "", "error": str(e)}


def close() -> None:
    global _browser, _page
    if _browser:
        try:
            _browser.close()
        except Exception:
            pass
    _browser = None
    _page = None


def status() -> Dict[str, Any]:
    return {
        "enabled": is_available(),
        "missing_deps": get_missing_deps(),
        "session_active": _page is not None,
    }
