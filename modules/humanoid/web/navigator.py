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
_pw = None
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


def _ensure_session(headless: bool = True) -> Dict[str, Any]:
    """Asegura que exista una sesión Playwright activa (browser+page)."""
    if not is_available():
        return {"ok": False, "error": "playwright not installed", "missing_deps": _missing_deps}
    global _pw, _browser, _page
    if _page is not None:
        return {"ok": True}
    try:
        # sync_playwright().start() -> Playwright (con .stop()).
        _pw = _playwright().start()  # type: ignore[union-attr]
        _browser = _pw.chromium.launch(headless=headless)
        _page = _browser.new_page()
        return {"ok": True}
    except Exception as e:
        try:
            close()
        except Exception:
            pass
        return {"ok": False, "error": str(e)}


def open_url(url: str, timeout_ms: int = 30000, show_browser: Optional[bool] = None) -> Dict[str, Any]:
    """Open URL. Returns {ok, error}. Policy + rate limit applied."""
    if not is_available():
        return {"ok": False, "error": "playwright not installed", "missing_deps": _missing_deps}
    allowed, reason = can_navigate(url)
    if not allowed:
        _audit("open_url", False, {"url": url}, reason)
        return {"ok": False, "error": reason}
    t0 = time.perf_counter()
    try:
        if show_browser is None:
            show = (os.getenv("DIGITAL_FEET_SHOW_BROWSER", "false") or "").strip().lower() in ("1", "true", "yes")
        else:
            show = bool(show_browser)
        s = _ensure_session(headless=(not show))
        if not s.get("ok"):
            return {"ok": False, "error": s.get("error") or "no session", "missing_deps": s.get("missing_deps")}
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


def configure_page(viewport: Optional[Dict[str, int]] = None, user_agent: Optional[str] = None) -> Dict[str, Any]:
    """Configure current page: viewport size and/or user agent. Spec: configure page."""
    if not is_available() or _page is None:
        return {"ok": False, "error": "no session"}
    try:
        if viewport:
            _page.set_viewport_size(viewport)
        if user_agent:
            _page.set_extra_http_headers({"User-Agent": user_agent})
        _audit("configure_page", True, {"viewport": viewport, "user_agent": bool(user_agent)})
        return {"ok": True}
    except Exception as e:
        _audit("configure_page", False, {}, str(e))
        return {"ok": False, "error": str(e)}


def close() -> None:
    global _pw, _browser, _page
    if _browser:
        try:
            _browser.close()
        except Exception:
            pass
    _browser = None
    _page = None
    if _pw is not None:
        try:
            _pw.stop()
        except Exception:
            pass
    _pw = None


def run_steps(steps: List[Dict[str, Any]], timeout_ms: int = 30000, show_browser: Optional[bool] = None) -> Dict[str, Any]:
    """
    Ejecuta un workflow de navegación. Cada step es un dict con:
      - kind: goto|click|fill|wait|press|screenshot|extract_text|close
    Retorna {ok, steps, last_url?, screenshot_path?, text?}
    """
    if not is_available():
        return {"ok": False, "error": "playwright not installed", "missing_deps": _missing_deps}
    results: List[Dict[str, Any]] = []
    last_text = ""
    last_shot = ""
    last_url = ""
    t0 = time.perf_counter()
    try:
        if show_browser is None:
            show = (os.getenv("DIGITAL_FEET_SHOW_BROWSER", "false") or "").strip().lower() in ("1", "true", "yes")
        else:
            show = bool(show_browser)
        s = _ensure_session(headless=(not show))
        if not s.get("ok"):
            return {"ok": False, "error": s.get("error") or "no session", "missing_deps": s.get("missing_deps")}
        for i, st in enumerate(steps or []):
            kind = (st.get("kind") or "").strip().lower()
            try:
                if kind in ("goto", "open", "open_url", "url"):
                    url = str(st.get("url") or "")
                    allowed, reason = can_navigate(url)
                    if not allowed:
                        raise RuntimeError(reason or "navigation denied")
                    _page.goto(url, timeout=int(st.get("timeout_ms") or timeout_ms))
                    try:
                        last_url = _page.url if _page is not None else last_url
                    except Exception:
                        pass
                    results.append({"i": i, "kind": "goto", "ok": True, "url": url})
                    _memory_web("navigate", url)
                elif kind == "click":
                    sel = str(st.get("selector") or "")
                    _page.click(sel, timeout=int(st.get("timeout_ms") or 5000))
                    try:
                        last_url = _page.url if _page is not None else last_url
                    except Exception:
                        pass
                    results.append({"i": i, "kind": "click", "ok": True, "selector": sel})
                elif kind == "fill":
                    sel = str(st.get("selector") or "")
                    val = str(st.get("value") or "")
                    _page.fill(sel, val, timeout=int(st.get("timeout_ms") or 5000))
                    try:
                        last_url = _page.url if _page is not None else last_url
                    except Exception:
                        pass
                    results.append({"i": i, "kind": "fill", "ok": True, "selector": sel})
                elif kind == "wait":
                    sel = str(st.get("selector") or "")
                    _page.wait_for_selector(sel, timeout=int(st.get("timeout_ms") or 5000))
                    try:
                        last_url = _page.url if _page is not None else last_url
                    except Exception:
                        pass
                    results.append({"i": i, "kind": "wait", "ok": True, "selector": sel})
                elif kind == "press":
                    keys = str(st.get("keys") or "")
                    _page.keyboard.press(keys)
                    try:
                        last_url = _page.url if _page is not None else last_url
                    except Exception:
                        pass
                    results.append({"i": i, "kind": "press", "ok": True, "keys": keys})
                elif kind == "screenshot":
                    out_dir = Path(os.getenv("DIGITAL_FEET_DIR", r"C:\ATLAS_PUSH\snapshots\digital_feet"))
                    out_dir.mkdir(parents=True, exist_ok=True)
                    name = st.get("name") or f"shot_{int(time.time())}.png"
                    path = str(out_dir / str(name))
                    _page.screenshot(path=path)
                    last_shot = path
                    try:
                        last_url = _page.url if _page is not None else last_url
                    except Exception:
                        pass
                    results.append({"i": i, "kind": "screenshot", "ok": True, "path": path})
                    _memory_web("screenshot", path)
                elif kind == "extract_text":
                    html = _page.content()
                    from .extractors import extract_text_from_html
                    last_text = extract_text_from_html(html)[:5000]
                    try:
                        last_url = _page.url if _page is not None else last_url
                    except Exception:
                        pass
                    results.append({"i": i, "kind": "extract_text", "ok": True, "chars": len(last_text)})
                    _memory_web("extract", "page_content", last_text[:1000])
                elif kind == "close":
                    close()
                    results.append({"i": i, "kind": "close", "ok": True})
                else:
                    raise RuntimeError("unsupported_step_kind")
                # OPS narration (permanent comms)
                try:
                    from modules.humanoid.comms.ops_bus import emit as ops_emit

                    extra = ""
                    if kind == "goto":
                        extra = f" url={url}"
                    elif kind in ("click", "wait"):
                        extra = f" selector={sel}"
                    elif kind == "fill":
                        extra = f" selector={sel} len={len(val)}"
                    elif kind == "press":
                        extra = f" keys={keys}"
                    elif kind == "screenshot":
                        extra = f" path={path}"
                    ops_emit(
                        "digital_feet",
                        f"Paso web {i+1}/{len(steps or [])}: {kind}.{extra}",
                        level="info",
                        data={"step_kind": kind, "i": i, "last_url": last_url},
                        evidence_path=(last_shot if kind == "screenshot" else ""),
                    )
                except Exception:
                    pass
            except Exception as e:
                results.append({"i": i, "kind": kind or "?", "ok": False, "error": str(e)})
                _audit("run_steps", False, {"step": st, "i": i}, str(e))
                return {
                    "ok": False,
                    "error": str(e),
                    "steps": results,
                    "screenshot_path": last_shot,
                    "text": last_text,
                    "ms": int((time.perf_counter() - t0) * 1000),
                }
        _audit("run_steps", True, {"steps": len(results)})
        return {
            "ok": True,
            "steps": results,
            "last_url": last_url,
            "screenshot_path": last_shot,
            "text": last_text,
            "ms": int((time.perf_counter() - t0) * 1000),
        }
    except Exception as e:
        _audit("run_steps", False, {"steps": len(steps or [])}, str(e))
        return {"ok": False, "error": str(e), "steps": results, "ms": int((time.perf_counter() - t0) * 1000)}


def status() -> Dict[str, Any]:
    return {
        "enabled": is_available(),
        "missing_deps": get_missing_deps(),
        "session_active": _page is not None,
    }
