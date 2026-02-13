"""ATLAS_MODE lite/pro/ultra + SYSTEM_MODE observe/safe/pro/aggressive/ultra + auto-adapt from deps/hardware."""
from __future__ import annotations

import os
from typing import Any, Dict

def _env(name: str, default: str) -> str:
    v = os.getenv(name)
    return (v or "").strip().lower() or default


def get_atlas_mode() -> str:
    """lite | pro | ultra."""
    m = _env("ATLAS_MODE", "pro")
    if m in ("lite", "pro", "ultra"):
        return m
    return "pro"


def get_system_mode() -> str:
    """observe | safe | pro | aggressive | ultra."""
    m = _env("SYSTEM_MODE", "safe")
    if m in ("observe", "safe", "pro", "aggressive", "ultra"):
        return m
    return "safe"


def is_observe_only() -> bool:
    """True si solo inspección, no modifica nada."""
    return get_system_mode() == "observe"


def is_safe_or_higher() -> bool:
    """True si puede modificar bajo riesgo bajo."""
    m = get_system_mode()
    return m in ("safe", "pro", "aggressive", "ultra")


def is_aggressive_or_ultra() -> bool:
    """True si autoexpansión o autoevolución."""
    return get_system_mode() in ("aggressive", "ultra")


def is_evolution_enabled() -> bool:
    return _env("EVOLUTION_ENABLED", "true") in ("1", "true", "yes")


def is_selfprog_enabled() -> bool:
    return _env("SELFPROG_ENABLED", "true") in ("1", "true", "yes")


def is_auto_update_enabled() -> bool:
    return _env("AUTO_UPDATE_ENABLED", "true") in ("1", "true", "yes")


def is_auto_refactor_enabled() -> bool:
    return _env("AUTO_REFRACTOR_ENABLED", "false") in ("1", "true", "yes")


def is_auto_dep_install_enabled() -> bool:
    return _env("AUTO_DEP_INSTALL", "true") in ("1", "true", "yes")


def is_auto_plugin_expansion_enabled() -> bool:
    return _env("AUTO_PLUGIN_EXPANSION", "false") in ("1", "true", "yes")


def is_rollback_enabled() -> bool:
    return _env("ROLLBACK_ENABLED", "true") in ("1", "true", "yes")


def rollback_on_smoke_fail() -> bool:
    return _env("ROLLBACK_ON_SMOKE_FAIL", "true") in ("1", "true", "yes")


def _screen_act_deps_ok() -> bool:
    """mss/pyautogui for capture+act; pywinauto optional."""
    try:
        import mss
    except ImportError:
        return False
    try:
        import pyautogui
    except ImportError:
        return False
    return True


def _playwright_ok() -> bool:
    try:
        from modules.humanoid.deps_checker import check_playwright
        return check_playwright().get("available", False)
    except Exception:
        return False


def _tesseract_ok() -> bool:
    try:
        import pytesseract
        pytesseract.get_tesseract_version()
        return True
    except Exception:
        return False


def _low_resource_heuristic() -> bool:
    """Heuristic: low RAM or no GPU => prefer fast model."""
    try:
        import psutil
        mem = psutil.virtual_memory()
        if mem.total < 8 * 1024 * 1024 * 1024:  # < 8GB
            return True
    except ImportError:
        pass
    return False


def is_screen_act_allowed() -> bool:
    """Pro/Ultra + deps (mss+pyautogui). Lite => False."""
    mode = get_atlas_mode()
    if mode == "lite":
        return False
    if mode in ("pro", "ultra") and _screen_act_deps_ok():
        return True
    return False


def is_record_replay_allowed() -> bool:
    """Pro/Ultra only. Ultra gets full record/replay."""
    return get_atlas_mode() in ("pro", "ultra")


def is_benchmark_allowed() -> bool:
    """Ultra only; or pro if METALEARN_ENABLED."""
    mode = get_atlas_mode()
    if mode == "ultra":
        return True
    if mode == "pro" and _env("METALEARN_ENABLED", "true") in ("1", "true", "yes"):
        return True
    return False


def use_fast_model_by_default() -> bool:
    """If low resource heuristic, prefer AI_FAST_MODEL."""
    return _low_resource_heuristic()


def get_mode_capabilities() -> Dict[str, Any]:
    """Capabilities by current mode + deps."""
    mode = get_atlas_mode()
    screen_act = is_screen_act_allowed()
    playwright = _playwright_ok()
    tesseract = _tesseract_ok()
    record_replay = is_record_replay_allowed() and mode == "ultra"
    system_mode = get_system_mode()
    return {
        "mode": mode,
        "system_mode": system_mode,
        "screen_capture": True,
        "screen_analyze": True,
        "screen_act": screen_act,
        "screen_record_replay": record_replay,
        "playwright": playwright,
        "ocr_tesseract": tesseract,
        "ocr_fallback_vision": not tesseract,
        "benchmark": is_benchmark_allowed(),
        "use_fast_model_default": use_fast_model_by_default(),
    }
