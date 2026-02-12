"""Unified dispatcher: run_hands, run_web, run_vision, run_voice with local/remote routing."""
from __future__ import annotations

import time
from typing import Any, Dict, Optional

from modules.humanoid.cluster import registry, router, trace, remote_client, remote_server
from modules.humanoid.dispatch.policies import is_action_allowed_for_node

REMOTE_TIMEOUT = 30


def _std_result(ok: bool, data: Any = None, ms: int = 0, error: Optional[str] = None, correlation_id: Optional[str] = None) -> Dict[str, Any]:
    out = {"ok": ok, "data": data, "ms": ms, "error": error}
    if correlation_id:
        out["correlation_id"] = correlation_id
    return out


def _run_local_hands(command: str, timeout_sec: int = 30, **kwargs: Any) -> Dict[str, Any]:
    return remote_server.execute_remote_hands({"command": command, "timeout_sec": timeout_sec, **kwargs})


def _run_local_web(action: str, timeout_sec: int = 30, **kwargs: Any) -> Dict[str, Any]:
    return remote_server.execute_remote_web({"action": action, "url": action, "timeout_sec": timeout_sec, **kwargs})


def _run_local_vision(image_path: str, **kwargs: Any) -> Dict[str, Any]:
    return remote_server.execute_remote_vision({"image_path": image_path, **kwargs})


def _run_local_voice(action: str = "status", **kwargs: Any) -> Dict[str, Any]:
    return remote_server.execute_remote_voice({"action": action, **kwargs})


def run_hands(command: str, timeout_sec: int = 30, prefer_remote: bool = False, correlation_id: Optional[str] = None, **kwargs: Any) -> Dict[str, Any]:
    """Execute hands (shell) locally or on best remote node."""
    t0 = time.perf_counter()
    cid = correlation_id or trace.get_correlation_id() or trace.new_correlation_id()
    if not registry.cluster_enabled():
        r = _run_local_hands(command, timeout_sec, **kwargs)
        return _std_result(r.get("ok", False), r.get("data"), int((time.perf_counter() - t0) * 1000), r.get("error"), cid)
    decision = router.route_decision("hands", prefer_remote=prefer_remote, require_capability="hands")
    if decision.get("route") == "remote" and decision.get("base_url") and is_action_allowed_for_node(decision.get("node_id", ""), "hands"):
        resp = remote_client.post_json(decision["base_url"], "/remote/hands", {"command": command, "timeout_sec": timeout_sec, **kwargs}, timeout_sec=REMOTE_TIMEOUT)
        body = resp.get("data") or {}
        ms = int((time.perf_counter() - t0) * 1000)
        if resp.get("ok") and body.get("ok"):
            return _std_result(True, body.get("data"), body.get("ms", ms), body.get("error"), body.get("correlation_id") or cid)
        return _std_result(False, body.get("data"), ms, body.get("error") or resp.get("error", "remote failed"), cid)
    r = _run_local_hands(command, timeout_sec, **kwargs)
    return _std_result(r.get("ok", False), r.get("data"), int((time.perf_counter() - t0) * 1000), r.get("error"), cid)


def run_web(action: str, timeout_sec: int = 30, prefer_remote: bool = False, correlation_id: Optional[str] = None, **kwargs: Any) -> Dict[str, Any]:
    """Execute web action locally or on best remote node."""
    t0 = time.perf_counter()
    cid = correlation_id or trace.get_correlation_id() or trace.new_correlation_id()
    if not registry.cluster_enabled():
        r = _run_local_web(action, timeout_sec, **kwargs)
        return _std_result(r.get("ok", False), r.get("data"), int((time.perf_counter() - t0) * 1000), r.get("error"), cid)
    decision = router.route_decision("web", prefer_remote=prefer_remote, require_capability="web")
    if decision.get("route") == "remote" and decision.get("base_url") and is_action_allowed_for_node(decision.get("node_id", ""), "web"):
        resp = remote_client.post_json(decision["base_url"], "/remote/web", {"action": action, "timeout_sec": timeout_sec, **kwargs}, timeout_sec=REMOTE_TIMEOUT)
        body = resp.get("data") or {}
        ms = int((time.perf_counter() - t0) * 1000)
        if resp.get("ok") and body.get("ok"):
            return _std_result(True, body.get("data"), body.get("ms", ms), body.get("error"), body.get("correlation_id") or cid)
        return _std_result(False, body.get("data"), ms, body.get("error") or resp.get("error", "remote failed"), cid)
    r = _run_local_web(action, timeout_sec, **kwargs)
    return _std_result(r.get("ok", False), r.get("data"), int((time.perf_counter() - t0) * 1000), r.get("error"), cid)


def run_vision(image_path: str, prefer_remote: bool = False, correlation_id: Optional[str] = None, **kwargs: Any) -> Dict[str, Any]:
    """Execute vision (analyze image) locally or on best remote node."""
    t0 = time.perf_counter()
    cid = correlation_id or trace.get_correlation_id() or trace.new_correlation_id()
    if not registry.cluster_enabled():
        r = _run_local_vision(image_path, **kwargs)
        return _std_result(r.get("ok", False), r.get("data"), int((time.perf_counter() - t0) * 1000), r.get("error"), cid)
    decision = router.route_decision("vision", prefer_remote=prefer_remote, require_capability="vision")
    if decision.get("route") == "remote" and decision.get("base_url") and is_action_allowed_for_node(decision.get("node_id", ""), "vision"):
        resp = remote_client.post_json(decision["base_url"], "/remote/vision", {"image_path": image_path, **kwargs}, timeout_sec=REMOTE_TIMEOUT)
        body = resp.get("data") or {}
        ms = int((time.perf_counter() - t0) * 1000)
        if resp.get("ok") and body.get("ok"):
            return _std_result(True, body.get("data"), body.get("ms", ms), body.get("error"), body.get("correlation_id") or cid)
        return _std_result(False, body.get("data"), ms, body.get("error") or resp.get("error", "remote failed"), cid)
    r = _run_local_vision(image_path, **kwargs)
    return _std_result(r.get("ok", False), r.get("data"), int((time.perf_counter() - t0) * 1000), r.get("error"), cid)


def run_voice(action: str = "status", prefer_remote: bool = False, correlation_id: Optional[str] = None, **kwargs: Any) -> Dict[str, Any]:
    """Execute voice action locally or on best remote node."""
    t0 = time.perf_counter()
    cid = correlation_id or trace.get_correlation_id() or trace.new_correlation_id()
    if not registry.cluster_enabled():
        r = _run_local_voice(action, **kwargs)
        return _std_result(r.get("ok", False), r.get("data"), int((time.perf_counter() - t0) * 1000), r.get("error"), cid)
    decision = router.route_decision("voice", prefer_remote=prefer_remote, require_capability="voice")
    if decision.get("route") == "remote" and decision.get("base_url") and is_action_allowed_for_node(decision.get("node_id", ""), "voice"):
        resp = remote_client.post_json(decision["base_url"], "/remote/voice", {"action": action, **kwargs}, timeout_sec=REMOTE_TIMEOUT)
        body = resp.get("data") or {}
        ms = int((time.perf_counter() - t0) * 1000)
        if resp.get("ok") and body.get("ok"):
            return _std_result(True, body.get("data"), body.get("ms", ms), body.get("error"), body.get("correlation_id") or cid)
        return _std_result(False, body.get("data"), ms, body.get("error") or resp.get("error", "remote failed"), cid)
    r = _run_local_voice(action, **kwargs)
    return _std_result(r.get("ok", False), r.get("data"), int((time.perf_counter() - t0) * 1000), r.get("error"), cid)
