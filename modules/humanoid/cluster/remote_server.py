"""Worker-side endpoints: /remote/hands, /remote/web, /remote/vision, /remote/voice. Verify HMAC."""
from __future__ import annotations

from typing import Any, Dict, Optional

# Verification helper: call from FastAPI dependency or before handler.
def verify_cluster_request(
    node_id: str,
    ts: str,
    nonce: str,
    method: str,
    path: str,
    body: Optional[bytes],
    signature: str,
) -> bool:
    from . import auth as cluster_auth
    return cluster_auth.verify_request(node_id, ts, nonce, method, path, body, signature)


def execute_remote_hands(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run hands (shell) locally. Called by HQ via POST /remote/hands."""
    try:
        from modules.humanoid.hands.safe_shell import SafeShell
        from modules.humanoid.policy import ActorContext, get_policy_engine
        cmd = payload.get("command") or payload.get("cmd") or ""
        if not cmd:
            return {"ok": False, "error": "missing command", "data": None}
        actor = ActorContext(actor="cluster", role="worker")
        decision = get_policy_engine().can(actor, "hands", "exec_command", target=cmd)
        if not decision.allow:
            return {"ok": False, "error": decision.reason or "policy denied", "data": None}
        shell = SafeShell()
        result = shell.run(cmd, timeout_sec=payload.get("timeout_sec", 30))
        return {"ok": result.get("ok", False), "data": result, "error": result.get("error")}
    except Exception as e:
        return {"ok": False, "error": str(e), "data": None}


def execute_remote_web(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run web action locally."""
    try:
        url = payload.get("action") or payload.get("url") or ""
        if not url:
            return {"ok": False, "error": "missing action/url", "data": None}
        from modules.humanoid.web.navigator import open_url
        timeout_ms = (payload.get("timeout_sec") or 30) * 1000
        result = open_url(url, timeout_ms=timeout_ms)
        return {"ok": result.get("ok", False), "data": result, "error": result.get("error")}
    except Exception as e:
        return {"ok": False, "error": str(e), "data": None}


def execute_remote_vision(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run vision (analyze image) locally."""
    try:
        image_path = payload.get("image_path") or payload.get("path") or ""
        if not image_path:
            return {"ok": False, "error": "missing image_path", "data": None}
        from modules.humanoid.vision.analyzer import analyze
        result = analyze(image_path)
        return {"ok": True, "data": result, "error": None}
    except Exception as e:
        return {"ok": False, "error": str(e), "data": None}


def execute_remote_voice(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run voice action locally."""
    try:
        action = payload.get("action") or "status"
        if action == "status":
            enabled = __import__("os").getenv("VOICE_ENABLED", "true").strip().lower() in ("1", "true", "yes")
            return {"ok": True, "data": {"enabled": enabled}, "error": None}
        return {"ok": False, "error": "unsupported action", "data": None}
    except Exception as e:
        return {"ok": False, "error": str(e), "data": None}
