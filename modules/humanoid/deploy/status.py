"""Build GET /deploy/status payload: mode, ports, PIDs, last_deploy, last_health, canary."""
from __future__ import annotations

from typing import Any, Dict

from .switcher import get_deploy_state


def build_deploy_status() -> Dict[str, Any]:
    """
    Returns payload for GET /deploy/status:
    ok, mode, active_port, staging_port, active_pid?, staging_pid?, last_deploy, last_health, canary.
    """
    state = get_deploy_state()
    active_port = state.get("active_port") or 8791
    staging_port = state.get("staging_port") or 8792
    last_deploy = state.get("last_deploy")
    if not last_deploy and state.get("last_deploy_ts"):
        last_deploy = {"ts": state["last_deploy_ts"], "ref": state.get("last_deploy_ref"), "result": state.get("last_deploy_result"), "error": state.get("last_deploy_error")}
    last_health = state.get("last_health")

    canary_payload: Dict[str, Any] = {"enabled": False, "percentage": 0.0, "features": [], "stats": {}}
    try:
        from .canary import get_canary_stats
        st = get_canary_stats()
        canary_payload = {
            "enabled": st.get("enabled", False),
            "percentage": st.get("percentage", 0.0),
            "features": st.get("features", []),
            "stats": {
                "canary_calls_1h": st.get("canary_calls_1h", 0),
                "stable_calls_1h": st.get("stable_calls_1h", 0),
                "canary_error_rate": st.get("canary_error_rate"),
                "canary_avg_latency_ms": st.get("canary_avg_latency_ms"),
                "stable_avg_latency_ms": st.get("stable_avg_latency_ms"),
            },
        }
    except Exception:
        pass

    return {
        "ok": True,
        "mode": state.get("mode", "single"),
        "active_port": int(active_port),
        "staging_port": int(staging_port),
        "active_pid": state.get("active_pid"),
        "staging_pid": state.get("staging_pid"),
        "last_deploy": last_deploy or {},
        "last_health": last_health or {},
        "canary": canary_payload,
    }
