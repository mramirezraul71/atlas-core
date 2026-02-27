"""Concurrent Goal Engine health check."""
from __future__ import annotations


def run() -> dict:
    try:
        from modules.humanoid.cortex.frontal.concurrent_engine import get_engine
        engine = get_engine()
        status = engine.get_status()
        active = status.get("active_goals", 0)
        total = status.get("total_tracked", 0)
        ticks = status.get("tick_count", 0)
        max_c = status.get("max_concurrent", 4)

        issues = []
        if active > max_c:
            issues.append(f"active_goals ({active}) exceeds max_concurrent ({max_c})")

        resources = status.get("resources", {}).get("resources", {})
        stuck = [r for r, info in resources.items()
                 if info.get("available", 1) == 0 and info.get("exclusive", False)]
        if stuck:
            issues.append(f"exclusive resources fully locked: {stuck}")

        ok = len(issues) == 0
        severity = "low" if ok else ("med" if active <= max_c else "high")

        return {
            "ok": ok,
            "check_id": "cge_health",
            "message": "ok" if ok else "; ".join(issues),
            "details": {
                "active_goals": active,
                "total_tracked": total,
                "tick_count": ticks,
                "locked_resources": stuck,
            },
            "severity": severity,
        }
    except Exception as e:
        return {
            "ok": False,
            "check_id": "cge_health",
            "message": str(e),
            "details": {"error": str(e)},
            "severity": "med",
        }
