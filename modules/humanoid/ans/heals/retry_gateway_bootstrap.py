"""Retry gateway bootstrap."""
from __future__ import annotations

from .base import heal_result


def run(**kwargs) -> dict:
    try:
        from modules.humanoid.gateway.bootstrap import bootstrap
        r = bootstrap()
        ok = r.get("ok", False)
        return heal_result(ok, "retry_gateway_bootstrap", r.get("message", "ok"), r)
    except Exception as e:
        return heal_result(False, "retry_gateway_bootstrap", str(e), {}, str(e))
