"""Tune router: lower priority for slow/failing model (bounded)."""
from __future__ import annotations

from .base import heal_result


def run(**kwargs) -> dict:
    # Metalearn handles tuning; ANS can trigger metalearn run
    try:
        from modules.humanoid.metalearn.cycle import run_cycle
        run_cycle()
        return heal_result(True, "tune_router", "metalearn cycle triggered", {})
    except Exception as e:
        return heal_result(False, "tune_router", str(e), {}, str(e))
