"""Gates de seguridad stub para F1."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class GateDecision:
    allow: bool
    reason: str = "f1_stub"


def evaluate_live_gate(live_enabled: bool) -> GateDecision:
    """Bloquea ejecución live por defecto."""
    if not live_enabled:
        return GateDecision(allow=False, reason="ATLAS_LIVE_TRADING_ENABLED=false")
    return GateDecision(allow=True, reason="live_explicitly_enabled")
