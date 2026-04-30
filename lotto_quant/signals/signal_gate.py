"""
lotto_quant.signals.signal_gate
===============================

Filters EV signals before alert dispatch and Kelly sizing.

Rules
-----
1. EV / dollar must exceed `MIN_EV_NET_POSITIVE`.
2. STRONG signals additionally require `MIN_EV_NET_STRONG` AND anomaly = True.
3. Hysteresis: do not fire the same signal type for the same game more than
   once per `min_repeat_seconds`.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from .. import config
from ..models.ev_calculator import EVResult

logger = logging.getLogger(__name__)


@dataclass
class GateDecision:
    pass_through: bool
    reason: str
    signal_type: str       # 'STALE_PRIZE' | 'JACKPOT_EV_POSITIVE' | 'MARKOV_ANOMALY' | 'NONE'
    signal_strength: str   # mirrors EVResult.signal_strength when passed
    is_repeat: bool


class SignalGate:
    """Stateful gate with per-game hysteresis."""

    def __init__(self, min_repeat_seconds: int = 6 * 3600):
        self.min_repeat_seconds = min_repeat_seconds
        self._last_emitted: Dict[Tuple[str, str], float] = {}

    # ── decision logic ─────────────────────────────────────────────
    def evaluate(self, ev: EVResult, signal_type: str = "STALE_PRIZE") -> GateDecision:
        if ev.adjusted_ev_nc <= 0 or ev.signal_strength == "NEGATIVE":
            return GateDecision(
                pass_through=False,
                reason=f"EV non-positive ({ev.ev_per_dollar:+.4f})",
                signal_type="NONE",
                signal_strength=ev.signal_strength,
                is_repeat=False,
            )

        if ev.signal_strength == "STRONG":
            ok = (
                ev.ev_per_dollar >= config.MIN_EV_NET_STRONG
                and ev.is_anomaly
            )
            if not ok:
                return GateDecision(
                    pass_through=False,
                    reason="STRONG flag set but criteria not met",
                    signal_type=signal_type,
                    signal_strength=ev.signal_strength,
                    is_repeat=False,
                )

        is_repeat = self._is_repeat(ev.game_id, signal_type)
        if is_repeat:
            return GateDecision(
                pass_through=False,
                reason="Repeat within hysteresis window",
                signal_type=signal_type,
                signal_strength=ev.signal_strength,
                is_repeat=True,
            )

        self._last_emitted[(ev.game_id, signal_type)] = time.time()
        return GateDecision(
            pass_through=True,
            reason="EV gate passed",
            signal_type=signal_type,
            signal_strength=ev.signal_strength,
            is_repeat=False,
        )

    def _is_repeat(self, game_id: str, signal_type: str) -> bool:
        last = self._last_emitted.get((game_id, signal_type))
        if last is None:
            return False
        return (time.time() - last) < self.min_repeat_seconds

    def reset(self) -> None:
        self._last_emitted.clear()
