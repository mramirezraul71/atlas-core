"""VisualSignalAdapter v0 (paper): interpreta contexto visual ya normalizado.

Este adaptador NO conecta cámaras/APIs. Recibe datos simples (spot, strikes cortos,
niveles) y produce un ``breach_context`` compatible con
``AutoCloseEngine.evaluate_position(..., breach_context=...)``.
"""
from __future__ import annotations

from typing import Any


class VisualSignalAdapter:
    """Adapter lógico para rupturas de strikes/niveles.

    ``breach_tolerance`` se interpreta como porcentaje relativo del nivel
    (ej.: ``0.001`` = 0.1% de tolerancia).
    """

    def __init__(self, *, breach_tolerance: float = 0.0):
        self.breach_tolerance = max(0.0, float(breach_tolerance))

    def analyze_price_context(
        self,
        *,
        spot: float,
        short_put_strike: float | None = None,
        short_call_strike: float | None = None,
        support_levels: list[float] | None = None,
        resistance_levels: list[float] | None = None,
        is_0dte: bool = False,
    ) -> dict[str, Any]:
        """Devuelve ``breach_context`` listo para ``AutoCloseEngine``.

        Claves de primer nivel garantizadas:
        - ``critical_breach``, ``short_strike_breached``, ``breach_detected``,
          ``level_broken``, ``risk_break``, ``details``.
        """
        spot_f = self._safe_float(spot)
        sp_levels = self._normalize_levels(support_levels)
        rs_levels = self._normalize_levels(resistance_levels)
        details: dict[str, Any] = {
            "spot": spot_f,
            "short_put_strike": self._safe_float(short_put_strike),
            "short_call_strike": self._safe_float(short_call_strike),
            "breached_side": None,
            "breached_level_type": None,
            "breached_level": None,
            "breach_magnitude_pct": None,
            "is_0dte": bool(is_0dte),
            "notes": [],
        }
        out = {
            "critical_breach": False,
            "short_strike_breached": False,
            "breach_detected": False,
            "level_broken": None,
            "risk_break": False,
            "details": details,
        }
        if spot_f is None or spot_f <= 0:
            details["notes"].append("invalid_spot")
            return out

        # 1) Rupturas de short strikes (prioridad más alta).
        put_strike = self._safe_float(short_put_strike)
        call_strike = self._safe_float(short_call_strike)
        if put_strike is not None and put_strike > 0:
            tol = self._tol_abs(put_strike)
            if spot_f < (put_strike - tol):
                self._apply_breach(
                    out,
                    details,
                    level=put_strike,
                    level_type="short_strike",
                    side="put",
                    spot=spot_f,
                    note="short_put_strike_breached_0dte" if is_0dte else "short_put_strike_breached",
                    is_critical=bool(is_0dte),
                    short_strike=True,
                )
                return out
        if call_strike is not None and call_strike > 0:
            tol = self._tol_abs(call_strike)
            if spot_f > (call_strike + tol):
                self._apply_breach(
                    out,
                    details,
                    level=call_strike,
                    level_type="short_strike",
                    side="call",
                    spot=spot_f,
                    note="short_call_strike_breached_0dte" if is_0dte else "short_call_strike_breached",
                    is_critical=bool(is_0dte),
                    short_strike=True,
                )
                return out

        # 2) Rupturas de niveles estructurales (si no hay ruptura de strikes).
        # soporte relevante: el más cercano por debajo del spot; si spot está por debajo de todos,
        # usar el soporte más bajo (piso estructural externo).
        if sp_levels:
            support_ref = max((s for s in sp_levels if s <= spot_f), default=min(sp_levels))
            tol = self._tol_abs(support_ref)
            if spot_f < (support_ref - tol):
                self._apply_breach(
                    out,
                    details,
                    level=support_ref,
                    level_type="support",
                    side="support",
                    spot=spot_f,
                    note="support_level_broken",
                    is_critical=bool(is_0dte),
                    short_strike=False,
                )
                return out

        # resistencia relevante: la más cercana por encima del spot; si spot está por encima de todas,
        # usar la resistencia más alta (techo estructural externo).
        if rs_levels:
            resistance_ref = min((r for r in rs_levels if r >= spot_f), default=max(rs_levels))
            tol = self._tol_abs(resistance_ref)
            if spot_f > (resistance_ref + tol):
                self._apply_breach(
                    out,
                    details,
                    level=resistance_ref,
                    level_type="resistance",
                    side="resistance",
                    spot=spot_f,
                    note="resistance_level_broken",
                    is_critical=bool(is_0dte),
                    short_strike=False,
                )
                return out

        details["notes"].append("no_breach_detected")
        return out

    def _apply_breach(
        self,
        out: dict[str, Any],
        details: dict[str, Any],
        *,
        level: float,
        level_type: str,
        side: str,
        spot: float,
        note: str,
        is_critical: bool,
        short_strike: bool,
    ) -> None:
        out["breach_detected"] = True
        out["level_broken"] = level
        out["critical_breach"] = bool(is_critical)
        out["risk_break"] = bool(is_critical)
        out["short_strike_breached"] = bool(short_strike)

        details["breached_side"] = side
        details["breached_level_type"] = level_type
        details["breached_level"] = level
        details["breach_magnitude_pct"] = round(abs(spot - level) / level * 100.0, 6) if level > 0 else None
        details["notes"].append(note)

    def _tol_abs(self, level: float) -> float:
        return abs(level) * self.breach_tolerance

    @staticmethod
    def _safe_float(value: Any) -> float | None:
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _normalize_levels(values: list[float] | None) -> list[float]:
        if not values:
            return []
        out: list[float] = []
        for v in values:
            try:
                fv = float(v)
            except (TypeError, ValueError):
                continue
            if fv > 0:
                out.append(fv)
        return sorted(set(out))
