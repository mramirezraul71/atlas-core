"""Analizador de cartera multi-estrategia para ATLAS-Quant OptionStrat.

Agrega PnL, griegas y métricas de riesgo de múltiples estrategias activas.

Responsabilidades
-----------------
- Mantener registro de estrategias abiertas (en memoria + JSON persistence).
- Calcular griegas totales de la cartera (delta/gamma/theta/vega/rho).
- Calcular PnL actual por estrategia y total.
- Exponer distribución de riesgo por asset_class, bias, símbolo.
- Controlar concentración: delta por símbolo, vega total, etc.
"""
from __future__ import annotations

import json
import threading
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from atlas_code_quant.options.strategy_engine import (
    GreeksVector,
    MarketSnapshot,
    Strategy,
    StrategyRiskSummary,
    greeks,
    pnl_today,
    strategy_risk_summary,
    strategy_to_dict,
    strategy_from_dict,
)


# ---------------------------------------------------------------------------
# PortfolioEntry — wrapper de estrategia con estado vivo
# ---------------------------------------------------------------------------

@dataclass
class PortfolioEntry:
    strategy: Strategy
    opened_at: datetime = field(default_factory=datetime.utcnow)
    entry_cost: float = 0.0          # débito pagado o crédito recibido
    quantity: int = 1                # unidades de la estructura
    status: str = "open"             # "open" | "closed" | "partial"
    notes: str = ""
    tags: List[str] = field(default_factory=list)

    # Snapshot de riesgo al abrir (calculado una vez)
    initial_risk_summary: Optional[StrategyRiskSummary] = None

    @property
    def name(self) -> str:
        return self.strategy.name

    @property
    def underlying(self) -> str:
        return self.strategy.underlying


# ---------------------------------------------------------------------------
# PortfolioGreeks — griegas agregadas de toda la cartera
# ---------------------------------------------------------------------------

@dataclass
class PortfolioGreeks:
    total: GreeksVector = field(default_factory=GreeksVector)
    by_underlying: Dict[str, GreeksVector] = field(default_factory=dict)
    by_strategy: Dict[str, GreeksVector] = field(default_factory=dict)

    def to_dict(self) -> Dict:
        return {
            "total": self.total.to_dict(),
            "by_underlying": {k: v.to_dict() for k, v in self.by_underlying.items()},
            "by_strategy": {k: v.to_dict() for k, v in self.by_strategy.items()},
        }


# ---------------------------------------------------------------------------
# PortfolioSummary — snapshot completo
# ---------------------------------------------------------------------------

@dataclass
class PortfolioSummary:
    n_strategies: int = 0
    total_pnl: float = 0.0
    greeks: PortfolioGreeks = field(default_factory=PortfolioGreeks)
    by_underlying: Dict[str, float] = field(default_factory=dict)   # pnl por subyacente
    by_bias: Dict[str, int] = field(default_factory=dict)           # conteo por bias
    by_asset_class: Dict[str, float] = field(default_factory=dict)  # pnl por asset class
    risk_flags: List[str] = field(default_factory=list)             # alertas de concentración
    entries: List[Dict] = field(default_factory=list)               # resumen por estrategia
    computed_at: str = field(default_factory=lambda: datetime.utcnow().isoformat())

    def to_dict(self) -> Dict:
        return {
            "n_strategies": self.n_strategies,
            "total_pnl": round(self.total_pnl, 2),
            "greeks": self.greeks.to_dict(),
            "by_underlying": {k: round(v, 2) for k, v in self.by_underlying.items()},
            "by_bias": self.by_bias,
            "by_asset_class": {k: round(v, 2) for k, v in self.by_asset_class.items()},
            "risk_flags": self.risk_flags,
            "entries": self.entries,
            "computed_at": self.computed_at,
        }


# ---------------------------------------------------------------------------
# PortfolioAnalyzer — clase principal
# ---------------------------------------------------------------------------

class PortfolioAnalyzer:
    """Gestiona y analiza una cartera de estrategias multi-activo.

    Thread-safe para uso desde LiveLoop y API en paralelo.

    Args:
        storage_path: ruta JSON para persistencia (opcional)
        delta_limit:  delta total máximo tolerado por subyacente
        vega_limit:   vega total máximo tolerado de la cartera
    """

    def __init__(
        self,
        storage_path: Optional[Path] = None,
        delta_limit_per_underlying: float = 500.0,
        vega_limit_total: float = 10_000.0,
    ):
        self._lock = threading.RLock()
        self._entries: List[PortfolioEntry] = []
        self._storage_path = storage_path
        self._delta_limit = delta_limit_per_underlying
        self._vega_limit = vega_limit_total

        if storage_path and storage_path.exists():
            self._load()

    # ------------------------------------------------------------------
    # Gestión de estrategias
    # ------------------------------------------------------------------

    def add_strategy(
        self,
        strategy: Strategy,
        market: MarketSnapshot,
        quantity: int = 1,
        notes: str = "",
        tags: List[str] | None = None,
    ) -> PortfolioEntry:
        """Añade una estrategia a la cartera y calcula su resumen inicial."""
        with self._lock:
            S_grid = np.linspace(
                max(0.01, market.underlying_price * 0.6),
                market.underlying_price * 1.4,
                500,
            )
            risk = strategy_risk_summary(strategy, market, S_grid)
            entry = PortfolioEntry(
                strategy=strategy,
                quantity=quantity,
                notes=notes,
                tags=tags or [],
                initial_risk_summary=risk,
            )
            self._entries.append(entry)
            self._save()
        return entry

    def remove_strategy(self, strategy_name: str) -> bool:
        """Cierra (marca como closed) una estrategia por nombre."""
        with self._lock:
            for e in self._entries:
                if e.name == strategy_name and e.status == "open":
                    e.status = "closed"
                    self._save()
                    return True
        return False

    def get_open_entries(self) -> List[PortfolioEntry]:
        with self._lock:
            return [e for e in self._entries if e.status == "open"]

    # ------------------------------------------------------------------
    # Análisis de cartera
    # ------------------------------------------------------------------

    def compute_summary(
        self,
        market_prices: Dict[str, float],
        risk_free_rate: float = 0.05,
    ) -> PortfolioSummary:
        """Calcula el resumen completo de la cartera.

        Args:
            market_prices: dict {symbol: current_price}
            risk_free_rate: tasa libre de riesgo para pricing BS
        """
        entries = self.get_open_entries()
        summary = PortfolioSummary(n_strategies=len(entries))
        total_greeks = GreeksVector()
        by_underlying_greeks: Dict[str, GreeksVector] = {}
        by_strategy_greeks: Dict[str, GreeksVector] = {}

        for entry in entries:
            sym = entry.underlying or _guess_symbol(entry.strategy)
            spot = market_prices.get(sym, 0.0)
            if spot <= 0:
                # Usar precio de entrada si no hay precio de mercado
                spot = _best_effort_spot(entry.strategy)

            market = MarketSnapshot(underlying_price=spot, risk_free_rate=risk_free_rate)
            S_arr = np.array([spot])

            # PnL actual
            entry_pnl_arr = pnl_today(entry.strategy, market, S_arr)
            entry_pnl = float(entry_pnl_arr[0]) * entry.quantity
            summary.total_pnl += entry_pnl
            summary.by_underlying[sym] = summary.by_underlying.get(sym, 0.0) + entry_pnl

            # Griegas
            g = greeks(entry.strategy, market)
            total_greeks = total_greeks + g

            by_underlying_greeks[sym] = (
                by_underlying_greeks.get(sym, GreeksVector()) + g
            )
            by_strategy_greeks[entry.name] = g

            # Asset class distribution
            ac = _dominant_asset_class(entry.strategy)
            summary.by_asset_class[ac] = summary.by_asset_class.get(ac, 0.0) + entry_pnl

            # Bias count
            bias = entry.strategy.metadata.get("bias", "unknown")
            summary.by_bias[bias] = summary.by_bias.get(bias, 0) + 1

            # Entry snapshot para la lista
            risk = entry.initial_risk_summary
            summary.entries.append({
                "name": entry.name,
                "underlying": sym,
                "current_pnl": round(entry_pnl, 2),
                "status": entry.status,
                "bias": bias,
                "greeks": g.to_dict(),
                "max_profit": round(risk.max_profit, 2) if risk and not isinstance(risk.max_profit, float) else (risk.max_profit if risk else None),
                "max_loss":   round(risk.max_loss, 2)   if risk and not isinstance(risk.max_loss, float)   else (risk.max_loss if risk else None),
                "breakevens": risk.breakevens if risk else [],
                "opened_at": entry.opened_at.isoformat(),
                "tags": entry.tags,
            })

        # Ensamblar griegas totales
        summary.greeks = PortfolioGreeks(
            total=total_greeks,
            by_underlying=by_underlying_greeks,
            by_strategy=by_strategy_greeks,
        )

        # Risk flags
        summary.risk_flags = self._check_risk_flags(
            total_greeks, by_underlying_greeks
        )

        return summary

    def compute_aggregate_greeks(
        self,
        market_prices: Dict[str, float],
        risk_free_rate: float = 0.05,
    ) -> GreeksVector:
        """Griegas totales de la cartera (rápido, sin PnL completo)."""
        entries = self.get_open_entries()
        total = GreeksVector()
        for entry in entries:
            sym = entry.underlying or _guess_symbol(entry.strategy)
            spot = market_prices.get(sym, _best_effort_spot(entry.strategy))
            if spot <= 0:
                continue
            market = MarketSnapshot(underlying_price=spot, risk_free_rate=risk_free_rate)
            total = total + greeks(entry.strategy, market)
        return total

    # ------------------------------------------------------------------
    # Risk flags
    # ------------------------------------------------------------------

    def _check_risk_flags(
        self,
        total: GreeksVector,
        by_underlying: Dict[str, GreeksVector],
    ) -> List[str]:
        flags = []

        # Delta total por subyacente
        for sym, g in by_underlying.items():
            if abs(g.delta) > self._delta_limit:
                flags.append(
                    f"DELTA_CONCENTRACIÓN: {sym} delta={g.delta:.0f} "
                    f"(límite ±{self._delta_limit:.0f})"
                )

        # Vega total
        if abs(total.vega) > self._vega_limit:
            flags.append(
                f"VEGA_EXCESO: vega total={total.vega:.0f} "
                f"(límite {self._vega_limit:.0f})"
            )

        # Theta negativo extremo (pagando mucho tiempo)
        if total.theta < -500:
            flags.append(
                f"THETA_NEGATIVO_EXTREMO: {total.theta:.0f}$/día"
            )

        return flags

    # ------------------------------------------------------------------
    # Persistencia
    # ------------------------------------------------------------------

    def _save(self) -> None:
        if not self._storage_path:
            return
        try:
            self._storage_path.parent.mkdir(parents=True, exist_ok=True)
            data = []
            for e in self._entries:
                data.append({
                    "strategy": strategy_to_dict(e.strategy),
                    "opened_at": e.opened_at.isoformat(),
                    "entry_cost": e.entry_cost,
                    "quantity": e.quantity,
                    "status": e.status,
                    "notes": e.notes,
                    "tags": e.tags,
                })
            tmp = self._storage_path.with_suffix(".tmp")
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            tmp.replace(self._storage_path)
        except Exception as exc:
            import logging
            logging.getLogger(__name__).error("PortfolioAnalyzer._save error: %s", exc)

    def _load(self) -> None:
        try:
            with open(self._storage_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self._entries = []
            for d in data:
                strat = strategy_from_dict(d["strategy"])
                entry = PortfolioEntry(
                    strategy=strat,
                    opened_at=datetime.fromisoformat(d.get("opened_at", datetime.utcnow().isoformat())),
                    entry_cost=d.get("entry_cost", 0.0),
                    quantity=d.get("quantity", 1),
                    status=d.get("status", "open"),
                    notes=d.get("notes", ""),
                    tags=d.get("tags", []),
                )
                self._entries.append(entry)
        except Exception as exc:
            import logging
            logging.getLogger(__name__).error("PortfolioAnalyzer._load error: %s", exc)

    # ------------------------------------------------------------------
    # Estado
    # ------------------------------------------------------------------

    def status(self) -> Dict:
        with self._lock:
            open_entries = [e for e in self._entries if e.status == "open"]
        return {
            "n_open": len(open_entries),
            "n_total": len(self._entries),
            "strategies": [e.name for e in open_entries],
        }


# ---------------------------------------------------------------------------
# Helpers internos
# ---------------------------------------------------------------------------

def _guess_symbol(strategy: Strategy) -> str:
    for leg in strategy.legs:
        from atlas_code_quant.options.strategy_engine import OptionLeg, LinearLeg
        if isinstance(leg, OptionLeg):
            return leg.underlying_symbol
        if isinstance(leg, LinearLeg):
            return leg.symbol
    return "UNKNOWN"


def _best_effort_spot(strategy: Strategy) -> float:
    from atlas_code_quant.options.strategy_engine import OptionLeg, LinearLeg
    for leg in strategy.legs:
        if isinstance(leg, OptionLeg) and leg.underlying_price > 0:
            return leg.underlying_price
        if isinstance(leg, LinearLeg) and leg.entry_price > 0:
            return leg.entry_price
    return 100.0  # fallback genérico


def _dominant_asset_class(strategy: Strategy) -> str:
    from atlas_code_quant.options.strategy_engine import OptionLeg, LinearLeg
    has_options = any(isinstance(l, OptionLeg) for l in strategy.legs)
    linear_classes = [l.asset_class for l in strategy.legs if isinstance(l, LinearLeg)]

    if has_options and not linear_classes:
        return "options"
    if has_options and linear_classes:
        return "covered_" + (linear_classes[0] if linear_classes else "stock")
    if linear_classes:
        return linear_classes[0]
    return "unknown"
