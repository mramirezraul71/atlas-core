"""Módulo 5 — Risk Engine: Kelly Fraccionado + Volatilidad Inversa.

Fórmula Kelly fraccionado (Quarter-Kelly):
    f* = (p*b - q) / b × 0.25
    p = win_rate, q = 1-p, b = avg_win / avg_loss

Tamaño de lote final:
    Lote = min(Kelly_shares, Capital×1% / (ATR₂₀ × Multiplier))

Circuit breaker:
    - DD >8% → fracción 0.1 (modo defensivo)
    - 3 pérdidas consecutivas → pausa 30 min

Memoria de riesgo:
    - Redis para estado en tiempo real
    - Historial persistente en archivo JSON
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional

logger = logging.getLogger("atlas.risk.kelly")

try:
    import numpy as np
    _NP_OK = True
except ImportError:
    _NP_OK = False


# ── Estructuras ───────────────────────────────────────────────────────────────

@dataclass
class PositionSize:
    shares: int
    kelly_fraction: float
    inv_vol_fraction: float
    final_fraction: float
    risk_usd: float
    capital_pct: float
    method: str = "kelly_inv_vol"


@dataclass
class RiskState:
    drawdown_pct: float = 0.0
    peak_equity: float = 0.0
    current_equity: float = 0.0
    consecutive_losses: int = 0
    win_rate_rolling: float = 0.5
    avg_win_usd: float = 0.0
    avg_loss_usd: float = 0.0
    circuit_breaker_active: bool = False
    circuit_breaker_until: float = 0.0
    kelly_fraction_active: float = 0.25
    total_trades: int = 0
    winning_trades: int = 0
    last_updated: float = 0.0


# ── Risk Engine ───────────────────────────────────────────────────────────────

class KellyRiskEngine:
    """Motor de gestión de riesgo con Kelly fraccionado y volatilidad inversa.

    Uso::

        engine = KellyRiskEngine(initial_capital=100_000)
        size = engine.compute_size(
            symbol="AAPL",
            price=185.50,
            atr=3.20,
            signal_confidence=0.72,
        )
        print(f"Comprar {size.shares} acciones, riesgo ${size.risk_usd:.2f}")
    """

    # ── Parámetros de riesgo ──────────────────────────────────────────────────
    BASE_KELLY_FRACTION  = 0.25     # Quarter-Kelly
    DEFENSIVE_FRACTION   = 0.10     # cuando DD >8%
    MAX_RISK_PER_TRADE   = 0.01     # 1% del capital máximo por trade
    MAX_POSITION_PCT     = 0.20     # 20% del capital en una posición
    DRAWDOWN_THRESHOLD   = 0.08     # 8% → circuit breaker
    CONSEC_LOSSES_CB     = 3        # 3 pérdidas consecutivas → pausa
    CB_PAUSE_MIN         = 30       # pausa en minutos tras circuit breaker
    ATR_SL_MULTIPLIER    = 2.5      # SL = ATR × 2.5
    MIN_SAMPLES_KELLY    = 6        # mínimo de trades para Kelly estadístico

    # ── Volatilidad inversa ───────────────────────────────────────────────────
    VOL_WINDOW = 20                 # ATR rolling para normalización
    VOL_NORMALIZE_TARGET = 0.02     # objetivo de volatilidad diaria (2%)

    def __init__(
        self,
        initial_capital: float = 100_000.0,
        state_path: str = "data/operation/risk_state.json",
    ) -> None:
        self.capital = initial_capital
        self.state_path = Path(state_path)
        self._state = RiskState(
            peak_equity=initial_capital,
            current_equity=initial_capital,
            kelly_fraction_active=self.BASE_KELLY_FRACTION,
        )
        self._trade_history: list[dict] = []
        self._atr_history: dict[str, list[float]] = {}

        self._load_state()

    # ── Cálculo de tamaño de posición ────────────────────────────────────────

    def compute_size(
        self,
        symbol: str,
        price: float,
        atr: float,
        signal_confidence: float = 0.65,
        capital_override: float | None = None,
    ) -> PositionSize:
        """Calcula tamaño de posición usando Kelly + Volatilidad Inversa."""
        capital = capital_override or self.capital

        # Circuit breaker activo
        if self._state.circuit_breaker_active:
            if time.time() < self._state.circuit_breaker_until:
                remaining = (self._state.circuit_breaker_until - time.time()) / 60
                logger.warning("Circuit breaker activo — %.0f min restantes", remaining)
                return PositionSize(
                    shares=0, kelly_fraction=0.0, inv_vol_fraction=0.0,
                    final_fraction=0.0, risk_usd=0.0, capital_pct=0.0,
                    method="circuit_breaker_blocked"
                )
            else:
                self._state.circuit_breaker_active = False
                logger.info("Circuit breaker desactivado")

        # ── Kelly fraccionado ────────────────────────────────────────────────
        kelly_f = self._compute_kelly_fraction()
        # Escalar por confianza de la señal
        kelly_f *= signal_confidence

        # ── Volatilidad inversa ──────────────────────────────────────────────
        inv_vol_f = self._compute_inv_vol_fraction(symbol, atr, price)

        # ── Fracción final ───────────────────────────────────────────────────
        final_f = min(kelly_f, inv_vol_f, self.MAX_POSITION_PCT)
        final_f = min(final_f, self._state.kelly_fraction_active)

        # ── Riesgo por ATR (1% del capital) ──────────────────────────────────
        max_risk_usd = capital * self.MAX_RISK_PER_TRADE
        sl_distance  = atr * self.ATR_SL_MULTIPLIER
        shares_by_risk = int(max_risk_usd / sl_distance) if sl_distance > 0 else 0

        # ── Shares por fracción Kelly ─────────────────────────────────────────
        kelly_capital = capital * final_f
        shares_by_kelly = int(kelly_capital / price) if price > 0 else 0

        # Final: mínimo de ambos métodos
        shares = min(shares_by_risk, shares_by_kelly)
        shares = max(shares, 0)

        risk_usd = shares * sl_distance
        capital_pct = (shares * price / capital * 100) if capital > 0 else 0.0

        size = PositionSize(
            shares          = shares,
            kelly_fraction  = kelly_f,
            inv_vol_fraction= inv_vol_f,
            final_fraction  = final_f,
            risk_usd        = round(risk_usd, 2),
            capital_pct     = round(capital_pct, 2),
            method          = "kelly_inv_vol",
        )

        logger.info(
            "RISK %s: %d shares | Kelly=%.3f InvVol=%.3f | Riesgo=$%.2f (%.2f%%)",
            symbol, shares, kelly_f, inv_vol_f, risk_usd, capital_pct
        )
        return size

    # ── Kelly fraccionado ────────────────────────────────────────────────────

    def _compute_kelly_fraction(self) -> float:
        """f* = (p*b - q) / b × 0.25"""
        if (self._state.total_trades < self.MIN_SAMPLES_KELLY or
                self._state.avg_loss_usd <= 0):
            return self.BASE_KELLY_FRACTION

        p = max(0.01, min(0.99, self._state.win_rate_rolling))
        q = 1 - p
        b = (self._state.avg_win_usd / self._state.avg_loss_usd
             if self._state.avg_loss_usd > 0 else 1.0)

        kelly_raw = (p * b - q) / b
        kelly_fractional = max(0.0, kelly_raw * self.BASE_KELLY_FRACTION)

        # Modo defensivo por drawdown
        if self._state.drawdown_pct >= self.DRAWDOWN_THRESHOLD:
            kelly_fractional = min(kelly_fractional, self.DEFENSIVE_FRACTION)
            logger.warning(
                "Modo defensivo: DD=%.2f%% → fracción=%.3f",
                self._state.drawdown_pct * 100, kelly_fractional
            )

        return kelly_fractional

    # ── Volatilidad inversa ───────────────────────────────────────────────────

    def _compute_inv_vol_fraction(self, symbol: str, atr: float, price: float) -> float:
        """Fracción inversamente proporcional a la volatilidad diaria."""
        if price <= 0 or atr <= 0:
            return self.MAX_POSITION_PCT

        # Registrar ATR normalizado
        atr_norm = atr / price
        if symbol not in self._atr_history:
            self._atr_history[symbol] = []
        self._atr_history[symbol].append(atr_norm)
        if len(self._atr_history[symbol]) > self.VOL_WINDOW:
            self._atr_history[symbol].pop(0)

        avg_vol = sum(self._atr_history[symbol]) / len(self._atr_history[symbol])
        if avg_vol <= 0:
            return self.MAX_POSITION_PCT

        # Fracción inversamente proporcional: más volátil → menor posición
        inv_vol = self.VOL_NORMALIZE_TARGET / avg_vol
        return min(inv_vol, self.MAX_POSITION_PCT)

    # ── Actualización de estado post-trade ───────────────────────────────────

    def record_trade(self, pnl_usd: float, symbol: str = "") -> None:
        """Registra resultado de trade y actualiza estado de riesgo."""
        self._trade_history.append({
            "ts": time.time(),
            "symbol": symbol,
            "pnl": pnl_usd,
        })

        # Actualizar equity y drawdown
        self._state.current_equity += pnl_usd
        self._state.peak_equity = max(self._state.peak_equity, self._state.current_equity)
        dd = ((self._state.peak_equity - self._state.current_equity) /
              self._state.peak_equity) if self._state.peak_equity > 0 else 0.0
        self._state.drawdown_pct = dd

        # Win rate rolling (últimos 30 trades)
        recent = self._trade_history[-30:]
        wins = sum(1 for t in recent if t["pnl"] > 0)
        self._state.win_rate_rolling = wins / len(recent)
        self._state.total_trades += 1

        if pnl_usd > 0:
            self._state.winning_trades += 1
            self._state.consecutive_losses = 0
            wins_pnl = [t["pnl"] for t in recent if t["pnl"] > 0]
            self._state.avg_win_usd = sum(wins_pnl) / len(wins_pnl) if wins_pnl else 0.0
        else:
            self._state.consecutive_losses += 1
            losses_pnl = [abs(t["pnl"]) for t in recent if t["pnl"] < 0]
            self._state.avg_loss_usd = sum(losses_pnl) / len(losses_pnl) if losses_pnl else 0.0

        # Circuit breaker
        if (dd >= self.DRAWDOWN_THRESHOLD or
                self._state.consecutive_losses >= self.CONSEC_LOSSES_CB):
            self._activate_circuit_breaker(dd)

        self._state.kelly_fraction_active = self._compute_kelly_fraction()
        self._state.last_updated = time.time()
        self._save_state()

        logger.info(
            "Trade: PnL=$%.2f | Equity=$%.2f | DD=%.2f%% | WR=%.2f%% | CB=%s",
            pnl_usd, self._state.current_equity,
            dd * 100, self._state.win_rate_rolling * 100,
            "✓" if self._state.circuit_breaker_active else "✗"
        )

    def _activate_circuit_breaker(self, dd: float) -> None:
        self._state.circuit_breaker_active = True
        self._state.circuit_breaker_until = time.time() + self.CB_PAUSE_MIN * 60
        logger.critical(
            "CIRCUIT BREAKER: DD=%.2f%% | Pérdidas consecutivas=%d | Pausa=%d min",
            dd * 100, self._state.consecutive_losses, self.CB_PAUSE_MIN
        )

    # ── Persistencia de estado ────────────────────────────────────────────────

    def _save_state(self) -> None:
        try:
            self.state_path.parent.mkdir(parents=True, exist_ok=True)
            self.state_path.write_text(json.dumps(asdict(self._state), indent=2))
        except Exception as exc:
            logger.error("Error guardando estado de riesgo: %s", exc)

    def _load_state(self) -> None:
        if not self.state_path.exists():
            return
        try:
            data = json.loads(self.state_path.read_text())
            # Solo restaurar campos que existen en el dataclass
            valid = {k: v for k, v in data.items() if hasattr(self._state, k)}
            for k, v in valid.items():
                setattr(self._state, k, v)
            logger.info("Estado de riesgo restaurado: DD=%.2f%% WR=%.2f%%",
                        self._state.drawdown_pct * 100, self._state.win_rate_rolling * 100)
        except Exception as exc:
            logger.warning("Error cargando estado de riesgo: %s", exc)

    # ── Acceso a estado ───────────────────────────────────────────────────────

    def state(self) -> RiskState:
        return self._state

    def status_dict(self) -> dict:
        s = self._state
        return {
            "equity":              s.current_equity,
            "peak_equity":         s.peak_equity,
            "drawdown_pct":        round(s.drawdown_pct * 100, 2),
            "consecutive_losses":  s.consecutive_losses,
            "win_rate_pct":        round(s.win_rate_rolling * 100, 2),
            "kelly_fraction":      round(s.kelly_fraction_active, 4),
            "circuit_breaker":     s.circuit_breaker_active,
            "total_trades":        s.total_trades,
            "avg_win_usd":         round(s.avg_win_usd, 2),
            "avg_loss_usd":        round(s.avg_loss_usd, 2),
        }

    def reset_circuit_breaker(self) -> None:
        """Desactiva manualmente el circuit breaker (requiere confirmación del operador)."""
        self._state.circuit_breaker_active = False
        self._state.consecutive_losses = 0
        self._save_state()
        logger.warning("Circuit breaker reseteado manualmente")
