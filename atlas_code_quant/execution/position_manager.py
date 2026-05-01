# ATLAS-Quant — Gestión determinista de estado de posiciones
"""Módulo de ciclo de vida de posiciones abiertas.

Responsabilidades:
  1. PositionState  — estado rico: MAE/MFE, trailing, barras, parciales, breakeven.
  2. Funciones puras — testables sin instanciar nada:
       update_trailing_stop(), check_close_conditions(), apply_partial_close()
  3. PositionManager — registro de posiciones + integración con ProductionGuard
     y KellyRiskEngine.

Diseño:
  - Las funciones puras devuelven copias (sin side-effects) → fácil unit-test.
  - PositionManager es el único punto de mutación de estado.
  - LiveLoop._run_cycle() lo llama ANTES de evaluar nuevas entradas:
      closes = pm.update_cycle(sym, price, regime, eod_bars_left)
      → ejecuta los CloseSignal
      → luego evalúa entradas solo para símbolos SIN posición abierta.

Prioridad de cierre (check_close_conditions):
  1. Stop Loss
  2. Take Profit completo (2R)
  3. Trailing Stop cruzado
  4. Cierre parcial 1R (REDUCE 50% + breakeven)
  5. EOD (N barras antes de cierre)
  6. Time exit (max_hold_bars)
  7. Régimen contrario
"""
from __future__ import annotations

import copy
import logging
import os
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

logger = logging.getLogger("atlas.execution.position_manager")

# ── Parámetros por defecto (override con variables de entorno) ────────────────
_ATR_SL_MULT    = float(os.getenv("ATLAS_ATR_SL_MULT",    "2.5"))
_ATR_TP_MULT    = float(os.getenv("ATLAS_ATR_TP_MULT",    "2.0"))
_PARTIAL_RATIO  = float(os.getenv("ATLAS_PARTIAL_RATIO",  "0.5"))   # 50% en 1R
_BARS_PER_DAY   = int(os.getenv("ATLAS_BARS_PER_DAY",     "4680"))  # 5s × 4680 ≈ 6.5h
_EOD_CLOSE_BARS = int(os.getenv("ATLAS_EOD_CLOSE_BARS",   "180"))   # 15 min antes cierre
_MAX_HOLD_DAYS  = int(os.getenv("ATLAS_MAX_HOLD_DAYS",    "10"))


# ── SignalKind ─────────────────────────────────────────────────────────────────

class SignalKind(str, Enum):
    """Tipos granulares de señal (extiende SignalType con semántica de cierre)."""
    OPEN_LONG     = "OPEN_LONG"
    OPEN_SHORT    = "OPEN_SHORT"
    ADD           = "ADD"           # añadir a posición existente (piramidaje)
    REDUCE        = "REDUCE"        # cierre parcial (1R breakeven)
    CLOSE_TP      = "CLOSE_TP"      # Take Profit completo
    CLOSE_SL      = "CLOSE_SL"      # Stop Loss
    CLOSE_TRAIL   = "CLOSE_TRAIL"   # Trailing Stop
    CLOSE_TIME    = "CLOSE_TIME"    # Time exit (max barras)
    CLOSE_REVERSE = "CLOSE_REVERSE" # Régimen contrario
    CLOSE_EOD     = "CLOSE_EOD"     # Fin de sesión
    FLAT          = "FLAT"


# ── PositionState ──────────────────────────────────────────────────────────────

@dataclass
class PositionState:
    """Estado completo de una posición abierta.

    Extiende el concepto de portfolio.Position con:
    - Trailing stop dinámico (nunca retrocede)
    - MAE/MFE por unidad (Maximum Adverse/Favorable Excursion)
    - Conteo de barras + max_hold_bars para time exit
    - Estado de cierre parcial (breakeven tras 1R)
    """
    symbol:               str
    side:                 str    # "long" | "short"
    qty:                  float  # unidades iniciales
    qty_remaining:        float  # unidades tras cierres parciales
    entry_price:          float
    stop_loss:            float
    take_profit:          float  # TP completo  (2R = entry ± 2×ATR)
    take_profit_partial:  float  # TP parcial   (1R = entry ± 1×ATR)
    trailing_stop:        float
    atr:                  float

    opened_at:            float = field(default_factory=time.time)
    last_update_at:       float = field(default_factory=time.time)
    bars_held:            int   = 0
    max_hold_bars:        int   = _MAX_HOLD_DAYS * _BARS_PER_DAY
    strategy:             str   = ""
    regime_at_open:       str   = ""

    partial_closed:       bool  = False   # True → ya se hizo cierre parcial 1R
    breakeven_active:     bool  = False   # True → SL movido a entry

    # MAE / MFE (precio, no USD)
    max_favorable_price:  float = 0.0
    max_adverse_price:    float = 0.0

    def __post_init__(self) -> None:
        self.max_favorable_price = self.entry_price
        self.max_adverse_price   = self.entry_price

    # ── Propiedades calculadas ────────────────────────────────────────────────

    @property
    def risk_per_unit(self) -> float:
        """Riesgo inicial 1R en precio (distance SL → entry)."""
        return abs(self.entry_price - self.stop_loss)

    @property
    def mfe_usd(self) -> float:
        """Maximum Favorable Excursion total en USD (posición actual)."""
        if self.side == "long":
            return (self.max_favorable_price - self.entry_price) * self.qty_remaining
        return (self.entry_price - self.max_favorable_price) * self.qty_remaining

    @property
    def mae_usd(self) -> float:
        """Maximum Adverse Excursion total en USD."""
        if self.side == "long":
            return (self.entry_price - self.max_adverse_price) * self.qty_remaining
        return (self.max_adverse_price - self.entry_price) * self.qty_remaining

    def unrealized_pnl(self, current_price: float) -> float:
        if self.side == "long":
            return (current_price - self.entry_price) * self.qty_remaining
        return (self.entry_price - current_price) * self.qty_remaining


# ── Funciones puras ────────────────────────────────────────────────────────────

def update_trailing_stop(
    pos: PositionState,
    current_price: float,
    k: float = _ATR_SL_MULT,
) -> PositionState:
    """Actualiza trailing stop y MAE/MFE. Devuelve copia (función pura).

    Regla inamovible:
      LONG:  trailing = max(trailing_actual, high_reciente − k×ATR)   nunca baja
      SHORT: trailing = min(trailing_actual, low_reciente  + k×ATR)   nunca sube
    """
    new = copy.copy(pos)
    new.last_update_at = time.time()
    new.bars_held += 1

    if pos.side == "long":
        new.max_favorable_price = max(pos.max_favorable_price, current_price)
        new.max_adverse_price   = min(pos.max_adverse_price,   current_price)
        candidate = new.max_favorable_price - k * pos.atr
        new.trailing_stop = max(pos.trailing_stop, candidate)
    else:
        new.max_favorable_price = min(pos.max_favorable_price, current_price)
        new.max_adverse_price   = max(pos.max_adverse_price,   current_price)
        candidate = new.max_favorable_price + k * pos.atr
        new.trailing_stop = min(pos.trailing_stop, candidate)

    return new


def check_close_conditions(
    pos: PositionState,
    current_price: float,
    current_regime: str = "",
    eod_bars_left: int = 9_999,
) -> tuple[Optional[str], float]:
    """Evalúa condiciones de cierre. Devuelve (signal_kind | None, qty_to_close).

    Función pura: no modifica pos, no tiene side-effects.

    Prioridad:
      1. SL        → CLOSE_SL     (total)
      2. TP        → CLOSE_TP     (total)
      3. Trailing  → CLOSE_TRAIL  (total)
      4. 1R        → REDUCE       (parcial _PARTIAL_RATIO)   [solo si no partial_closed]
      5. EOD       → CLOSE_EOD    (total)
      6. Time      → CLOSE_TIME   (total)
      7. Régimen   → CLOSE_REVERSE(total)
    """
    qty = pos.qty_remaining

    if pos.side == "long":
        # 1. Stop Loss
        if current_price <= pos.stop_loss:
            return SignalKind.CLOSE_SL, qty
        # 2. Take Profit completo
        if current_price >= pos.take_profit:
            return SignalKind.CLOSE_TP, qty
        # 3. Trailing stop cruzado (solo si está por encima del SL inicial)
        if pos.trailing_stop > pos.stop_loss and current_price <= pos.trailing_stop:
            return SignalKind.CLOSE_TRAIL, qty
        # 4. Parcial 1R
        if not pos.partial_closed and current_price >= pos.take_profit_partial:
            return SignalKind.REDUCE, round(qty * _PARTIAL_RATIO, 6)

    else:  # short
        # 1. Stop Loss
        if current_price >= pos.stop_loss:
            return SignalKind.CLOSE_SL, qty
        # 2. Take Profit completo
        if current_price <= pos.take_profit:
            return SignalKind.CLOSE_TP, qty
        # 3. Trailing stop cruzado
        if pos.trailing_stop < pos.stop_loss and current_price >= pos.trailing_stop:
            return SignalKind.CLOSE_TRAIL, qty
        # 4. Parcial 1R
        if not pos.partial_closed and current_price <= pos.take_profit_partial:
            return SignalKind.REDUCE, round(qty * _PARTIAL_RATIO, 6)

    # 5. EOD
    if eod_bars_left <= _EOD_CLOSE_BARS:
        return SignalKind.CLOSE_EOD, qty

    # 6. Time exit
    if pos.bars_held >= pos.max_hold_bars:
        return SignalKind.CLOSE_TIME, qty

    # 7. Régimen contrario (solo si fue LONG en régimen bull o SHORT en bear)
    if current_regime:
        long_reverse  = (pos.side == "long"  and current_regime == "bear"
                         and pos.regime_at_open == "bull")
        short_reverse = (pos.side == "short" and current_regime == "bull"
                         and pos.regime_at_open == "bear")
        if long_reverse or short_reverse:
            return SignalKind.CLOSE_REVERSE, qty

    return None, 0.0


def apply_partial_close(
    pos: PositionState,
    qty_closed: float,
) -> PositionState:
    """Aplica cierre parcial. Mueve SL a breakeven. Devuelve copia (función pura)."""
    new = copy.copy(pos)
    new.qty_remaining  -= qty_closed
    new.partial_closed  = True
    new.last_update_at  = time.time()

    if not pos.breakeven_active:
        new.stop_loss       = pos.entry_price   # inamovible hacia abajo desde aquí
        new.trailing_stop   = max(pos.trailing_stop, pos.entry_price) \
                              if pos.side == "long" \
                              else min(pos.trailing_stop, pos.entry_price)
        new.breakeven_active = True
        logger.info(
            "[PM] %s 1R → SL breakeven @ %.4f | qty restante=%.4f",
            pos.symbol, pos.entry_price, new.qty_remaining,
        )
    return new


# ── CloseSignal ────────────────────────────────────────────────────────────────

@dataclass
class CloseSignal:
    """Señal de cierre emitida por PositionManager para que LiveLoop la ejecute."""
    symbol:        str
    signal_kind:   str          # SignalKind.*
    side:          str          # "long" | "short"
    qty_to_close:  float
    current_price: float
    entry_price:   float
    stop_loss:     float
    take_profit:   float
    atr:           float
    strategy:      str
    is_partial:    bool  = False
    timestamp:     float = field(default_factory=time.time)


# ── PositionManager ────────────────────────────────────────────────────────────

class PositionManager:
    """Registro y gestión del ciclo de vida de posiciones abiertas.

    Coordina con:
      - ProductionGuard.gate_order() — antes de abrir o cerrar (total)
      - KellyRiskEngine.record_trade() — al cerrar con PnL realizado
      - Portfolio.close_position()    — contabilidad (opcional)

    Uso en LiveLoop::

        pm = PositionManager(guard=guard, risk_engine=risk)

        # Al abrir posición (resultado de signal BUY/SELL):
        pm.open(symbol, side, qty, entry_price, atr, strategy, regime)

        # En cada ciclo para cada símbolo:
        closes = pm.update_cycle(symbol, price, regime, eod_bars_left)
        for close in closes:
            executor.execute_close(close)

        # Al final de sesión:
        closes = pm.close_all_eod(prices)
    """

    def __init__(
        self,
        guard=None,       # ProductionGuard
        risk_engine=None, # KellyRiskEngine
        portfolio=None,   # Portfolio (contabilidad interna, opcional)
    ) -> None:
        self._positions:     dict[str, PositionState] = {}
        self._guard          = guard
        self._risk           = risk_engine
        self._portfolio      = portfolio
        self._closed_history: list[dict] = []

    # ── Apertura ──────────────────────────────────────────────────────────────

    def open(
        self,
        symbol: str,
        side: str,            # "long" | "short"
        qty: float,
        entry_price: float,
        atr: float,
        strategy: str = "",
        regime: str = "",
        portfolio_value: float = 100_000.0,
        sl_mult: float = _ATR_SL_MULT,
        tp_mult: float = _ATR_TP_MULT,
        max_hold_bars: Optional[int] = None,
    ) -> Optional[PositionState]:
        """Registra apertura de posición.

        Retorna PositionState si se abrió, None si bloqueada o duplicada.
        """
        if symbol in self._positions:
            logger.debug("[PM] Posición ya abierta para %s — ignorando", symbol)
            return None

        # ProductionGuard gate
        if self._guard is not None:
            ok, reason = self._guard.gate_order(
                order_value     = qty * entry_price,
                portfolio_value = portfolio_value,
            )
            if not ok:
                logger.warning("[PM] Apertura %s bloqueada: %s", symbol, reason)
                return None

        # Calcular niveles SL / TP / trailing
        if side == "long":
            sl      = round(entry_price - sl_mult * atr, 4)
            tp      = round(entry_price + tp_mult * atr, 4)
            tp1r    = round(entry_price + 1.0 * atr, 4)
            trail   = sl
        else:
            sl      = round(entry_price + sl_mult * atr, 4)
            tp      = round(entry_price - tp_mult * atr, 4)
            tp1r    = round(entry_price - 1.0 * atr, 4)
            trail   = sl

        pos = PositionState(
            symbol              = symbol,
            side                = side,
            qty                 = qty,
            qty_remaining       = qty,
            entry_price         = entry_price,
            stop_loss           = sl,
            take_profit         = tp,
            take_profit_partial = tp1r,
            trailing_stop       = trail,
            atr                 = atr,
            strategy            = strategy,
            regime_at_open      = regime,
            max_hold_bars       = max_hold_bars or (_MAX_HOLD_DAYS * _BARS_PER_DAY),
        )
        self._positions[symbol] = pos

        logger.info(
            "[PM] APERTURA %s %s qty=%.4f @ %.4f | SL=%.4f TP=%.4f 1R=%.4f trail=%.4f",
            side.upper(), symbol, qty, entry_price, sl, tp, tp1r, trail,
        )
        return pos

    # ── Ciclo de actualización ─────────────────────────────────────────────────

    def update_cycle(
        self,
        symbol: str,
        current_price: float,
        current_regime: str = "",
        eod_bars_left: int = 9_999,
        portfolio_value: float = 100_000.0,
    ) -> list[CloseSignal]:
        """Actualiza trailing/MAE/MFE y evalúa condiciones de cierre.

        Retorna lista de CloseSignal (vacía si no hay nada que cerrar).
        Llamado en cada ciclo de LiveLoop para cada símbolo con posición abierta.
        """
        pos = self._positions.get(symbol)
        if pos is None:
            return []

        # 1. Actualizar trailing stop + MAE/MFE
        pos = update_trailing_stop(pos, current_price)
        self._positions[symbol] = pos

        # 2. Evaluar condiciones de cierre
        kind, qty_close = check_close_conditions(
            pos, current_price, current_regime, eod_bars_left
        )
        if kind is None:
            return []

        is_partial = (kind == SignalKind.REDUCE)

        # ProductionGuard gate para cierres totales en modo live
        if self._guard is not None and not is_partial:
            ok, reason = self._guard.gate_order(
                order_value     = qty_close * current_price,
                portfolio_value = portfolio_value,
            )
            if not ok:
                logger.warning("[PM] Cierre %s bloqueado: %s", symbol, reason)
                return []

        close = CloseSignal(
            symbol        = symbol,
            signal_kind   = kind,
            side          = pos.side,
            qty_to_close  = qty_close,
            current_price = current_price,
            entry_price   = pos.entry_price,
            stop_loss     = pos.stop_loss,
            take_profit   = pos.take_profit,
            atr           = pos.atr,
            strategy      = pos.strategy,
            is_partial    = is_partial,
        )

        # Aplicar estado
        if is_partial:
            self._positions[symbol] = apply_partial_close(pos, qty_close)
        else:
            pnl = _pnl(pos, current_price, qty_close)
            self._close_and_record(symbol, pos, current_price, pnl, kind)

        return [close]

    # ── Cierre forzado EOD ─────────────────────────────────────────────────────

    def close_all_eod(
        self,
        prices: dict[str, float],
        portfolio_value: float = 100_000.0,
    ) -> list[CloseSignal]:
        """Cierra todas las posiciones abiertas. Llamado al final de sesión."""
        signals: list[CloseSignal] = []
        for symbol, pos in list(self._positions.items()):
            price = prices.get(symbol, pos.entry_price)
            close = CloseSignal(
                symbol        = symbol,
                signal_kind   = SignalKind.CLOSE_EOD,
                side          = pos.side,
                qty_to_close  = pos.qty_remaining,
                current_price = price,
                entry_price   = pos.entry_price,
                stop_loss     = pos.stop_loss,
                take_profit   = pos.take_profit,
                atr           = pos.atr,
                strategy      = pos.strategy,
                is_partial    = False,
            )
            pnl = _pnl(pos, price, pos.qty_remaining)
            self._close_and_record(symbol, pos, price, pnl, SignalKind.CLOSE_EOD)
            signals.append(close)
            logger.warning("[PM] EOD CLOSE %s PnL=%.2f", symbol, pnl)
        return signals

    # ── Consultas ─────────────────────────────────────────────────────────────

    def has_position(self, symbol: str) -> bool:
        return symbol in self._positions

    def get_position(self, symbol: str) -> Optional[PositionState]:
        return self._positions.get(symbol)

    def open_symbols(self) -> list[str]:
        return list(self._positions.keys())

    def summary(self) -> dict:
        return {
            "open_positions": len(self._positions),
            "closed_trades":  len(self._closed_history),
            "positions": [
                {
                    "symbol":      p.symbol,
                    "side":        p.side,
                    "qty":         p.qty_remaining,
                    "entry_price": p.entry_price,
                    "stop_loss":   p.stop_loss,
                    "take_profit": p.take_profit,
                    "trailing":    p.trailing_stop,
                    "bars_held":   p.bars_held,
                    "partial":     p.partial_closed,
                    "breakeven":   p.breakeven_active,
                    "mfe_usd":     round(p.mfe_usd, 2),
                    "mae_usd":     round(p.mae_usd, 2),
                }
                for p in self._positions.values()
            ],
        }

    # ── Helpers privados ──────────────────────────────────────────────────────

    def _close_and_record(
        self,
        symbol: str,
        pos: PositionState,
        exit_price: float,
        pnl: float,
        reason: str,
    ) -> None:
        self._positions.pop(symbol, None)
        record = {
            "symbol":      symbol,
            "side":        pos.side,
            "qty":         pos.qty_remaining,
            "entry_price": pos.entry_price,
            "exit_price":  exit_price,
            "pnl":         round(pnl, 4),
            "reason":      reason,
            "bars_held":   pos.bars_held,
            "strategy":    pos.strategy,
            "closed_at":   time.time(),
        }
        self._closed_history.append(record)

        if self._risk is not None:
            try:
                self._risk.record_trade(pnl, symbol=symbol)
            except Exception as exc:
                logger.warning("[PM] record_trade error: %s", exc)

        if self._portfolio is not None:
            try:
                self._portfolio.close_position(symbol, exit_price)
            except Exception as exc:
                logger.warning("[PM] portfolio.close_position error: %s", exc)

        logger.info(
            "[PM] CIERRE %s %s @ %.4f | PnL=%.2f | razón=%s | bars=%d",
            pos.side.upper(), symbol, exit_price, pnl, reason, pos.bars_held,
        )


# ── Función auxiliar ──────────────────────────────────────────────────────────

def _pnl(pos: PositionState, exit_price: float, qty: float) -> float:
    if pos.side == "long":
        return (exit_price - pos.entry_price) * qty
    return (pos.entry_price - exit_price) * qty
