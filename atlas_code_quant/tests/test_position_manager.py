"""Tests unitarios para execution/position_manager.py

Cubre:
  1. update_trailing_stop — LONG y SHORT, regla de no-retroceso
  2. check_close_conditions — SL, TP, TRAIL, REDUCE 1R, EOD, TIME, REVERSE
  3. apply_partial_close — reducción de qty + breakeven
  4. PositionManager.open — gate guard + cálculo de niveles
  5. PositionManager.update_cycle — integración de los pasos anteriores
  6. PositionManager.close_all_eod
"""
from __future__ import annotations

import time
import pytest

from atlas_code_quant.execution.position_manager import (
    PositionState,
    SignalKind,
    apply_partial_close,
    check_close_conditions,
    update_trailing_stop,
    PositionManager,
    _BARS_PER_DAY,
    _EOD_CLOSE_BARS,
)


# ── Fixtures ──────────────────────────────────────────────────────────────────

def make_long(
    entry: float = 100.0,
    atr: float = 2.0,
    qty: float = 10.0,
    regime: str = "bull",
    bars: int = 0,
    partial: bool = False,
    breakeven: bool = False,
    max_hold_bars: int = 10 * _BARS_PER_DAY,
) -> PositionState:
    sl   = round(entry - 2.5 * atr, 4)   # 95.0
    tp   = round(entry + 2.0 * atr, 4)   # 104.0
    tp1r = round(entry + 1.0 * atr, 4)   # 102.0
    pos = PositionState(
        symbol="AAPL", side="long", qty=qty, qty_remaining=qty,
        entry_price=entry, stop_loss=sl, take_profit=tp,
        take_profit_partial=tp1r, trailing_stop=sl, atr=atr,
        strategy="trend", regime_at_open=regime,
        partial_closed=partial, breakeven_active=breakeven,
        max_hold_bars=max_hold_bars,
    )
    pos.bars_held = bars
    return pos


def make_short(
    entry: float = 100.0,
    atr: float = 2.0,
    qty: float = 10.0,
) -> PositionState:
    sl   = round(entry + 2.5 * atr, 4)   # 105.0
    tp   = round(entry - 2.0 * atr, 4)   # 96.0
    tp1r = round(entry - 1.0 * atr, 4)   # 98.0
    return PositionState(
        symbol="SPY", side="short", qty=qty, qty_remaining=qty,
        entry_price=entry, stop_loss=sl, take_profit=tp,
        take_profit_partial=tp1r, trailing_stop=sl, atr=atr,
        strategy="mean_rev", regime_at_open="bear",
    )


# ══════════════════════════════════════════════════════════════════════════════
# 1. update_trailing_stop
# ══════════════════════════════════════════════════════════════════════════════

class TestUpdateTrailingStop:

    def test_long_precio_sube_trailing_sube(self):
        pos = make_long(entry=100, atr=2.0)
        # trailing inicial = 95.0 (= 100 - 2.5×2)
        new = update_trailing_stop(pos, current_price=103.0)
        # candidate = 103 - 2.5×2 = 98.0 > 95.0 → trailing sube
        assert new.trailing_stop == pytest.approx(98.0)
        assert new.max_favorable_price == pytest.approx(103.0)

    def test_long_precio_baja_trailing_no_retrocede(self):
        pos = make_long(entry=100, atr=2.0)
        # Primero sube
        pos = update_trailing_stop(pos, current_price=106.0)
        assert pos.trailing_stop == pytest.approx(106.0 - 5.0)  # 101.0
        # Ahora baja
        pos2 = update_trailing_stop(pos, current_price=104.0)
        # candidate = 104 - 5 = 99 < 101 → trailing NO retrocede
        assert pos2.trailing_stop == pytest.approx(101.0)

    def test_short_precio_baja_trailing_baja(self):
        pos = make_short(entry=100, atr=2.0)
        # trailing inicial = 105.0
        new = update_trailing_stop(pos, current_price=97.0)
        # candidate = 97 + 5 = 102 < 105 → trailing baja
        assert new.trailing_stop == pytest.approx(102.0)
        assert new.max_favorable_price == pytest.approx(97.0)

    def test_short_precio_sube_trailing_no_retrocede(self):
        pos = make_short(entry=100, atr=2.0)
        pos = update_trailing_stop(pos, current_price=96.0)   # trail = 101
        pos2 = update_trailing_stop(pos, current_price=98.0)  # cand = 103 > 101 → no retrocede
        assert pos2.trailing_stop == pytest.approx(101.0)

    def test_bars_held_incrementa(self):
        pos = make_long()
        assert pos.bars_held == 0
        new = update_trailing_stop(pos, current_price=100.0)
        assert new.bars_held == 1

    def test_mae_tracking_long(self):
        pos = make_long(entry=100)
        # precio baja → MAE aumenta
        new = update_trailing_stop(pos, current_price=97.0)
        assert new.max_adverse_price == pytest.approx(97.0)
        # precio sube → MAE no cambia
        new2 = update_trailing_stop(new, current_price=103.0)
        assert new2.max_adverse_price == pytest.approx(97.0)


# ══════════════════════════════════════════════════════════════════════════════
# 2. check_close_conditions
# ══════════════════════════════════════════════════════════════════════════════

class TestCheckCloseConditions:

    def test_sl_long(self):
        pos = make_long(entry=100, atr=2.0)   # SL = 95
        kind, qty = check_close_conditions(pos, current_price=94.5)
        assert kind == SignalKind.CLOSE_SL
        assert qty == pytest.approx(pos.qty_remaining)

    def test_tp_long(self):
        pos = make_long(entry=100, atr=2.0)   # TP = 104
        kind, qty = check_close_conditions(pos, current_price=104.5)
        assert kind == SignalKind.CLOSE_TP
        assert qty == pytest.approx(pos.qty_remaining)

    def test_trailing_long(self):
        pos = make_long(entry=100, atr=2.0)
        # Subir precio para que trailing avance
        pos = update_trailing_stop(pos, current_price=108.0)   # trail = 108-5 = 103
        kind, qty = check_close_conditions(pos, current_price=102.5)
        assert kind == SignalKind.CLOSE_TRAIL
        assert qty == pytest.approx(pos.qty_remaining)

    def test_reduce_1r_long(self):
        pos = make_long(entry=100, atr=2.0)   # 1R TP = 102
        kind, qty = check_close_conditions(pos, current_price=102.5)
        assert kind == SignalKind.REDUCE
        assert qty == pytest.approx(pos.qty_remaining * 0.5)

    def test_no_reduce_si_ya_partial_closed(self):
        pos = make_long(entry=100, atr=2.0, partial=True)
        kind, qty = check_close_conditions(pos, current_price=102.5)
        # 1R ya cerrado; TP completo aún no → None
        assert kind is None

    def test_eod(self):
        pos = make_long(entry=100)
        # current_price=100.5: entre SL y TP, no dispara nada salvo EOD
        kind, qty = check_close_conditions(pos, current_price=100.5, eod_bars_left=0)
        assert kind == SignalKind.CLOSE_EOD

    def test_time_exit(self):
        pos = make_long(entry=100, atr=2.0, bars=10 * _BARS_PER_DAY)
        kind, qty = check_close_conditions(pos, current_price=100.5)
        assert kind == SignalKind.CLOSE_TIME

    def test_reverse_bull_to_bear(self):
        pos = make_long(entry=100, regime="bull")
        kind, qty = check_close_conditions(pos, current_price=100.5,
                                           current_regime="bear")
        assert kind == SignalKind.CLOSE_REVERSE

    def test_no_signal_en_posicion_neutral(self):
        pos = make_long(entry=100, atr=2.0)
        kind, qty = check_close_conditions(pos, current_price=100.5)
        assert kind is None
        assert qty == pytest.approx(0.0)

    def test_sl_short(self):
        pos = make_short(entry=100, atr=2.0)   # SL = 105
        kind, qty = check_close_conditions(pos, current_price=105.5)
        assert kind == SignalKind.CLOSE_SL

    def test_reduce_1r_short(self):
        pos = make_short(entry=100, atr=2.0)   # 1R TP = 98
        kind, qty = check_close_conditions(pos, current_price=97.5)
        assert kind == SignalKind.REDUCE
        assert qty == pytest.approx(pos.qty_remaining * 0.5)


# ══════════════════════════════════════════════════════════════════════════════
# 3. apply_partial_close
# ══════════════════════════════════════════════════════════════════════════════

class TestApplyPartialClose:

    def test_reduce_qty_remaining(self):
        pos = make_long(qty=10)
        new = apply_partial_close(pos, qty_closed=5.0)
        assert new.qty_remaining == pytest.approx(5.0)
        assert new.partial_closed is True

    def test_sl_movido_a_breakeven(self):
        pos = make_long(entry=100, atr=2.0)   # SL original = 95
        new = apply_partial_close(pos, qty_closed=5.0)
        assert new.stop_loss == pytest.approx(100.0)   # breakeven
        assert new.breakeven_active is True

    def test_trailing_no_baja_de_breakeven(self):
        pos = make_long(entry=100, atr=2.0)
        new = apply_partial_close(pos, qty_closed=5.0)
        # trailing debe ser ≥ entry
        assert new.trailing_stop >= new.entry_price

    def test_segunda_llamada_no_mueve_sl_de_nuevo(self):
        pos = make_long(entry=100, atr=2.0)
        pos = apply_partial_close(pos, qty_closed=3.0)
        # El SL ya está en breakeven. Una segunda reducción no lo mueve más.
        pos2 = apply_partial_close(pos, qty_closed=2.0)
        assert pos2.stop_loss == pytest.approx(100.0)


# ══════════════════════════════════════════════════════════════════════════════
# 4. PositionManager — apertura
# ══════════════════════════════════════════════════════════════════════════════

class TestPositionManagerOpen:

    def test_apertura_basica(self):
        pm = PositionManager()
        pos = pm.open("AAPL", "long", qty=5, entry_price=150, atr=3.0)
        assert pos is not None
        assert pm.has_position("AAPL")
        assert pos.stop_loss == pytest.approx(150 - 2.5 * 3.0)
        assert pos.take_profit == pytest.approx(150 + 2.0 * 3.0)
        assert pos.take_profit_partial == pytest.approx(150 + 1.0 * 3.0)

    def test_no_duplicada(self):
        pm = PositionManager()
        pm.open("AAPL", "long", qty=5, entry_price=150, atr=3.0)
        pos2 = pm.open("AAPL", "long", qty=5, entry_price=151, atr=3.0)
        assert pos2 is None

    def test_guard_bloquea_apertura(self):
        class FakeGuard:
            def gate_order(self, order_value, portfolio_value):
                return False, "daily_loss_limit"

        pm = PositionManager(guard=FakeGuard())
        pos = pm.open("SPY", "long", qty=10, entry_price=500, atr=5.0)
        assert pos is None
        assert not pm.has_position("SPY")

    def test_short_niveles_correctos(self):
        pm = PositionManager()
        pos = pm.open("TSLA", "short", qty=3, entry_price=200, atr=5.0)
        assert pos.stop_loss   == pytest.approx(200 + 2.5 * 5.0)   # 212.5
        assert pos.take_profit == pytest.approx(200 - 2.0 * 5.0)   # 190.0
        assert pos.take_profit_partial == pytest.approx(200 - 5.0) # 195.0


# ══════════════════════════════════════════════════════════════════════════════
# 5. PositionManager — update_cycle
# ══════════════════════════════════════════════════════════════════════════════

class TestPositionManagerUpdateCycle:

    def test_sin_posicion_retorna_vacio(self):
        pm = PositionManager()
        closes = pm.update_cycle("AAPL", 155.0)
        assert closes == []

    def test_genera_close_sl(self):
        pm = PositionManager()
        pm.open("AAPL", "long", qty=5, entry_price=150, atr=3.0)
        # SL = 150 - 7.5 = 142.5 → precio 142 → CLOSE_SL
        closes = pm.update_cycle("AAPL", current_price=142.0)
        assert len(closes) == 1
        assert closes[0].signal_kind == SignalKind.CLOSE_SL
        assert not pm.has_position("AAPL")

    def test_genera_reduce_y_aplica_breakeven(self):
        pm = PositionManager()
        pm.open("AAPL", "long", qty=10, entry_price=150, atr=3.0)
        # 1R = 150 + 3 = 153 → precio 153.5 → REDUCE
        closes = pm.update_cycle("AAPL", current_price=153.5)
        assert len(closes) == 1
        assert closes[0].signal_kind == SignalKind.REDUCE
        assert closes[0].is_partial is True
        # Posición sigue abierta con qty reducida y breakeven activo
        assert pm.has_position("AAPL")
        pos = pm.get_position("AAPL")
        assert pos.breakeven_active is True
        assert pos.stop_loss == pytest.approx(150.0)

    def test_close_eod(self):
        pm = PositionManager()
        pm.open("SPY", "long", qty=5, entry_price=500, atr=5.0)
        closes = pm.update_cycle("SPY", current_price=502.0, eod_bars_left=0)
        assert len(closes) == 1
        assert closes[0].signal_kind == SignalKind.CLOSE_EOD
        assert not pm.has_position("SPY")

    def test_risk_engine_record_trade_llamado(self):
        """KellyRiskEngine.record_trade() debe llamarse al cerrar."""
        records = []

        class FakeRisk:
            def record_trade(self, pnl, symbol=""):
                records.append((pnl, symbol))

        pm = PositionManager(risk_engine=FakeRisk())
        pm.open("AAPL", "long", qty=5, entry_price=150, atr=3.0)
        pm.update_cycle("AAPL", current_price=140.0)  # SL cruzado
        assert len(records) == 1
        pnl, sym = records[0]
        assert sym == "AAPL"
        assert pnl < 0   # pérdida


# ══════════════════════════════════════════════════════════════════════════════
# 6. PositionManager.close_all_eod
# ══════════════════════════════════════════════════════════════════════════════

class TestCloseAllEod:

    def test_cierra_todas(self):
        pm = PositionManager()
        pm.open("AAPL", "long",  qty=5, entry_price=150, atr=3.0)
        pm.open("SPY",  "short", qty=3, entry_price=500, atr=8.0)

        prices = {"AAPL": 152.0, "SPY": 497.0}
        closes = pm.close_all_eod(prices)

        assert len(closes) == 2
        assert all(c.signal_kind == SignalKind.CLOSE_EOD for c in closes)
        assert pm.open_symbols() == []

    def test_precio_fallback_a_entry(self):
        pm = PositionManager()
        pm.open("NVDA", "long", qty=2, entry_price=800, atr=10.0)
        # No pasamos precio de NVDA → debe usar entry_price
        closes = pm.close_all_eod(prices={})
        assert closes[0].current_price == pytest.approx(800.0)

    def test_no_crashes_sin_posiciones(self):
        pm = PositionManager()
        closes = pm.close_all_eod(prices={})
        assert closes == []


# ══════════════════════════════════════════════════════════════════════════════
# 7. Integración update_trailing + check_close (pipeline real)
# ══════════════════════════════════════════════════════════════════════════════

class TestTrailingThenClose:

    def test_trailing_luego_cierre_correcto(self):
        """Simula subida de precio, trailing avanza, luego caída → CLOSE_TRAIL."""
        pos = make_long(entry=100, atr=2.0)

        # Precio sube a 110
        for p in [102, 104, 106, 108, 110]:
            pos = update_trailing_stop(pos, float(p))

        # trailing = 110 - 5 = 105
        assert pos.trailing_stop == pytest.approx(105.0)

        # Precio cae a 103 → por debajo del trailing (105) y por debajo del TP (104)
        # → CLOSE_TRAIL (no CLOSE_TP porque 103 < 104 = TP)
        pos = update_trailing_stop(pos, 103.0)
        kind, qty = check_close_conditions(pos, current_price=103.0)
        assert kind == SignalKind.CLOSE_TRAIL

    def test_1r_luego_tp_completo(self):
        """1R parcial → breakeven → precio llega a TP completo."""
        pos = make_long(entry=100, atr=2.0)   # 1R=102, TP=104

        # 1R alcanzado
        kind, qty = check_close_conditions(pos, current_price=102.5)
        assert kind == SignalKind.REDUCE
        pos = apply_partial_close(pos, qty_closed=qty)
        assert pos.stop_loss == pytest.approx(100.0)

        # TP completo
        kind2, qty2 = check_close_conditions(pos, current_price=104.5)
        assert kind2 == SignalKind.CLOSE_TP
        assert qty2 == pytest.approx(pos.qty_remaining)
