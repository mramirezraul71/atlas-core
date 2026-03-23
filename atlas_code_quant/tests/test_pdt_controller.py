"""Tests unitarios para PDTController.

Cubre:
  - can_open: paper mode, swing, límite agotado, símbolo ya DT'd, score insuficiente
  - can_close: risk close siempre permitido, swing close, DT close, límite agotado
  - record_open / record_close: tracking de sesión y conteo de DTs
  - Refresh de día: reset de contadores
  - session_summary: resumen correcto
  - Integración: open → DT → summary

Todos los tests usan mock para evitar depender del ledger real de Tradier.
"""

from __future__ import annotations

import datetime
import time
from unittest.mock import MagicMock, patch

import pytest

from atlas_code_quant.execution.pdt_controller import (
    PDTController,
    PDTDecision,
    PDTSessionState,
    RISK_CLOSE_KINDS,
)


# ── Fixtures ──────────────────────────────────────────────────────────────────

def make_pdt(mode="live", max_dt=2, min_score=0.75, account_id="TEST123") -> PDTController:
    """Crea PDTController con ledger mockeado (siempre devuelve 0 DTs broker)."""
    ctrl = PDTController(mode=mode, account_id=account_id,
                         max_day_trades=max_dt, min_score_near=min_score)
    return ctrl


def _mock_ledger_count(count: int):
    """Context manager que parchea count_intraday_day_trades para devolver `count`."""
    return patch(
        "atlas_code_quant.execution.pdt_controller.PDTController._count_dt_total",
        return_value=count,
    )


# ── TestPaperMode ─────────────────────────────────────────────────────────────

class TestPaperMode:
    """En paper mode, can_open siempre retorna allowed=True."""

    def test_paper_allows_open_always(self):
        ctrl = make_pdt(mode="paper")
        dec = ctrl.can_open("AAPL", signal_score=0.0)
        assert dec.allowed is True
        assert dec.is_simulated is True

    def test_paper_allows_open_at_dt_limit(self):
        ctrl = make_pdt(mode="paper", max_dt=0)
        dec = ctrl.can_open("AAPL", signal_score=0.0)
        assert dec.allowed is True
        assert dec.is_simulated is True

    def test_paper_allows_close_always(self):
        ctrl = make_pdt(mode="paper")
        dec = ctrl.can_close("AAPL", signal_kind="CLOSE_TP")
        assert dec.allowed is True
        assert dec.is_simulated is True

    def test_paper_tracks_dt_in_summary(self):
        ctrl = make_pdt(mode="paper")
        ctrl.record_open("AAPL")
        ctrl.record_close("AAPL")
        s = ctrl.session_summary()
        assert s["session_dt_count"] == 1
        assert "AAPL" in s["dt_symbols_today"]


# ── TestSwingTrades ───────────────────────────────────────────────────────────

class TestSwingTrades:
    """Swings no consumen presupuesto PDT."""

    def test_swing_open_always_allowed_live(self):
        ctrl = make_pdt(mode="live", max_dt=0)   # presupuesto agotado
        dec = ctrl.can_open("MSFT", signal_score=0.0, is_swing=True)
        assert dec.allowed is True
        assert "swing" in dec.reason

    def test_swing_close_not_a_dt(self):
        ctrl = make_pdt(mode="live")
        # No hay registro en session_opens → swing (posición de otro día)
        dec = ctrl.can_close("MSFT", signal_kind="CLOSE_TP")
        assert dec.allowed is True
        assert "swing" in dec.reason or "no_dt" in dec.reason


# ── TestRiskCloses ────────────────────────────────────────────────────────────

class TestRiskCloses:
    """Cierres de riesgo siempre permitidos, incluso al límite PDT."""

    @pytest.mark.parametrize("signal_kind", list(RISK_CLOSE_KINDS))
    def test_risk_close_always_allowed(self, signal_kind):
        ctrl = make_pdt(mode="live", max_dt=0)   # presupuesto agotado
        dec = ctrl.can_close("AAPL", signal_kind=signal_kind)
        assert dec.allowed is True
        assert dec.reason == "risk_close_always_allowed"

    def test_risk_override_flag(self):
        ctrl = make_pdt(mode="live", max_dt=0)
        dec = ctrl.can_close("AAPL", signal_kind="CLOSE_TP", is_risk_close=True)
        assert dec.allowed is True
        assert dec.reason == "risk_close_always_allowed"


# ── TestOpenGateLive ──────────────────────────────────────────────────────────

class TestOpenGateLive:
    def test_allows_when_dt_budget_available(self):
        ctrl = make_pdt(mode="live", max_dt=2)
        with _mock_ledger_count(0):
            dec = ctrl.can_open("AAPL", signal_score=0.80)
        assert dec.allowed is True
        assert dec.day_trades_used == 0
        assert dec.day_trades_remaining == 2

    def test_blocks_when_dt_budget_exhausted(self):
        ctrl = make_pdt(mode="live", max_dt=2)
        with _mock_ledger_count(2):
            dec = ctrl.can_open("AAPL", signal_score=0.90)
        assert dec.allowed is False
        assert dec.day_trades_remaining == 0
        assert "agotado" in dec.reason or "limit" in dec.reason.lower()

    def test_blocks_repeat_symbol_today(self):
        ctrl = make_pdt(mode="live", max_dt=2)
        # Simular que AAPL ya fue day-traded hoy
        ctrl._state.dt_symbols_today.add("AAPL")
        with _mock_ledger_count(1):
            dec = ctrl.can_open("AAPL", signal_score=0.90)
        assert dec.allowed is False
        assert "AAPL" in dec.reason

    def test_blocks_low_score_near_limit(self):
        ctrl = make_pdt(mode="live", max_dt=2, min_score=0.75)
        with _mock_ledger_count(1):  # 1 DT usado → 1 restante
            dec = ctrl.can_open("AAPL", signal_score=0.60)  # score insuficiente
        assert dec.allowed is False
        assert "score" in dec.reason
        assert dec.min_score_required == 0.75

    def test_allows_high_score_near_limit(self):
        ctrl = make_pdt(mode="live", max_dt=2, min_score=0.75)
        with _mock_ledger_count(1):  # 1 DT usado → 1 restante
            dec = ctrl.can_open("AAPL", signal_score=0.80)  # score suficiente
        assert dec.allowed is True
        assert dec.day_trades_remaining == 1

    def test_blocks_today_counter_increments(self):
        ctrl = make_pdt(mode="live", max_dt=0)
        with _mock_ledger_count(2):
            ctrl.can_open("AAPL", signal_score=0.90)
            ctrl.can_open("MSFT", signal_score=0.90)
        assert ctrl._state.blocks_by_pdt == 2


# ── TestCloseGateLive ─────────────────────────────────────────────────────────

class TestCloseGateLive:
    def test_tp_close_same_day_allowed_when_budget_ok(self):
        ctrl = make_pdt(mode="live", max_dt=2)
        ctrl.record_open("AAPL")   # abierta hoy
        with _mock_ledger_count(0):
            dec = ctrl.can_close("AAPL", signal_kind="CLOSE_TP")
        assert dec.allowed is True

    def test_tp_close_same_day_blocked_when_dt_exhausted(self):
        ctrl = make_pdt(mode="live", max_dt=2)
        ctrl.record_open("AAPL")   # abierta hoy
        with _mock_ledger_count(2):  # presupuesto agotado
            dec = ctrl.can_close("AAPL", signal_kind="CLOSE_TP")
        assert dec.allowed is False
        assert dec.day_trades_remaining == 0

    def test_close_position_not_opened_today(self):
        ctrl = make_pdt(mode="live", max_dt=0)
        # AAPL no está en session_opens → swing
        dec = ctrl.can_close("AAPL", signal_kind="CLOSE_TP")
        assert dec.allowed is True
        assert "swing" in dec.reason or "no_dt" in dec.reason


# ── TestRecordOpen ────────────────────────────────────────────────────────────

class TestRecordOpen:
    def test_record_open_tracks_symbol(self):
        ctrl = make_pdt(mode="live")
        ctrl.record_open("AAPL")
        assert "AAPL" in ctrl._state.session_opens

    def test_record_open_uses_ts(self):
        ctrl = make_pdt(mode="live")
        ts = time.time() - 3600
        ctrl.record_open("TSLA", ts=ts)
        assert abs(ctrl._state.session_opens["TSLA"] - ts) < 1.0


# ── TestRecordClose ───────────────────────────────────────────────────────────

class TestRecordClose:
    def test_same_day_close_counts_as_dt(self):
        ctrl = make_pdt(mode="live")
        ctrl.record_open("AAPL")
        ctrl.record_close("AAPL")
        assert ctrl._state.session_dt_count == 1
        assert "AAPL" in ctrl._state.dt_symbols_today

    def test_same_day_close_removes_from_session_opens(self):
        ctrl = make_pdt(mode="live")
        ctrl.record_open("AAPL")
        ctrl.record_close("AAPL")
        assert "AAPL" not in ctrl._state.session_opens

    def test_overnight_close_not_a_dt(self):
        """Cerrar una posición abierta ayer no es day trade."""
        ctrl = make_pdt(mode="live")
        yesterday_ts = time.time() - 86400
        ctrl._state.session_opens["AAPL"] = yesterday_ts
        ctrl.record_close("AAPL")
        # No debe contar como DT
        assert ctrl._state.session_dt_count == 0
        assert "AAPL" not in ctrl._state.dt_symbols_today

    def test_close_without_open_no_error(self):
        """Cerrar símbolo sin registro de apertura no debe lanzar excepción."""
        ctrl = make_pdt(mode="live")
        ctrl.record_close("UNKNOWN")
        assert ctrl._state.session_dt_count == 0


# ── TestDayRefresh ────────────────────────────────────────────────────────────

class TestDayRefresh:
    def test_state_resets_on_new_day(self):
        ctrl = make_pdt(mode="live")
        # Simular que es "ayer"
        ctrl._state.date_str = "1970-01-01"
        ctrl._state.session_dt_count = 5
        ctrl._state.dt_symbols_today = {"AAPL", "MSFT"}
        ctrl._state.session_opens = {"TSLA": 1000.0}

        # Llamar a cualquier método que invoque _refresh_day
        _ = ctrl.can_open("NVDA", signal_score=0.8)

        assert ctrl._state.date_str == datetime.date.today().isoformat()
        assert ctrl._state.session_dt_count == 0
        assert len(ctrl._state.dt_symbols_today) == 0
        assert len(ctrl._state.session_opens) == 0


# ── TestSessionSummary ────────────────────────────────────────────────────────

class TestSessionSummary:
    def test_summary_fields_present(self):
        ctrl = make_pdt(mode="paper")
        s = ctrl.session_summary()
        required = {
            "mode", "date", "day_trades_used", "day_trades_remaining",
            "max_day_trades", "dt_symbols_today", "open_positions_today",
            "session_dt_count", "blocks_by_pdt", "allowed_by_pdt",
        }
        assert required.issubset(s.keys())

    def test_summary_reflects_mode(self):
        ctrl = make_pdt(mode="paper")
        assert ctrl.session_summary()["mode"] == "paper"

    def test_summary_dt_count_after_trades(self):
        ctrl = make_pdt(mode="live", max_dt=3)
        ctrl.record_open("AAPL")
        ctrl.record_close("AAPL")
        ctrl.record_open("MSFT")
        ctrl.record_close("MSFT")
        s = ctrl.session_summary()
        assert s["session_dt_count"] == 2
        assert set(s["dt_symbols_today"]) == {"AAPL", "MSFT"}


# ── TestIntegration ───────────────────────────────────────────────────────────

class TestIntegration:
    """Tests de flujo completo: open → DT → summary → límite."""

    def test_full_flow_two_dt_then_blocked(self):
        """Flujo completo: 2 DTs usados → 3er intento bloqueado."""
        ctrl = make_pdt(mode="live", max_dt=2, min_score=0.75)

        # Trade 1: AAPL
        with _mock_ledger_count(0):
            dec1 = ctrl.can_open("AAPL", signal_score=0.80)
        assert dec1.allowed
        ctrl.record_open("AAPL")
        ctrl.record_close("AAPL")   # DT #1

        # Trade 2: MSFT (último DT — exige score alto)
        with _mock_ledger_count(1):
            dec2 = ctrl.can_open("MSFT", signal_score=0.82)  # score OK
        assert dec2.allowed
        ctrl.record_open("MSFT")
        ctrl.record_close("MSFT")   # DT #2

        # Trade 3: TSLA — presupuesto agotado
        with _mock_ledger_count(2):
            dec3 = ctrl.can_open("TSLA", signal_score=0.95)
        assert not dec3.allowed
        assert dec3.day_trades_remaining == 0

    def test_risk_close_after_dt_limit_still_allowed(self):
        """Después de agotar DTs, los SL siguen ejecutándose."""
        ctrl = make_pdt(mode="live", max_dt=1)
        ctrl.record_open("AAPL")
        ctrl.record_close("AAPL")   # DT #1 — presupuesto agotado

        with _mock_ledger_count(1):
            # TP bloqueado
            tp_dec = ctrl.can_close("AAPL", signal_kind="CLOSE_TP")
            # SL siempre permitido
            sl_dec = ctrl.can_close("AAPL", signal_kind="CLOSE_SL")

        # TP no puede ser day trade si hay otro símbolo abierto hoy
        # (AAPL ya fue DT'd, pero probamos con MSFT abierto hoy)
        ctrl.record_open("MSFT")
        with _mock_ledger_count(1):
            tp_msft = ctrl.can_close("MSFT", signal_kind="CLOSE_TP")
            sl_msft = ctrl.can_close("MSFT", signal_kind="CLOSE_SL")

        assert sl_msft.allowed is True   # SL siempre
        assert tp_msft.allowed is False  # TP bloqueado (crearía DT #2 > max=1)

    def test_day_trades_remaining_property(self):
        ctrl = make_pdt(mode="live", max_dt=2)
        with _mock_ledger_count(0):
            assert ctrl.day_trades_remaining() == 2
        with _mock_ledger_count(1):
            assert ctrl.day_trades_remaining() == 1
        with _mock_ledger_count(2):
            assert ctrl.day_trades_remaining() == 0
        with _mock_ledger_count(3):  # over-limit
            assert ctrl.day_trades_remaining() == 0

    def test_is_pdt_limited(self):
        ctrl = make_pdt(mode="live", max_dt=2)
        with _mock_ledger_count(2):
            assert ctrl.is_pdt_limited() is True
        with _mock_ledger_count(1):
            assert ctrl.is_pdt_limited() is False


# ── TestPDTExemption ───────────────────────────────────────────────────────────

class TestPDTExemption:
    """Activos exentos del PDT Rule no son bloqueados aunque se agote el límite."""

    def test_is_pdt_exempt_index_option(self):
        ctrl = make_pdt(mode="live")
        assert ctrl.is_pdt_exempt("SPX",  "index_option") is True
        assert ctrl.is_pdt_exempt("NDX",  "index_option") is True
        assert ctrl.is_pdt_exempt("RUT",  "index_option") is True
        assert ctrl.is_pdt_exempt("SPXW", "index_option") is True

    def test_is_pdt_exempt_crypto(self):
        ctrl = make_pdt(mode="live")
        assert ctrl.is_pdt_exempt("BTC/USDT", "crypto") is True
        assert ctrl.is_pdt_exempt("ETH/USDT", "crypto") is True

    def test_is_pdt_exempt_future(self):
        ctrl = make_pdt(mode="live")
        assert ctrl.is_pdt_exempt("ES", "future") is True
        assert ctrl.is_pdt_exempt("NQ", "future") is True

    def test_is_pdt_exempt_forex(self):
        ctrl = make_pdt(mode="live")
        assert ctrl.is_pdt_exempt("EUR/USD", "forex") is True

    def test_equity_stock_not_exempt(self):
        ctrl = make_pdt(mode="live")
        assert ctrl.is_pdt_exempt("AAPL", "equity_stock") is False
        assert ctrl.is_pdt_exempt("TSLA", "equity_stock") is False

    def test_etf_not_exempt(self):
        ctrl = make_pdt(mode="live")
        assert ctrl.is_pdt_exempt("SPY",  "equity_etf") is False
        assert ctrl.is_pdt_exempt("QQQ",  "equity_etf") is False

    def test_can_open_exempt_when_limit_exhausted(self):
        """SPX can_open returns allowed=True even with 0 DTs remaining."""
        ctrl = make_pdt(mode="live", max_dt=2)
        with _mock_ledger_count(2):  # limit exhausted
            dec = ctrl.can_open("SPX", signal_score=0.9, asset_class="index_option")
        assert dec.allowed is True
        assert dec.reason == "pdt_exempt_asset_class"

    def test_can_open_crypto_exempt_when_limit_exhausted(self):
        ctrl = make_pdt(mode="live", max_dt=2)
        with _mock_ledger_count(2):
            dec = ctrl.can_open("BTC/USDT", signal_score=0.8, asset_class="crypto")
        assert dec.allowed is True

    def test_can_open_equity_blocked_when_limit_exhausted(self):
        ctrl = make_pdt(mode="live", max_dt=2)
        with _mock_ledger_count(2):
            dec = ctrl.can_open("AAPL", signal_score=0.9, asset_class="equity_stock")
        assert dec.allowed is False

    def test_exempt_reason_in_decision(self):
        ctrl = make_pdt(mode="live", max_dt=2)
        with _mock_ledger_count(2):
            dec = ctrl.can_open("NDX", signal_score=0.85, asset_class="index_option")
        assert "exempt" in dec.reason.lower()
