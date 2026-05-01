# ATLAS-EXECUTION — Módulo 7A: Signal Executor
"""Ejecuta señales de trading recibidas desde atlas_quant_core._execute_signal().

Flujo:
  1. Recibe TradeSignal (o dict equivalente) del loop principal
  2. Valida confianza visual ≥ umbral configurable
  3. Aplica tamaño Kelly final (shares) vía KellySizer existente
  4. Construye OrderRequest compatible con route_order_to_tradier()
  5. Envía orden: API Tradier → fallback HID si falla
  6. Registra ejecución: journal SQLite + memoria episódica

Diseño para bajo consumo Jetson:
  - Sin threads propios (llamado desde live_loop.py)
  - Pool de re-intentos limitado a 2 (no spin loops)
  - Logging estructurado para Prometheus scraping
"""
from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass, field
from typing import Any, Optional

logger = logging.getLogger("atlas.execution.signal_executor")

# ── Imports del stack existente ───────────────────────────────────────────────
from atlas_code_quant.config.settings import settings
from atlas_code_quant.execution.tradier_execution import (
    TradierOrderBlocked,
    build_tradier_order_payload,
    route_order_to_tradier,
)
from atlas_code_quant.execution.tradier_controls import resolve_account_session
from atlas_code_quant.execution.kelly_sizer import KellySizer
from atlas_code_quant.api.schemas import OrderRequest
from atlas_code_quant.execution.alpaca_paper import (
    AlpacaPaperBroker,
    get_alpaca_paper_broker,
    _tradier_error_is_fallback,
)


# ── Constantes ────────────────────────────────────────────────────────────────

_MODE = os.getenv("ATLAS_MODE", "paper").strip().lower()   # "paper" | "live"
_VISUAL_CONF_THRESHOLD = float(os.getenv("ATLAS_VISUAL_MIN_CONF", "0.92"))
_MAX_API_RETRIES = 2
_RETRY_DELAY_S   = 1.5


@dataclass
class ExecutionResult:
    """Resultado de una ejecución de señal."""
    signal_symbol:   str
    signal_type:     str          # BUY | SELL | EXIT
    order_id:        str = ""
    status:          str = "pending"   # submitted | blocked | hid_fallback | failed | simulated
    mode:            str = _MODE
    quantity:        int = 0
    fill_price:      float = 0.0
    slippage_pct:    float = 0.0
    exec_ms:         float = 0.0
    error:           str = ""
    tradier_response: dict = field(default_factory=dict)
    hid_used:        bool = False
    visual_validated: bool = False
    timestamp:       float = field(default_factory=time.time)


class SignalExecutor:
    """Capa de ejecución que conecta el loop de señales con Tradier y HID.

    Uso::

        executor = SignalExecutor(mode="paper")
        result = executor.execute(signal, ocr_result=cam.latest_result())
    """

    def __init__(
        self,
        mode: str = _MODE,
        hid_controller=None,        # HIDController (opcional, para fallback)
        error_agent=None,           # ErrorMemoryAgent (opcional, para logging)
        journal_service=None,       # JournalService existente (opcional)
    ) -> None:
        self.mode = mode.strip().lower()
        self._hid = hid_controller
        self._error_agent = error_agent
        self._journal = journal_service
        self._kelly = KellySizer(
            fraction=settings.kelly_fraction,
            max_position_pct=settings.kelly_max_position_pct,
            min_samples=settings.kelly_min_samples,
            atr_sl_multiplier=settings.atr_sl_multiplier,
            atr_tp_multiplier=settings.atr_tp_multiplier,
        )
        self._exec_history: list[ExecutionResult] = []

        logger.info(
            "SignalExecutor inicializado — modo=%s visual_threshold=%.0f%%",
            self.mode.upper(), _VISUAL_CONF_THRESHOLD * 100
        )

    # ── Entry point principal ─────────────────────────────────────────────────

    def execute(
        self,
        signal,               # TradeSignal o dict compatible
        ocr_result=None,      # OCRResult de camera_interface (para validación visual)
        capital: float | None = None,
    ) -> ExecutionResult:
        """Ejecuta una señal de trading. Retorna ExecutionResult siempre (nunca lanza)."""
        t0 = time.monotonic()

        # Leer modo dinámicamente para reaccionar a cambios F9 en caliente
        current_mode = os.getenv("ATLAS_MODE", self.mode).strip().lower()

        # Normalizar signal (acepta dataclass o dict)
        sig = self._normalize(signal)

        result = ExecutionResult(
            signal_symbol = sig["symbol"],
            signal_type   = sig["signal_type"],
            mode          = current_mode,
            quantity      = sig.get("position_size", 0),
        )

        # ── 1. Validación visual ───────────────────────────────────────────────
        if ocr_result is not None:
            visual_ok = self._validate_visual(sig, ocr_result)
            result.visual_validated = visual_ok
            if not visual_ok:
                logger.warning(
                    "EXEC BLOQUEADO [%s] — confianza visual baja (ocr_conf=%.2f)",
                    sig["symbol"], getattr(ocr_result, "ocr_confidence", 0)
                )
                # No bloquear en paper — solo advertir
                if current_mode == "live":
                    result.status = "blocked"
                    result.error  = "visual_confidence_below_threshold"
                    result.exec_ms = (time.monotonic() - t0) * 1000
                    self._record(result)
                    return result

        # ── 2. Cantidad final con Kelly ───────────────────────────────────────
        quantity = self._resolve_quantity(sig, capital)
        if quantity <= 0:
            result.status = "blocked"
            result.error  = "quantity_zero_kelly_or_risk"
            result.exec_ms = (time.monotonic() - t0) * 1000
            self._record(result)
            return result
        result.quantity = quantity

        # ── 3. Enviar orden ───────────────────────────────────────────────────
        order_req = self._build_order_request(sig, quantity)
        result = self._send_with_fallback(result, order_req, sig)
        result.exec_ms = (time.monotonic() - t0) * 1000

        # ── 4. Calcular slippage ──────────────────────────────────────────────
        entry = sig.get("entry_price", 0)
        if result.fill_price > 0 and entry > 0:
            result.slippage_pct = abs(result.fill_price - entry) / entry * 100

        self._record(result)
        return result

    # ── Envío con fallback HID ────────────────────────────────────────────────

    def _send_with_fallback(
        self,
        result: ExecutionResult,
        order_req: OrderRequest,
        sig: dict,
    ) -> ExecutionResult:
        """Intenta API Tradier → si falla, usa HID → si falla, simula (paper only)."""
        current_mode = os.getenv("ATLAS_MODE", self.mode).strip().lower()

        # Modo simulado si paper y precio ya en signal
        if current_mode == "paper":
            return self._simulate(result, sig, order_req)

        # ── Intentos vía BrokerRouter ─────────────────────────────────────────
        from atlas_code_quant.execution.broker_router import BrokerRouter
        _router = BrokerRouter(mode=current_mode)
        asset_class = sig.get("asset_class", "equity_stock")

        last_error = ""
        for attempt in range(1, _MAX_API_RETRIES + 1):
            try:
                response = _router.route(order_req, asset_class=asset_class)
                order_id = str(
                    (response.get("tradier_response") or {}).get("id", "")
                )
                result.status          = "submitted"
                result.order_id        = order_id
                result.fill_price      = float(
                    (response.get("tradier_response") or {}).get("avg_fill_price",
                    sig.get("entry_price", 0))
                )
                result.tradier_response = response
                logger.info(
                    "ORDEN ENVIADA [%s] %s qty=%d id=%s",
                    sig["symbol"], sig["signal_type"], result.quantity, order_id
                )
                return result

            except TradierOrderBlocked as exc:
                logger.warning("ORDEN BLOQUEADA [%s]: %s", sig["symbol"], exc)
                result.status = "blocked"
                result.error  = str(exc)
                return result

            except Exception as exc:
                last_error = str(exc)
                logger.error(
                    "API error (intento %d/%d) [%s]: %s",
                    attempt, _MAX_API_RETRIES, sig["symbol"], exc
                )
                if attempt < _MAX_API_RETRIES:
                    time.sleep(_RETRY_DELAY_S)

        # ── Fallback HID ──────────────────────────────────────────────────────
        if self._hid is not None and self._hid.armed:
            try:
                from atlas_code_quant.hardware.control_interface import (
                    HIDOrder, OrderSide, TradierWebController
                )
                side = OrderSide.BUY if sig["signal_type"] == "BUY" else OrderSide.SELL
                hid_order = HIDOrder(
                    symbol     = sig["symbol"],
                    side       = side,
                    quantity   = result.quantity,
                    order_type = "market",
                    reason     = "api_fallback",
                )
                ctrl = TradierWebController(self._hid)
                ok = ctrl.execute_order(hid_order)
                result.status   = "hid_fallback" if ok else "failed"
                result.hid_used = True
                if not ok:
                    result.error = f"HID también falló — último error API: {last_error}"
                logger.warning(
                    "FALLBACK HID [%s]: %s", sig["symbol"],
                    "OK" if ok else "FALLIDO"
                )
            except Exception as exc:
                result.status = "failed"
                result.error  = f"HID exception: {exc}"
                logger.critical("ERROR FATAL HID [%s]: %s", sig["symbol"], exc)
        else:
            result.status = "failed"
            result.error  = f"API falló y HID no armado. Último error: {last_error}"
            logger.critical(
                "EJECUCIÓN FALLIDA [%s] — sin fallback disponible", sig["symbol"]
            )

            if self._error_agent:
                self._error_agent.record_error(
                    error_type  = "api_disconnect",
                    description = f"Orden fallida para {sig['symbol']}: {last_error}",
                    context     = {"signal": sig, "retries": _MAX_API_RETRIES},
                )

        return result

    def _simulate(
        self,
        result: ExecutionResult,
        sig: dict,
        order_req: OrderRequest,
    ) -> ExecutionResult:
        """Paper mode: intenta Tradier sandbox → fallback Alpaca → simulación local."""
        entry = sig.get("entry_price", 0.0)

        # 1. Intentar Tradier sandbox primero
        try:
            response = route_order_to_tradier(order_req)
            tradier_resp = response.get("tradier_response") or {}
            result.status           = "simulated"
            result.order_id         = str(tradier_resp.get("id", f"TRADIER-PAPER-{int(time.time())}"))
            result.fill_price       = float(tradier_resp.get("avg_fill_price") or entry)
            result.tradier_response = response
            logger.info(
                "PAPER/TRADIER [%s] %s qty=%d fill=%.4f",
                sig["symbol"], sig["signal_type"], result.quantity, result.fill_price,
            )
            return result

        except TradierOrderBlocked as exc:
            result.status = "blocked"
            result.error  = str(exc)
            return result

        except Exception as tradier_exc:
            if not _tradier_error_is_fallback(tradier_exc):
                # Error no recuperable (ej. auth) — no tiene sentido ir a Alpaca
                slippage = entry * 0.0005
                fill = entry + slippage if sig["signal_type"] == "BUY" else entry - slippage
                result.status     = "simulated"
                result.fill_price = round(fill, 4)
                result.order_id   = f"PAPER-{int(time.time())}-{sig['symbol']}"
                result.error      = f"tradier_non_fallback: {tradier_exc}"
                logger.warning("PAPER/LOCAL [%s] Tradier error no-fallback: %s", sig["symbol"], tradier_exc)
                return result

            logger.warning(
                "PAPER Tradier sandbox no disponible (%s) — intentando Alpaca paper",
                tradier_exc,
            )

        # 2. Fallback Alpaca paper
        from atlas_code_quant.config.settings import settings as _s
        if getattr(_s, "alpaca_paper_fallback", True):
            alpaca = get_alpaca_paper_broker()
            if alpaca.is_configured():
                side  = "buy" if sig["signal_type"] == "BUY" else "sell"
                ares  = alpaca.place_order(
                    symbol     = sig["symbol"],
                    side       = side,
                    qty        = result.quantity,
                    order_type = "market",
                )
                if ares.status not in ("error", "rejected"):
                    result.status     = "simulated"
                    result.order_id   = f"ALPACA-{ares.order_id}"
                    result.fill_price = ares.fill_price or entry
                    result.error      = ""
                    logger.info(
                        "PAPER/ALPACA [%s] %s qty=%d id=%s",
                        sig["symbol"], side.upper(), result.quantity, ares.order_id,
                    )
                    return result
                else:
                    logger.warning("PAPER/ALPACA orden rechazada [%s]: %s", sig["symbol"], ares.error)
            else:
                logger.warning(
                    "Alpaca paper no configurado (ALPACA_API_KEY vacío). "
                    "Crea cuenta gratis en https://app.alpaca.markets"
                )

        # 3. Último recurso: simulación local sin broker
        slippage = entry * 0.0005
        fill = entry + slippage if sig["signal_type"] == "BUY" else entry - slippage
        result.status     = "simulated"
        result.fill_price = round(fill, 4)
        result.order_id   = f"PAPER-LOCAL-{int(time.time())}-{sig['symbol']}"
        logger.info(
            "PAPER/LOCAL [%s] %s qty=%d fill=%.4f (sin broker)",
            sig["symbol"], sig["signal_type"], result.quantity, fill,
        )
        return result

    # ── Validación visual ─────────────────────────────────────────────────────

    def _validate_visual(self, sig: dict, ocr_result) -> bool:
        """Verifica que la OCR confidence y precio coinciden."""
        conf = getattr(ocr_result, "ocr_confidence", 0.0)
        if conf < _VISUAL_CONF_THRESHOLD:
            return False
        # Verificar precio si hay datos
        prices = getattr(ocr_result, "prices", [])
        entry  = sig.get("entry_price", 0)
        if prices and entry > 0:
            best  = min(prices, key=lambda p: abs(p - entry))
            delta = abs(best - entry) / entry
            if delta > 0.005:   # 0.5% tolerancia
                return False
        return True

    # ── Cantidad final ────────────────────────────────────────────────────────

    def _resolve_quantity(self, sig: dict, capital: float | None) -> int:
        """Usa Kelly del signal si ya viene calculado, o recalcula."""
        qty = int(sig.get("position_size", 0) or 0)
        current_mode = os.getenv("ATLAS_MODE", self.mode).strip().lower()

        if self._should_use_equity_kelly(sig, current_mode=current_mode):
            resolved_qty, reason = self._resolve_equity_kelly_quantity(sig, capital, planned_qty=qty)
            logger.info(
                "EQUITY/KELLY %s qty=%d planned=%d reason=%s",
                sig.get("symbol", ""), resolved_qty, qty, reason,
            )
            return resolved_qty

        if qty > 0:
            return qty

        # Re-calcular Kelly si no viene
        cap = capital or 100_000.0
        price = sig.get("entry_price", 1.0)
        atr   = sig.get("atr", price * 0.02)

        try:
            result = self._kelly.compute(
                sig.get("symbol", ""), sig.get("strategy", "momentum"), []
            )
            qty = int(self._kelly.position_size(cap, price, result, atr))
        except Exception:
            qty = max(1, int(cap * 0.01 / max(atr, 0.01)))

        return max(0, qty)

    def _should_use_equity_kelly(self, sig: dict, *, current_mode: str) -> bool:
        if self._is_exit_signal(sig):
            return False
        if sig.get("use_options") or sig.get("option_strategy_type"):
            return False
        if current_mode == "paper":
            return bool(getattr(settings, "equity_kelly_paper_enabled", True))
        return bool(getattr(settings, "equity_kelly_live_enabled", False))

    @staticmethod
    def _is_exit_signal(sig: dict) -> bool:
        signal_type = str(sig.get("signal_type", "")).upper()
        if signal_type == "EXIT":
            return True
        if str(sig.get("position_effect", "")).strip().lower() == "close":
            return True
        if sig.get("exit_reason"):
            return True
        signal_kind = str(sig.get("signal_kind", "")).strip().lower()
        return "close" in signal_kind or "exit" in signal_kind

    def _resolve_equity_kelly_quantity(
        self,
        sig: dict,
        capital: float | None,
        *,
        planned_qty: int,
    ) -> tuple[int, str]:
        cap = self._safe_float(capital if capital is not None else sig.get("capital"))
        price = self._safe_float(sig.get("entry_price"))
        atr = self._safe_float(sig.get("atr"))
        min_qty = max(1, int(getattr(settings, "equity_kelly_min_qty", 1) or 1))
        risk_pct = float(getattr(settings, "equity_kelly_max_risk_per_trade_pct", 0.01) or 0.01)
        pnl_history = self._extract_pnl_history(sig)

        if cap <= 0 or price <= 0:
            return 0, "missing_capital_or_price"

        symbol = str(sig.get("symbol", "")).strip()
        strategy = str(sig.get("strategy") or sig.get("strategy_type") or "equity").strip()

        if len(pnl_history) >= self._kelly.min_samples:
            kelly_result = self._kelly.compute(symbol, strategy, pnl_history)
            max_units = self._kelly.position_size(
                capital=cap,
                price=price,
                result=kelly_result,
                atr=atr,
                max_risk_per_trade_pct=risk_pct,
            )
            reason = "kelly_history"
        elif planned_qty > 0:
            max_units = self._kelly.max_units(
                capital=cap,
                price=price,
                atr=atr,
                position_pct=self._kelly.max_position_pct,
                max_risk_per_trade_pct=risk_pct,
            )
            reason = "fallback_cap_only"
        else:
            return 0, "insufficient_history_no_seed"

        resolved_qty = int(max_units)
        if planned_qty > 0:
            resolved_qty = min(planned_qty, resolved_qty)
        if resolved_qty < min_qty:
            return 0, f"qty_below_min:{resolved_qty}"
        return max(0, resolved_qty), reason

    @staticmethod
    def _safe_float(value: Any) -> float:
        try:
            out = float(value)
        except (TypeError, ValueError):
            return 0.0
        return out if out > 0 else 0.0

    @staticmethod
    def _extract_pnl_history(sig: dict) -> list[float]:
        raw = sig.get("pnl_history")
        if not isinstance(raw, (list, tuple)):
            return []
        history: list[float] = []
        for item in raw:
            try:
                history.append(float(item))
            except (TypeError, ValueError):
                continue
        return history

    # ── Construcción de OrderRequest ──────────────────────────────────────────

    def _build_order_request(self, sig: dict, quantity: int) -> OrderRequest:
        """Construye OrderRequest — bifurca equity vs opciones."""
        if sig.get("use_options") and sig.get("option_strategy_type"):
            return self._build_option_order_request(sig, quantity)
        return self._build_equity_order_request(sig, quantity)

    def _build_equity_order_request(self, sig: dict, quantity: int) -> OrderRequest:
        """OrderRequest estándar para equity/ETF/crypto/futuros."""
        signal_type = sig.get("signal_type", "BUY").upper()
        side_map    = {"BUY": "buy", "SELL": "sell", "EXIT": "sell"}
        scope       = "live" if self.mode == "live" else "paper"
        return OrderRequest(
            symbol        = sig["symbol"],
            side          = side_map.get(signal_type, "buy"),
            size          = quantity,
            order_type    = "market",
            duration      = "day",
            account_scope = scope,  # type: ignore[arg-type]
            preview       = (self.mode == "paper"),
            tag           = f"atlas_quant_{sig.get('strategy', 'auto')}",
        )

    def _build_option_order_request(self, sig: dict, quantity: int) -> OrderRequest:
        """Construye OrderRequest multi-leg para estrategias de opciones."""
        from atlas_code_quant.execution.option_selector import (
            select_option_contract, pick_expiration,
        )
        from atlas_code_quant.execution.option_chain_cache import OptionChainCache
        from atlas_code_quant.config.settings import settings as _s
        from atlas_code_quant.api.schemas import TradierOrderLeg

        scope  = "live" if self.mode == "live" else "paper"
        symbol = sig["symbol"]

        # Sin cadena real en papel → construir OrderRequest mínimo con hint
        # (la cadena real se obtiene en el ciclo de live_loop si tradier está up)
        legs_raw = sig.get("_option_legs") or []
        if legs_raw:
            legs = [
                TradierOrderLeg(
                    option_symbol = leg.get("symbol", ""),
                    side          = "buy_to_open" if leg.get("side") == "long" else "sell_to_open",
                    quantity      = quantity,
                )
                for leg in legs_raw
                if leg.get("symbol")
            ]
            return OrderRequest(
                symbol        = symbol,
                side          = "buy",
                size          = quantity,
                order_type    = "market",
                duration      = "day",
                account_scope = scope,   # type: ignore[arg-type]
                preview       = (self.mode == "paper"),
                legs          = legs if legs else None,
                tradier_class = "multileg" if len(legs) > 1 else "option",
                tag           = f"atlas_options_{sig.get('option_strategy_type','auto')}",
            )

        # Sin patas precalculadas → orden equity normal como fallback
        logger.warning("[OPTS] %s — sin patas precalculadas, enviando equity", symbol)
        return self._build_equity_order_request(sig, quantity)

    # ── Registro de ejecución ─────────────────────────────────────────────────

    def _record(self, result: ExecutionResult) -> None:
        self._exec_history.append(result)
        if len(self._exec_history) > 1000:
            self._exec_history.pop(0)

        # Registrar en journal si disponible
        if self._journal is not None:
            try:
                self._journal.record_execution(result)
            except Exception as exc:
                logger.debug("Error registrando en journal: %s", exc)

        # Prometheus-friendly log
        logger.info(
            "EXEC_RESULT symbol=%s type=%s status=%s qty=%d fill=%.4f "
            "slippage=%.3f%% hid=%s visual=%s ms=%.0f",
            result.signal_symbol, result.signal_type, result.status,
            result.quantity, result.fill_price, result.slippage_pct,
            result.hid_used, result.visual_validated, result.exec_ms,
        )

    @staticmethod
    def _normalize(signal) -> dict:
        if isinstance(signal, dict):
            return signal
        # TradeSignal dataclass → dict
        keys = [
            "symbol", "signal_type", "entry_price", "take_profit", "stop_loss",
            "atr", "confidence", "regime", "strategy", "position_size",
            "kelly_fraction", "iv_rank", "visual_confirmed", "exit_reason",
        ]
        return {k: getattr(signal, k, None) for k in keys}

    # ── Estadísticas ──────────────────────────────────────────────────────────

    def stats(self) -> dict:
        hist = self._exec_history
        submitted  = sum(1 for r in hist if r.status == "submitted")
        simulated  = sum(1 for r in hist if r.status == "simulated")
        blocked    = sum(1 for r in hist if r.status == "blocked")
        hid_used   = sum(1 for r in hist if r.hid_used)
        avg_slip   = (
            sum(r.slippage_pct for r in hist if r.fill_price > 0) /
            max(1, sum(1 for r in hist if r.fill_price > 0))
        )
        return {
            "mode":         self.mode,
            "total":        len(hist),
            "submitted":    submitted,
            "simulated":    simulated,
            "blocked":      blocked,
            "hid_fallback": hid_used,
            "avg_slippage_pct": round(avg_slip, 4),
        }
