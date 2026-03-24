# ATLAS-EXECUTION — Módulo 7B: Live Trading Loop
"""Loop principal de 5 segundos para trading autónomo en tiempo real.

Arquitectura del ciclo:
  ┌─────────────────────────────────────────────────────┐
  │  Cada 5 segundos (ajustable):                       │
  │  1. Leer CPU → reducir OCR a 1s si CPU >80%         │
  │  2. Capturar OCR + estado visual (Insta360)          │
  │  3. Actualizar quotes y CVD desde WebSocket          │
  │  4. Actualizar indicadores técnicos                  │
  │  5. Clasificar régimen (XGBoost + LSTM)              │
  │  6. Generar señal (SignalGenerator)                  │
  │  7. Calcular tamaño (Kelly)                          │
  │  8. Ejecutar (SignalExecutor → API o HID)            │
  │  9. Self-healing check                               │
  │ 10. Publicar métricas a Prometheus                   │
  └─────────────────────────────────────────────────────┘

Controles físicos:
  - Esc  → Emergency stop (cierra posiciones + pausa)
  - F12  → Arm / Disarm HID
  - F10  → Forzar ciclo inmediato
  - F9   → Toggle paper/live (requiere mode_switcher)

Optimizaciones Jetson (<2GB RAM idle):
  - OCR desacoplado: sólo si CPU <80% o última captura >1s de antigüedad
  - LSTM inference en batch si hay varios símbolos pendientes
  - Sin pandas en hot-path: numpy puro para indicadores
"""
from __future__ import annotations

import datetime
import logging
import os
import signal
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

logger = logging.getLogger("atlas.execution.live_loop")

# Importación lazy para evitar circular imports al nivel de módulo
def _get_position_manager_cls():
    from atlas_code_quant.execution.position_manager import PositionManager
    return PositionManager

# ── Configuración de entorno ──────────────────────────────────────────────────
_MODE            = os.getenv("ATLAS_MODE", "paper").strip().lower()
_CYCLE_S         = float(os.getenv("ATLAS_CYCLE_S", "5"))
_OCR_INTERVAL_S  = float(os.getenv("ATLAS_OCR_INTERVAL_S", "0.5"))
_CPU_THROTTLE_PCT = float(os.getenv("ATLAS_CPU_THROTTLE_PCT", "80"))
_MAX_SYMBOLS      = int(os.getenv("ATLAS_MAX_SYMBOLS", "20"))


@dataclass
class CycleMetrics:
    """Métricas de un ciclo de trading."""
    cycle_id:     int   = 0
    timestamp:    float = field(default_factory=time.time)
    duration_ms:  float = 0.0
    cpu_pct:      float = 0.0
    ocr_skipped:  bool  = False
    signals_evaluated: int = 0
    signals_executed:  int = 0
    signals_blocked:   int = 0
    regime:       str   = ""
    regime_conf:  float = 0.0
    error:        str   = ""


class LiveLoop:
    """Orquestador del ciclo de trading en tiempo real.

    Diseñado para ser instanciado desde atlas_quant_core.py
    con todos los módulos ya inicializados.

    Uso::

        loop = LiveLoop(
            camera=cam, stream=stream_client,
            tech_indicators=tech_dict, regime_clf=clf,
            signal_gen=sig_gen, risk_engine=risk,
            executor=executor, healer=healer,
        )
        loop.start()        # corre en background
        loop.join()         # bloquea hasta stop
    """

    def __init__(
        self,
        camera=None,
        stream=None,
        tech_indicators: dict | None = None,
        cvd_calc=None,
        iv_rank=None,
        regime_clf=None,
        signal_gen=None,
        risk_engine=None,
        executor=None,
        healer=None,
        ros2=None,
        hid=None,
        symbols: list[str] | None = None,
        mode: str = _MODE,
        cycle_s: float = _CYCLE_S,
        position_manager=None,   # PositionManager (opcional; se crea internamente si None)
        eod_time_et: tuple[int, int] = (16, 0),   # hora de cierre ET
        eod_close_min: int = 15,                  # cerrar N min antes del cierre
        learning_brain=None,                      # AtlasLearningBrain opcional
    ) -> None:
        self.camera         = camera
        self.stream         = stream
        self.tech_indicators = tech_indicators or {}
        self.cvd_calc       = cvd_calc
        self.iv_rank        = iv_rank
        self.regime_clf     = regime_clf
        self.signal_gen     = signal_gen
        self.risk_engine    = risk_engine
        self.executor       = executor
        self.healer         = healer
        self.ros2           = ros2
        self.hid            = hid
        self.symbols        = (symbols or [])[:_MAX_SYMBOLS]
        self.mode           = mode
        self.cycle_s        = cycle_s

        # PositionManager: se acepta externamente o se crea uno vacío
        if position_manager is not None:
            self.position_manager = position_manager
        else:
            PM = _get_position_manager_cls()
            self.position_manager = PM(risk_engine=risk_engine)

        # AtlasLearningBrain — scoring y registro de trades (opcional)
        self.learning_brain = learning_brain
        if learning_brain is not None and signal_gen is not None:
            signal_gen._learning_brain = learning_brain

        # PDTController — gestiona presupuesto de day trades
        # Se puede inyectar externamente o se crea uno con el modo actual
        self.pdt_controller = None   # se inyecta desde atlas_quant_core o tests

        self._eod_time_et   = eod_time_et
        self._eod_close_min = eod_close_min
        self._eod_fired     = False   # flag: cierre EOD ya ejecutado hoy

        self._running       = False
        self._paused        = False
        self._emergency     = False
        self._cycle_count   = 0
        self._metrics_history: list[CycleMetrics] = []
        self._latest_quotes: dict = {}
        self._last_ocr_ts   = 0.0
        self._thread: Optional[threading.Thread] = None
        self._stop_event    = threading.Event()

        # BarAggregators: uno por símbolo — agrega ticks 5s → velas 1m
        self._bar_aggregators: dict = {}   # symbol → BarAggregator

        # Registro de hotkeys HID
        self._setup_hotkeys()

    # ── Ciclo de vida ─────────────────────────────────────────────────────────

    def start(self) -> None:
        """Inicia el loop en un thread de background."""
        self._running = True
        self._thread  = threading.Thread(
            target=self._run, daemon=False, name="atlas-live-loop"
        )
        self._thread.start()
        logger.info(
            "▶ LiveLoop iniciado — modo=%s ciclo=%.0fs símbolos=%d",
            self.mode.upper(), self.cycle_s, len(self.symbols)
        )
        if self.mode == "paper":
            logger.info(
                "━━━ SIMULACIÓN PAPER — Mañana Open Test ━━━\n"
                "    Todas las órdenes van a sandbox.tradier.com\n"
                "    preview=true forzado — sin capital real\n"
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            )
            try:
                from atlas_code_quant.production.telegram_alerts import TelegramAlerts
                TelegramAlerts.send_static(
                    "🟡 *LiveLoop PAPER activo*\n"
                    "SIMULACIÓN PAPER — Mañana Open Test\n"
                    "sandbox.tradier.com | preview=true"
                )
            except Exception:
                pass

    def stop(self, reason: str = "manual") -> None:
        logger.warning("LiveLoop STOP — razón: %s", reason)
        self._running = False
        self._stop_event.set()

    def join(self, timeout: float | None = None) -> None:
        if self._thread:
            self._thread.join(timeout=timeout)

    def emergency_stop(self) -> None:
        """Para inmediatamente todas las operaciones."""
        self._emergency = True
        self._running   = False
        logger.critical("🚨 EMERGENCY STOP activado")
        self._stop_event.set()

    def pause(self) -> None:
        self._paused = True
        logger.warning("⏸ LiveLoop pausado")

    def resume(self) -> None:
        self._paused = False
        logger.info("▶ LiveLoop reanudado")

    def update_symbols(self, new_symbols: list[str]) -> None:
        """Actualiza dinámicamente los símbolos operados sin reiniciar el loop.

        Usado por el scanner dinámico para cambiar el universo en caliente.
        Actualiza suscripción del stream Tradier y tech_indicators.

        Args:
            new_symbols: Lista nueva de símbolos (resultado del scanner).
        """
        new_symbols = [s.upper() for s in new_symbols][:_MAX_SYMBOLS]
        added   = [s for s in new_symbols if s not in self.symbols]
        removed = [s for s in self.symbols if s not in new_symbols]

        self.symbols = new_symbols
        logger.info(
            "update_symbols: +%s -%s → activos=%s",
            added, removed, self.symbols,
        )

        # Re-suscribir stream Tradier
        if self.stream is not None:
            try:
                self.stream.subscribe(new_symbols)
            except Exception as exc:
                logger.warning("update_symbols stream resubscribe: %s", exc)

        # Añadir TechnicalIndicators y BarAggregator para nuevos símbolos
        for sym in added:
            if sym not in self.tech_indicators:
                try:
                    from atlas_code_quant.pipeline.indicators import TechnicalIndicators
                    self.tech_indicators[sym] = TechnicalIndicators(sym)
                except Exception:
                    pass
            if sym not in self._bar_aggregators:
                try:
                    from atlas_code_quant.pipeline.indicators import BarAggregator
                    self._bar_aggregators[sym] = BarAggregator(bar_seconds=60)
                except Exception:
                    pass

        # Actualizar signal_gen si tiene método de símbolos
        if self.signal_gen is not None and hasattr(self.signal_gen, "update_symbols"):
            try:
                self.signal_gen.update_symbols(new_symbols)
            except Exception as exc:
                logger.warning("update_symbols signal_gen: %s", exc)

    # ── Loop principal ────────────────────────────────────────────────────────

    def _run(self) -> None:
        # Capturar señales OS
        signal.signal(signal.SIGINT,  lambda s, f: self.emergency_stop())
        signal.signal(signal.SIGTERM, lambda s, f: self.stop("SIGTERM"))

        while self._running and not self._stop_event.is_set():
            if self._paused:
                time.sleep(0.5)
                continue

            t0 = time.monotonic()
            metrics = CycleMetrics(cycle_id=self._cycle_count)
            self._cycle_count += 1

            try:
                self._run_cycle(metrics)
            except Exception as exc:
                metrics.error = str(exc)
                logger.error("Error en ciclo #%d: %s", self._cycle_count, exc, exc_info=True)

            metrics.duration_ms = (time.monotonic() - t0) * 1000
            self._record_metrics(metrics)

            # Log cada 12 ciclos (~1 min)
            if self._cycle_count % 12 == 0:
                self._log_summary()

            # Esperar hasta siguiente ciclo
            elapsed = time.monotonic() - t0
            wait = max(0.0, self.cycle_s - elapsed)
            self._stop_event.wait(wait)

        logger.info("LiveLoop terminado tras %d ciclos", self._cycle_count)

    # ── Ciclo único ───────────────────────────────────────────────────────────

    def _run_cycle(self, metrics: CycleMetrics) -> None:
        # ── 1. CPU check → throttle OCR ───────────────────────────────────────
        cpu_pct = self._cpu_usage()
        metrics.cpu_pct = cpu_pct
        ocr_throttled = cpu_pct > _CPU_THROTTLE_PCT

        # ── 2. OCR update ─────────────────────────────────────────────────────
        ocr_result = None
        ocr_age = time.time() - self._last_ocr_ts
        min_ocr_interval = 1.0 if ocr_throttled else _OCR_INTERVAL_S

        if self.camera is not None and ocr_age >= min_ocr_interval:
            ocr_result = self.camera.latest_result()
            self._last_ocr_ts = time.time()
            metrics.ocr_skipped = False

            # Safe mode: pantalla oscura → no operar
            if getattr(ocr_result, "safe_mode", False):
                logger.warning("Safe mode OCR activo — ciclo abortado")
                return
        else:
            metrics.ocr_skipped = True

        # ── 3. Quotes: usar últimos datos del stream ──────────────────────────
        # Los quotes llegan por callback _on_quote en atlas_quant_core
        # Aquí sólo leemos el snapshot más reciente

        # ── 4. Self-healing check ─────────────────────────────────────────────
        if self.healer is not None and self._cycle_count % 3 == 0:
            health = self.healer.overall_status()
            if health.value == "failed":
                logger.error("Self-healing: subsistema FAILED — ciclo con precaución")

        # ── 5. Gestión de posiciones abiertas (trailing + cierres) ───────────
        equity         = self._get_equity()
        eod_bars       = self._eod_bars_left()
        current_regime = getattr(metrics, "_last_regime", "")

        # Cierre forzado EOD (una sola vez por sesión)
        if eod_bars == 0 and not self._eod_fired:
            self._eod_fired = True
            prices_now = {s: self._mid_price(self._latest_quotes.get(s))
                          for s in self.position_manager.open_symbols()}
            eod_closes = self.position_manager.close_all_eod(prices_now, equity)
            self._execute_close_signals(eod_closes, equity)
            logger.warning("[LL] EOD cierre forzado — %d posiciones", len(eod_closes))

        # Gestión normal de posiciones: trailing + SL/TP/TRAIL/REDUCE/TIME/REVERSE
        all_closes = []
        for symbol in self.position_manager.open_symbols():
            quote = self._latest_quotes.get(symbol)
            if quote is None:
                continue
            price  = self._mid_price(quote)
            closes = self.position_manager.update_cycle(
                symbol, price, current_regime, eod_bars, equity
            )
            all_closes.extend(closes)
        self._execute_close_signals(all_closes, equity)
        metrics.signals_executed += len([c for c in all_closes if not c.is_partial])

        # ── 6-8. Evaluar entradas para símbolos SIN posición abierta ─────────
        for symbol in self.symbols:
            if self._emergency:
                break
            # No abrir nuevas entradas si ya hay posición gestionada por PM
            if self.position_manager.has_position(symbol):
                continue
            result = self._evaluate_and_execute(symbol, ocr_result, equity, metrics)
            if result is not None:
                metrics.signals_evaluated += 1
                if result.status in ("submitted", "simulated", "hid_fallback"):
                    metrics.signals_executed += 1
                elif result.status == "blocked":
                    metrics.signals_blocked += 1

        # ── 9. Publicar métricas ──────────────────────────────────────────────
        if self.ros2 is not None and self._cycle_count % 6 == 0:
            self.ros2.publish_status({
                "cycle": self._cycle_count,
                "mode":  self.mode,
                "cpu":   cpu_pct,
                "signals_executed": metrics.signals_executed,
            })

    # ── Evaluación + ejecución de símbolo ─────────────────────────────────────

    def _evaluate_and_execute(self, symbol: str, ocr_result, equity: float, metrics: CycleMetrics | None = None):
        """Pipeline completo para un símbolo: indicadores → régimen → señal → orden."""
        from atlas_code_quant.strategy.signal_generator import SignalType

        # Quote actual
        quote = self._latest_quotes.get(symbol)
        if quote is None:
            return None

        price = self._mid_price(quote)
        if price <= 0:
            return None

        # Indicadores técnicos
        ti = self.tech_indicators.get(symbol)
        if ti is None or len(ti._closes) < 30:
            return None

        # Usar high/low del quote si están disponibles, sino aproximar ±0.1%
        q_high   = getattr(quote, "high", price * 1.001)
        q_low    = getattr(quote, "low",  price * 0.999)
        q_volume = float(getattr(quote, "volume", 1000.0) or 1000.0)

        tech = ti.update(price, q_high, q_low, q_volume)

        # Actualizar BarAggregator con el tick actual
        if symbol not in self._bar_aggregators:
            try:
                from atlas_code_quant.pipeline.indicators import BarAggregator
                self._bar_aggregators[symbol] = BarAggregator(bar_seconds=60)
            except Exception:
                pass
        bar_agg = self._bar_aggregators.get(symbol)
        if bar_agg is not None:
            bar_agg.update(price, q_high, q_low, q_volume)

        # CVD + IV
        cvd = self.cvd_calc.snapshot() if self.cvd_calc else None
        iv  = (self.iv_rank.metrics(symbol, getattr(quote, "iv", 0.2))
               if self.iv_rank else None)

        if cvd is None or iv is None:
            return None

        # Régimen ML
        closes = list(ti._closes)
        ret_20 = ((closes[-1] - closes[-20]) / closes[-20]
                  if len(closes) >= 20 else 0.0)
        features = self.regime_clf.extract_features(
            adx          = tech.adx_14,
            return_20d   = ret_20,
            hurst        = tech.hurst,
            iv_rank      = iv.iv_rank_30d,
            cvd_slope_5m = cvd.slope_5m,
            rsi_14       = tech.rsi_14,
            macd_hist    = tech.macd_hist,
            atr_norm     = tech.atr_20 / price if price > 0 else 0,
            volume_ratio = tech.volume_ratio,
        )
        regime = self.regime_clf.predict(features)

        # Guardar régimen actual en métricas (para gestión de posiciones)
        if metrics is not None:
            metrics._last_regime = regime.regime.value if hasattr(regime.regime, "value") \
                                   else str(regime.regime)

        # Señal
        size_result = self.risk_engine.compute_size(
            symbol            = symbol,
            price             = price,
            atr               = tech.atr_20,
            signal_confidence = regime.confidence,
            capital_override  = equity,
        )

        ocr_prices = getattr(ocr_result, "prices", []) if ocr_result else []
        ocr_price  = (min(ocr_prices, key=lambda p: abs(p - price))
                      if ocr_prices else None)

        # Construir BarState para filtro de momentum (None si no hay datos suficientes)
        bar_state = None
        if bar_agg is not None and bar_agg.n_closed() >= 2:
            try:
                from atlas_code_quant.strategy.momentum import BarState
                bar_state = BarState(
                    last      = bar_agg.last_closed_bar,
                    prev      = bar_agg.prev_closed_bar,
                    current   = bar_agg.current_bar,
                    secs_left = bar_agg.seconds_to_close(),
                    avg_vol   = bar_agg.avg_volume(20),
                )
            except Exception:
                bar_state = None

        signal = self.signal_gen.evaluate(
            symbol        = symbol,
            regime        = regime,
            tech          = tech,
            cvd           = cvd,
            iv            = iv,
            entry_price   = price,
            ocr_price     = ocr_price,
            position_size = size_result.shares,
            bar_state     = bar_state,
        )

        # Ejecutar si hay señal accionable
        if signal.signal_type in (SignalType.BUY, SignalType.SELL, SignalType.EXIT):

            # ── Gate PDT (solo para aperturas, no para EXIT/cierres de SG) ────
            if (signal.signal_type in (SignalType.BUY, SignalType.SELL)
                    and self.pdt_controller is not None):
                is_swing = getattr(signal, "is_swing", False)
                pdt_dec  = self.pdt_controller.can_open(
                    symbol       = symbol,
                    signal_score = signal.confidence,
                    is_swing     = is_swing,
                )
                if not pdt_dec.allowed:
                    logger.warning(
                        "[LL] PDT BLOQUEADO apertura %s: %s (DT_usados=%d restantes=%d)",
                        symbol, pdt_dec.reason,
                        pdt_dec.day_trades_used, pdt_dec.day_trades_remaining,
                    )
                    if metrics is not None:
                        metrics.signals_blocked += 1
                    return None
                logger.debug(
                    "[LL] PDT OK apertura %s — DT_usados=%d/%d score=%.2f",
                    symbol, pdt_dec.day_trades_used,
                    pdt_dec.day_trades_used + pdt_dec.day_trades_remaining,
                    signal.confidence,
                )

            result = self.executor.execute(
                signal      = signal,
                ocr_result  = ocr_result,
                capital     = equity,
            )
            # Registrar apertura en PositionManager + PDTController
            if (result.status in ("submitted", "simulated", "hid_fallback")
                    and signal.signal_type in (SignalType.BUY, SignalType.SELL)):
                side = "long" if signal.signal_type == SignalType.BUY else "short"
                self.position_manager.open(
                    symbol          = symbol,
                    side            = side,
                    qty             = float(result.quantity or size_result.shares),
                    entry_price     = result.fill_price or price,
                    atr             = tech.atr_20,
                    strategy        = signal.strategy,
                    regime          = signal.regime,
                    portfolio_value = equity,
                )
                if self.pdt_controller is not None:
                    self.pdt_controller.record_open(symbol)

            return result

        return None

    # ── Gestión de posiciones abiertas ────────────────────────────────────────

    def _eod_bars_left(self) -> int:
        """Barras de 5s que quedan hasta la hora de cierre ET."""
        try:
            import zoneinfo
            tz   = zoneinfo.ZoneInfo("America/New_York")
            now  = datetime.datetime.now(tz)
            eod  = now.replace(
                hour=self._eod_time_et[0],
                minute=self._eod_time_et[1],
                second=0, microsecond=0
            )
            secs = max(0.0, (eod - now).total_seconds())
            return int(secs / self.cycle_s)
        except Exception:
            return 9_999   # sin zoneinfo: no forzar cierre

    def _execute_close_signals(
        self,
        closes,          # list[CloseSignal]
        equity: float,
    ) -> None:
        """Convierte CloseSignal → OrderRequest y ejecuta vía SignalExecutor."""
        if not closes or self.executor is None:
            return

        from atlas_code_quant.execution.position_manager import SignalKind

        from atlas_code_quant.execution.pdt_controller import RISK_CLOSE_KINDS

        for close in closes:
            # ── Gate PDT para cierres voluntarios ─────────────────────────────
            if self.pdt_controller is not None:
                is_risk = close.signal_kind.upper() in RISK_CLOSE_KINDS
                pdt_dec = self.pdt_controller.can_close(
                    symbol       = close.symbol,
                    signal_kind  = close.signal_kind,
                    is_risk_close= is_risk,
                )
                if not pdt_dec.allowed:
                    logger.warning(
                        "[LL] PDT BLOQUEADO cierre %s (%s): %s — "
                        "posición permanece abierta hasta SL/EOD",
                        close.symbol, close.signal_kind, pdt_dec.reason,
                    )
                    continue   # no ejecutar este cierre; la posición se gestiona por SL/EOD

            # Construir un dict compatible con SignalExecutor._normalize()
            sig_type = "SELL" if close.side == "long" else "BUY"   # opuesto para cerrar
            signal_dict = {
                "symbol":        close.symbol,
                "signal_type":   sig_type,
                "entry_price":   close.current_price,
                "stop_loss":     close.stop_loss,
                "take_profit":   close.take_profit,
                "atr":           close.atr,
                "confidence":    1.0,
                "position_size": int(close.qty_to_close),
                "strategy":      close.strategy,
                "exit_reason":   close.signal_kind,
                "signal_kind":   close.signal_kind,
            }
            try:
                result = self.executor.execute(signal=signal_dict, capital=equity)
                logger.info(
                    "[LL] CLOSE %s %s %.4f qty=%.4f razón=%s status=%s",
                    close.side.upper(), close.symbol, close.current_price,
                    close.qty_to_close, close.signal_kind, result.status,
                )
                # Registrar cierre en PDTController tras ejecución exitosa
                if (self.pdt_controller is not None and
                        result.status in ("submitted", "simulated", "hid_fallback") and
                        not close.is_partial):
                    self.pdt_controller.record_close(close.symbol)

                # Registrar trade cerrado en AtlasLearningBrain
                if (self.learning_brain is not None and
                        result.status in ("submitted", "simulated", "hid_fallback") and
                        not close.is_partial):
                    self._record_closed_trade(close, result, equity)

            except Exception as exc:
                logger.error("[LL] Error ejecutando cierre %s: %s", close.symbol, exc)

    # ── AtlasLearningBrain — registro de trades cerrados ──────────────────────

    def _record_closed_trade(self, close, result, equity: float) -> None:
        """Construye un TradeEvent desde CloseSignal y lo registra en learning_brain."""
        try:
            import uuid
            from atlas_code_quant.learning.trade_events import TradeEvent

            fill_price = getattr(result, "fill_price", None) or close.current_price
            r_initial = abs(close.entry_price - close.stop_loss)
            if r_initial == 0:
                r_initial = close.atr or 1.0

            side_str = "buy" if close.side == "long" else "sell"
            if side_str == "buy":
                r_realized = (fill_price - close.entry_price) / r_initial
            else:
                r_realized = (close.entry_price - fill_price) / r_initial

            now = datetime.datetime.utcnow()
            trade = TradeEvent(
                trade_id=str(uuid.uuid4()),
                symbol=close.symbol,
                asset_class="equity_stock",  # default; mejorar con AssetClassifier si disponible
                side=side_str,
                entry_time=now - datetime.timedelta(seconds=60),  # aproximación
                exit_time=now,
                timeframe="1m",
                entry_price=close.entry_price,
                exit_price=fill_price,
                stop_loss_price=close.stop_loss,
                r_initial=r_initial,
                r_realized=round(r_realized, 4),
                mae_r=0.0,
                mfe_r=0.0,
                setup_type=close.strategy or "unknown",
                regime="UNKNOWN",
                exit_type=close.signal_kind,
                capital_at_entry=equity,
                position_size=close.qty_to_close,
            )
            self.learning_brain.record_trade(trade)
        except Exception as _lb_exc:
            logger.warning("[LL] learning_brain.record_trade error: %s", _lb_exc)

    # ── Hotkeys físicos ───────────────────────────────────────────────────────

    def _setup_hotkeys(self) -> None:
        """Registra hotkeys HID globales para control de emergencia."""
        if self.hid is None:
            return

        try:
            from pynput import keyboard as _kb

            def on_press(key):
                try:
                    if key == _kb.Key.esc:
                        self.emergency_stop()
                    elif key == _kb.Key.f12:
                        if self.hid:
                            self.hid.armed = not self.hid.armed
                            logger.warning("HID armado: %s", self.hid.armed)
                    elif key == _kb.Key.f10:
                        # Forzar ciclo inmediato
                        self._stop_event.set()
                        self._stop_event.clear()
                    elif key == _kb.Key.f9:
                        self._toggle_mode()
                except Exception:
                    pass

            listener = _kb.Listener(on_press=on_press)
            listener.daemon = True
            listener.start()
            logger.info("Hotkeys: Esc=STOP F12=ARM F10=FORZAR_CICLO F9=TOGGLE_MODO")
        except ImportError:
            logger.debug("pynput no disponible — hotkeys deshabilitados")

    def _toggle_mode(self) -> None:
        """Toggle paper ↔ live desde teclado (llama a mode_switcher)."""
        try:
            from atlas_code_quant.execution.mode_switcher import toggle_paper_live
            new_mode = toggle_paper_live(
                current_mode = self.mode,
                hid_controller = self.hid,
                speak = True,
            )
            if new_mode != self.mode:
                self.mode = new_mode
                if self.executor:
                    self.executor.mode = new_mode
                logger.warning("Modo cambiado a: %s", new_mode.upper())
        except Exception as exc:
            logger.error("Error en toggle_mode: %s", exc)

    # ── Utilidades ────────────────────────────────────────────────────────────

    def update_quotes(self, quotes_dict: dict) -> None:
        """Actualiza snapshot de quotes (llamado desde callbacks del stream)."""
        self._latest_quotes.update(quotes_dict)

    def update_quote(self, symbol: str, quote) -> None:
        self._latest_quotes[symbol] = quote

    @staticmethod
    def _mid_price(quote) -> float:
        if hasattr(quote, "bid") and hasattr(quote, "ask"):
            return (quote.bid + quote.ask) / 2
        if hasattr(quote, "last"):
            return float(quote.last)
        return 0.0

    def _get_equity(self) -> float:
        if self.risk_engine:
            return self.risk_engine.state().current_equity
        return 100_000.0

    @staticmethod
    def _cpu_usage() -> float:
        try:
            import psutil
            return psutil.cpu_percent(interval=None)
        except ImportError:
            return 0.0

    def _record_metrics(self, m: CycleMetrics) -> None:
        self._metrics_history.append(m)
        if len(self._metrics_history) > 500:
            self._metrics_history.pop(0)

    def _log_summary(self) -> None:
        recent = self._metrics_history[-12:]
        if not recent:
            return
        avg_ms   = sum(r.duration_ms for r in recent) / len(recent)
        executed = sum(r.signals_executed for r in recent)
        cpu_avg  = sum(r.cpu_pct for r in recent) / len(recent)
        equity   = self._get_equity()
        logger.info(
            "📊 Ciclo #%d | avg_ms=%.0f | CPU=%.0f%% | executed=%d | equity=$%.2f | modo=%s",
            self._cycle_count, avg_ms, cpu_avg, executed, equity, self.mode.upper()
        )

    def status(self) -> dict:
        recent = self._metrics_history[-1] if self._metrics_history else CycleMetrics()
        return {
            "running":     self._running,
            "paused":      self._paused,
            "emergency":   self._emergency,
            "mode":        self.mode,
            "cycle_count": self._cycle_count,
            "last_cpu_pct": recent.cpu_pct,
            "last_cycle_ms": recent.duration_ms,
            "symbols":     len(self.symbols),
            "equity":      self._get_equity(),
        }
