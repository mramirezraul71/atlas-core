"""Módulo 4 — Generador de Señales + Lógica de Entrada/Salida.

Criterios de entrada (todos deben cumplirse):
  1. Scanner: IV Rank >70 + IV/HV Ratio >1.2
  2. Régimen: Bull/Bear/Sideways (confianza ≥65%)
  3. RSI divergencia + MACD hist cruce + Volume spike >1.8×
  4. CVD >+1σ (comprador) o <-1σ (vendedor)
  5. Visual trigger: OCR confirma precio dentro del rango esperado

Criterios de salida:
  - TP: 2×ATR desde entrada
  - SL: 2.5×ATR desde entrada
  - Trailing: activo en régimen Trend
  - Time exit: 5-20 días
  - Divergencia opuesta: señal contraria cierra posición
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

logger = logging.getLogger("atlas.strategy.signals")


class SignalType(str, Enum):
    BUY  = "BUY"
    SELL = "SELL"
    FLAT = "FLAT"    # sin señal
    EXIT = "EXIT"    # salir posición existente


# Re-export para conveniencia — la lógica granular vive en position_manager.py
from atlas_code_quant.execution.position_manager import SignalKind  # noqa: E402


@dataclass
class TradeSignal:
    """Señal de trading generada por el motor de estrategia."""
    symbol: str
    signal_type: SignalType
    timestamp: float
    entry_price: float
    take_profit: float
    stop_loss: float
    atr: float
    confidence: float
    regime: str = ""
    strategy: str = ""
    position_size: int = 0
    kelly_fraction: float = 0.0
    iv_rank: float = 0.0
    cvd_delta: float = 0.0
    rsi: float = 50.0
    volume_ratio: float = 1.0
    visual_confirmed: bool = False
    exit_reason: str = ""
    max_hold_days: int = 10
    trailing_active: bool = False
    metadata: dict = field(default_factory=dict)
    # Tipo granular (opcional — no rompe código existente que no lo use)
    signal_kind: str = SignalKind.FLAT
    # ── Multi-asset (Fases 1-4) ────────────────────────────────────────────
    asset_class: str = "equity_stock"          # AssetClass.value
    use_options: bool = False                  # True → construir orden de opciones
    option_strategy_type: str = ""             # StrategyType hint
    preferred_dte_range: tuple = (14, 45)      # (min_dte, max_dte)
    iv_hv_ratio: float = 1.0                   # IV / HV histórica


@dataclass
class OpenPosition:
    """Posición abierta con estado de trailing y time exit."""
    symbol: str
    side: SignalType
    entry_price: float
    current_price: float
    take_profit: float
    stop_loss: float
    trailing_stop: float
    atr: float
    quantity: int
    opened_at: float
    max_hold_days: int
    strategy: str
    highest_price: float = 0.0   # para trailing long
    lowest_price: float = 999999.0  # para trailing short


class SignalGenerator:
    """Motor central de señales de trading.

    Integra régimen ML, indicadores técnicos, CVD y confirmación visual
    para generar señales de alta probabilidad.

    Uso::

        gen = SignalGenerator()
        signal = gen.evaluate(
            symbol="AAPL",
            regime=regime_output,
            tech=tech_snapshot,
            cvd=cvd_snapshot,
            iv=iv_metrics,
            entry_price=185.50,
            ocr_price=185.48,    # precio confirmado por OCR
        )
    """

    # ── Umbrales de entrada ───────────────────────────────────────────────────
    MIN_IV_RANK           = 70.0     # IV Rank mínimo para considerar
    MIN_IV_HV_RATIO       = 1.2      # IV/HV mínimo
    MIN_VOLUME_SPIKE      = 1.8      # multiplicador de volumen
    MIN_MTF_COHERENCE     = 0.70     # coherencia multi-TF mínima para operar
    RSI_OVERBOUGHT        = 70.0
    RSI_OVERSOLD          = 30.0
    CVD_ZSCORE_THRESHOLD  = 1.0      # desviaciones estándar
    VISUAL_PRICE_TOLERANCE = 0.005   # 0.5% tolerancia OCR vs API

    # ── Parámetros de salida ──────────────────────────────────────────────────
    TP_ATR_MULT = 2.0
    SL_ATR_MULT = 2.5
    MIN_HOLD_DAYS = 5
    MAX_HOLD_DAYS = 20
    TRAILING_ACTIVATION = 1.5  # activar trailing cuando ganancia ≥ 1.5×ATR

    def __init__(self, learning_brain=None, pattern_lab=None) -> None:
        self._positions: dict[str, OpenPosition] = {}
        self._signal_history: list[TradeSignal] = []
        self._cvd_history: list[float] = []   # para z-score
        self._learning_brain = learning_brain  # AtlasLearningBrain opcional
        self._pattern_lab = pattern_lab        # PatternLabService opcional

    # ── Evaluación de señal ───────────────────────────────────────────────────

    def evaluate(
        self,
        symbol: str,
        regime,
        tech,
        cvd,
        iv,
        entry_price: float,
        ocr_price: float | None = None,
        current_capital: float = 100_000.0,
        position_size: int = 0,
        bar_state=None,         # BarState opcional — activa filtro de momentum si se provee
        pattern_lab_ctx=None,   # PatternLabContext opcional — gate de signal_score
        motif_edge: float = 0.5,  # edge_score de MotifLabService [0-1], 0.5=neutral
        tin_score: float = 0.5,   # TechnicalIndicatorNet prob [0-1], 0.5=neutral
        mtf_report=None,          # MultiTFReport opcional — gate de coherencia MTF
    ) -> TradeSignal:
        """Evalúa condiciones y retorna señal de trading o FLAT."""

        atr = tech.atr_20

        # ── 1. Verificar posición abierta (gestión de salida) ──
        if symbol in self._positions:
            exit_signal = self._check_exit(
                self._positions[symbol], entry_price
            )
            if exit_signal is not None:
                return exit_signal

        # ── 2. Scanner: IV Rank y IV/HV ──────────────────────────────────────
        if iv.iv_rank_30d < self.MIN_IV_RANK:
            return self._flat(symbol, entry_price, atr, "iv_rank_below_threshold",
                              iv.iv_rank_30d)

        if iv.iv_hv_ratio < self.MIN_IV_HV_RATIO:
            return self._flat(symbol, entry_price, atr, "iv_hv_ratio_below_threshold",
                              iv.iv_hv_ratio)

        # ── 3. Régimen ML ─────────────────────────────────────────────────────
        from atlas_code_quant.models.regime_classifier import MarketRegime

        if regime.regime == MarketRegime.FLAT:
            return self._flat(symbol, entry_price, atr, "regime_confidence_low",
                              regime.confidence)

        # ── 3.5. Gate Multi-TF coherence ──────────────────────────────────────
        if mtf_report is not None:
            _coh = float(mtf_report.coherence_score)
            if _coh < self.MIN_MTF_COHERENCE:
                return self._flat(
                    symbol, entry_price, atr,
                    f"mtf_coherence_low",
                    _coh,
                )

        # ── 4. Señal técnica ──────────────────────────────────────────────────
        signal_dir = self._get_technical_direction(tech, regime)
        if signal_dir == SignalType.FLAT:
            return self._flat(symbol, entry_price, atr, "no_technical_confluence", 0.0)

        # ── 5. CVD z-score ────────────────────────────────────────────────────
        self._cvd_history.append(cvd.delta_imbalance)
        if len(self._cvd_history) > 200:
            self._cvd_history.pop(0)

        cvd_z = self._compute_zscore(cvd.delta_imbalance)

        if signal_dir == SignalType.BUY and cvd_z < self.CVD_ZSCORE_THRESHOLD:
            return self._flat(symbol, entry_price, atr, "cvd_not_confirming_buy", cvd_z)

        if signal_dir == SignalType.SELL and cvd_z > -self.CVD_ZSCORE_THRESHOLD:
            return self._flat(symbol, entry_price, atr, "cvd_not_confirming_sell", cvd_z)

        # ── 6. Volume spike ───────────────────────────────────────────────────
        if tech.volume_ratio < self.MIN_VOLUME_SPIKE:
            return self._flat(symbol, entry_price, atr, "volume_spike_insufficient",
                              tech.volume_ratio)

        # ── 7. Confirmación visual OCR ────────────────────────────────────────
        visual_ok = self._check_visual(entry_price, ocr_price)

        # ── 7.5. Filtro de momentum (velas 1m) — sólo si se provee bar_state ──
        if bar_state is not None:
            from atlas_code_quant.strategy.momentum import (
                MomentumConfig,
                should_open_momentum_long,
                should_open_momentum_short,
            )
            _mcfg = MomentumConfig()
            if signal_dir == SignalType.BUY:
                if not should_open_momentum_long(bar_state, tech, cvd, regime, _mcfg):
                    return self._flat(symbol, entry_price, atr,
                                      "momentum_not_confirmed_long", 0.0)
            else:
                if not should_open_momentum_short(bar_state, tech, cvd, regime, _mcfg):
                    return self._flat(symbol, entry_price, atr,
                                      "momentum_not_confirmed_short", 0.0)

        # ── 7.8. Pattern Lab gate (signal_score) ──────────────────────────────
        _pattern_score = 0.5   # neutral si no hay contexto
        if pattern_lab_ctx is not None:
            _pattern_score = float(getattr(pattern_lab_ctx, "signal_score", 0.5))
            _min_score = getattr(
                getattr(self._pattern_lab, "config", None), "min_signal_score_to_pass", 0.55
            ) if self._pattern_lab is not None else 0.55
            if _pattern_score < _min_score:
                return self._flat(symbol, entry_price, atr,
                                  "pattern_lab_score_below_threshold", _pattern_score)

        # ── 8. Construir señal confirmada ─────────────────────────────────────
        tp, sl = self._compute_tp_sl(entry_price, atr, signal_dir)
        trailing = regime.regime in (MarketRegime.BULL, MarketRegime.BEAR)

        # signal_score = 30% motif_edge + 40% tin_score + 30% regime_conf
        # 0.5 → neutral en motif/tin, confidence puro del régimen como base
        _base_score   = float(regime.confidence)
        _signal_score = (0.30 * float(motif_edge)
                       + 0.40 * float(tin_score)
                       + 0.30 * _base_score)
        _signal_score = max(0.0, min(1.0, _signal_score))

        signal = TradeSignal(
            symbol         = symbol,
            signal_type    = signal_dir,
            timestamp      = time.time(),
            entry_price    = entry_price,
            take_profit    = tp,
            signal_kind    = (SignalKind.OPEN_LONG if signal_dir == SignalType.BUY
                              else SignalKind.OPEN_SHORT),
            stop_loss      = sl,
            atr            = atr,
            confidence     = _signal_score,   # 30% motif + 40% TIN + 30% regime
            regime         = regime.regime.value,
            strategy       = self._pick_strategy(regime.regime),
            position_size  = position_size,
            iv_rank        = iv.iv_rank_30d,
            cvd_delta      = cvd.delta_imbalance,
            rsi            = tech.rsi_14,
            volume_ratio   = tech.volume_ratio,
            visual_confirmed = visual_ok,
            max_hold_days  = self.MAX_HOLD_DAYS,
            trailing_active = trailing,
        )

        # Metadata — scores de todos los componentes
        signal.metadata["regime_confidence"] = round(_base_score, 4)
        signal.metadata["motif_edge"]        = round(float(motif_edge), 4)
        signal.metadata["tin_score"]         = round(float(tin_score), 4)
        signal.metadata["signal_score"]      = round(_signal_score, 4)
        if pattern_lab_ctx is not None:
            signal.metadata["pattern_score"]   = round(_pattern_score, 4)
            signal.metadata["tin_prob"]        = round(getattr(pattern_lab_ctx, "tin_prob_positive", 0.5), 4)
            signal.metadata["arima_deviation"] = round(getattr(pattern_lab_ctx, "arima_deviation", 0.0), 6)
            signal.metadata["n_motifs"]        = getattr(pattern_lab_ctx, "n_motifs_found", 0)

        _mtf_coh = float(mtf_report.coherence_score) if mtf_report is not None else -1.0
        signal.metadata["mtf_coherence_at_signal"] = round(_mtf_coh, 4) if _mtf_coh >= 0 else None

        logger.info(
            "SEÑAL %s %s | score=%.3f (regime=%.2f motif=%.2f tin=%.2f mtf_coh=%.2f) | TP=%.2f SL=%.2f | OCR=%s",
            signal_dir.value, symbol, _signal_score, _base_score, motif_edge, tin_score,
            _mtf_coh if _mtf_coh >= 0 else 0.0,
            tp, sl, "OK" if visual_ok else "NO",
        )

        # Registrar posición abierta
        self._positions[symbol] = OpenPosition(
            symbol        = symbol,
            side          = signal_dir,
            entry_price   = entry_price,
            current_price = entry_price,
            take_profit   = tp,
            stop_loss     = sl,
            trailing_stop = sl,
            atr           = atr,
            quantity      = position_size,
            opened_at     = time.time(),
            max_hold_days = self.MAX_HOLD_DAYS,
            strategy      = signal.strategy,
            highest_price = entry_price,
            lowest_price  = entry_price,
        )

        self._signal_history.append(signal)

        # ── 9. Scoring de aprendizaje (AtlasLearningBrain) ───────────────────
        if self._learning_brain is not None:
            try:
                from atlas_code_quant.learning.trade_events import SignalContext
                import datetime as _dt
                ctx = SignalContext(
                    symbol=symbol,
                    asset_class=signal.asset_class,
                    side=signal_dir.value,
                    setup_type=signal.strategy or "unknown",
                    regime=signal.regime,
                    timeframe="1m",
                    entry_price=entry_price,
                    stop_loss_price=sl,
                    r_initial=abs(entry_price - sl),
                    rsi=tech.rsi_14,
                    macd_hist=getattr(tech, "macd_hist", 0.0),
                    atr=atr,
                    bb_pct=getattr(tech, "bb_pct", 0.0),
                    volume_ratio=tech.volume_ratio,
                    cvd=cvd.delta_imbalance,
                    iv_rank=iv.iv_rank_30d,
                    iv_hv_ratio=getattr(iv, "iv_hv_ratio", 1.0),
                    capital=current_capital,
                    position_size=float(position_size),
                    signal_time=_dt.datetime.utcnow(),
                )
                score_result = self._learning_brain.score_signal(ctx)
                signal.metadata["learning_score"] = score_result.total_score
                signal.metadata["learning_approved"] = score_result.approved
                signal.metadata["learning_reasoning"] = score_result.reasoning
                signal.metadata["size_multiplier"] = score_result.size_multiplier

                if not score_result.approved:
                    logger.info(
                        "SEÑAL %s %s BLOQUEADA por learning_brain: score=%.3f < threshold=%.2f — %s",
                        signal_dir.value, symbol, score_result.total_score,
                        score_result.threshold_used, score_result.reasoning,
                    )
                    return self._flat(symbol, entry_price, atr, "learning_score_below_threshold",
                                      score_result.total_score)
            except Exception as _lb_exc:
                logger.warning("learning_brain.score_signal error: %s", _lb_exc)

        return signal

    # ── Gestión de salida ─────────────────────────────────────────────────────

    def _check_exit(self, pos: OpenPosition, current_price: float) -> Optional[TradeSignal]:
        """Verifica si la posición abierta debe cerrarse."""
        pos.current_price = current_price
        reason = ""

        if pos.side == SignalType.BUY:
            pos.highest_price = max(pos.highest_price, current_price)

            # Trailing stop para posición long
            if pos.trailing_active:
                new_trail = pos.highest_price - pos.atr * self.SL_ATR_MULT
                pos.trailing_stop = max(pos.trailing_stop, new_trail)

            if current_price >= pos.take_profit:
                reason = "take_profit"
            elif current_price <= pos.trailing_stop:
                reason = "trailing_stop" if current_price < pos.stop_loss else "stop_loss"

        else:  # SELL
            pos.lowest_price = min(pos.lowest_price, current_price)

            if pos.trailing_active:
                new_trail = pos.lowest_price + pos.atr * self.SL_ATR_MULT
                pos.trailing_stop = min(pos.trailing_stop, new_trail)

            if current_price <= pos.take_profit:
                reason = "take_profit"
            elif current_price >= pos.trailing_stop:
                reason = "stop_loss"

        # Time exit
        days_open = (time.time() - pos.opened_at) / 86400
        if days_open >= pos.max_hold_days:
            reason = "time_exit"

        if reason:
            del self._positions[pos.symbol]
            _kind_map = {
                "take_profit":   SignalKind.CLOSE_TP,
                "stop_loss":     SignalKind.CLOSE_SL,
                "trailing_stop": SignalKind.CLOSE_TRAIL,
                "time_exit":     SignalKind.CLOSE_TIME,
            }
            return TradeSignal(
                symbol       = pos.symbol,
                signal_type  = SignalType.EXIT,
                timestamp    = time.time(),
                entry_price  = current_price,
                take_profit  = pos.take_profit,
                stop_loss    = pos.stop_loss,
                atr          = pos.atr,
                confidence   = 1.0,
                exit_reason  = reason,
                strategy     = pos.strategy,
                signal_kind  = _kind_map.get(reason, SignalKind.CLOSE_SL),
            )

        return None

    # ── Dirección técnica ─────────────────────────────────────────────────────

    def _get_technical_direction(self, tech, regime) -> SignalType:
        """Bull/Bear según régimen + confirmación RSI/MACD."""
        from atlas_code_quant.models.regime_classifier import MarketRegime

        is_bull = regime.regime == MarketRegime.BULL
        is_bear = regime.regime == MarketRegime.BEAR
        is_sideways = regime.regime == MarketRegime.SIDEWAYS

        # MACD cruce
        macd_bullish = tech.macd > tech.macd_signal and tech.macd_hist > 0
        macd_bearish = tech.macd < tech.macd_signal and tech.macd_hist < 0

        # RSI divergencia (oversold en bull, overbought en bear, extremos en sideways)
        rsi_bull = tech.rsi_14 < self.RSI_OVERSOLD + 15   # RSI saliendo de oversold
        rsi_bear = tech.rsi_14 > self.RSI_OVERBOUGHT - 15

        if is_bull and macd_bullish and rsi_bull:
            return SignalType.BUY
        if is_bear and macd_bearish and rsi_bear:
            return SignalType.SELL
        if is_sideways:
            # Mean reversion: comprar oversold, vender overbought
            if tech.rsi_14 < self.RSI_OVERSOLD and macd_bullish:
                return SignalType.BUY
            if tech.rsi_14 > self.RSI_OVERBOUGHT and macd_bearish:
                return SignalType.SELL

        return SignalType.FLAT

    # ── Utilidades ────────────────────────────────────────────────────────────

    def _compute_tp_sl(
        self, price: float, atr: float, direction: SignalType
    ) -> tuple[float, float]:
        if direction == SignalType.BUY:
            tp = price + atr * self.TP_ATR_MULT
            sl = price - atr * self.SL_ATR_MULT
        else:
            tp = price - atr * self.TP_ATR_MULT
            sl = price + atr * self.SL_ATR_MULT
        return round(tp, 2), round(sl, 2)

    def _pick_strategy(self, regime) -> str:
        from atlas_code_quant.models.regime_classifier import MarketRegime
        map_ = {
            MarketRegime.BULL:     "trend_following",
            MarketRegime.BEAR:     "trend_following_short",
            MarketRegime.SIDEWAYS: "mean_reversion",
        }
        return map_.get(regime, "unknown")

    def _check_visual(self, api_price: float, ocr_price: Optional[float]) -> bool:
        """Verifica que el precio OCR coincide con el precio API (±0.5%)."""
        if ocr_price is None or ocr_price <= 0:
            return False
        diff = abs(api_price - ocr_price) / api_price
        return diff <= self.VISUAL_PRICE_TOLERANCE

    def _compute_zscore(self, value: float) -> float:
        if len(self._cvd_history) < 10:
            return 0.0
        import numpy as np
        arr = self._cvd_history[-100:]
        mean = float(sum(arr) / len(arr))
        std  = float((sum((x - mean) ** 2 for x in arr) / len(arr)) ** 0.5)
        return (value - mean) / std if std > 1e-10 else 0.0

    def _flat(self, symbol: str, price: float, atr: float,
              reason: str, value: float) -> TradeSignal:
        logger.debug("FLAT %s — %s (%.3f)", symbol, reason, value)
        return TradeSignal(
            symbol      = symbol,
            signal_type = SignalType.FLAT,
            timestamp   = time.time(),
            entry_price = price,
            take_profit = price + atr * self.TP_ATR_MULT,
            stop_loss   = price - atr * self.SL_ATR_MULT,
            atr         = atr,
            confidence  = 0.0,
            exit_reason = reason,
        )

    def open_positions(self) -> dict[str, OpenPosition]:
        return dict(self._positions)

    def signal_history(self, last_n: int = 50) -> list[TradeSignal]:
        return self._signal_history[-last_n:]
