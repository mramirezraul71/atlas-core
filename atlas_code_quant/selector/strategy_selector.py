from __future__ import annotations

import math
from datetime import datetime
from typing import Any

from backtesting.winning_probability import (
    StrategyLeg,
    _capital_at_risk,
    _strategy_payoff,
    get_winning_probability,
)
from learning.adaptive_policy import AdaptiveLearningService
from monitoring.strategy_tracker import StrategyTracker
from operations.strategy_playbooks import build_strategy_playbook


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if math.isnan(result) or math.isinf(result):
        return default
    return result


def _clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _strategy_meta(strategy_type: str) -> dict[str, str]:
    if strategy_type == "equity_long":
        return {
            "asset_class": "equity",
            "side": "buy",
            "position_effect": "open",
            "order_type": "market",
            "label": "compra de accion",
            "family": "equity",
        }
    if strategy_type == "equity_short":
        return {
            "asset_class": "equity",
            "side": "sell_short",
            "position_effect": "open",
            "order_type": "market",
            "label": "venta en corto",
            "family": "equity",
        }
    if strategy_type in {"long_call", "long_put"}:
        return {
            "asset_class": "option",
            "side": "buy",
            "position_effect": "open",
            "order_type": "market",
            "label": "opcion simple",
            "family": "simple",
        }
    if strategy_type in {"bull_call_debit_spread", "bear_put_debit_spread"}:
        return {
            "asset_class": "multileg",
            "side": "buy",
            "position_effect": "open",
            "order_type": "debit",
            "label": "spread de debito",
            "family": "debit",
        }
    if strategy_type in {"bull_put_credit_spread", "bear_call_credit_spread"}:
        return {
            "asset_class": "multileg",
            "side": "sell",
            "position_effect": "open",
            "order_type": "credit",
            "label": "spread de credito",
            "family": "credit",
        }
    return {
        "asset_class": "multileg",
        "side": "buy",
        "position_effect": "open",
        "order_type": "debit",
        "label": strategy_type,
        "family": "options",
    }


def _timeframe_profile(timeframe: str) -> dict[str, float]:
    mapping = {
        "5m": {"risk_pct": 0.35, "equity_stop_pct": 0.006, "camera_fit": 58.0},
        "15m": {"risk_pct": 0.50, "equity_stop_pct": 0.008, "camera_fit": 68.0},
        "1h": {"risk_pct": 0.75, "equity_stop_pct": 0.011, "camera_fit": 80.0},
        "4h": {"risk_pct": 1.00, "equity_stop_pct": 0.015, "camera_fit": 88.0},
        "1d": {"risk_pct": 1.20, "equity_stop_pct": 0.020, "camera_fit": 92.0},
    }
    return mapping.get(str(timeframe or "").lower(), {"risk_pct": 0.75, "equity_stop_pct": 0.010, "camera_fit": 72.0})


def _chart_interval(timeframe: str) -> str:
    return {
        "5m": "5",
        "15m": "15",
        "1h": "60",
        "4h": "240",
        "1d": "D",
    }.get(str(timeframe or "").lower(), "60")


def _chart_plan(symbol: str, timeframe: str, higher_timeframe: str | None, chart_provider: str) -> dict[str, Any]:
    provider = str(chart_provider or "tradingview").lower()
    base_symbol = str(symbol or "").upper()
    primary_interval = _chart_interval(timeframe)
    higher_interval = _chart_interval(higher_timeframe or "1h")
    overview_interval = "D"
    if provider == "yahoo":
        primary_url = f"https://finance.yahoo.com/quote/{base_symbol}/chart"
        higher_url = primary_url
        overview_url = primary_url
    else:
        primary_url = f"https://www.tradingview.com/chart/?symbol=NASDAQ%3A{base_symbol}&interval={primary_interval}"
        higher_url = f"https://www.tradingview.com/chart/?symbol=NASDAQ%3A{base_symbol}&interval={higher_interval}"
        overview_url = f"https://www.tradingview.com/chart/?symbol=NASDAQ%3A{base_symbol}&interval={overview_interval}"
    return {
        "provider": provider,
        "auto_open_supported": True,
        "targets": [
            {
                "title": f"{base_symbol} disparo {timeframe}",
                "timeframe": timeframe,
                "url": primary_url,
            },
            {
                "title": f"{base_symbol} confirmacion {higher_timeframe or '1h'}",
                "timeframe": higher_timeframe or "1h",
                "url": higher_url,
            },
            {
                "title": f"{base_symbol} contexto diario",
                "timeframe": "1d",
                "url": overview_url,
            },
        ],
        "steps": [
            "abrir grafico principal del setup",
            "abrir grafico de confirmacion superior",
            "abrir grafico de contexto diario",
            "centrar la zona de entrada en pantalla",
            "validar con la camara antes del ticket",
        ],
    }


def _expected_visual_signature(candidate: dict[str, Any], strategy_type: str) -> dict[str, Any]:
    direction = str(candidate.get("direction") or "").lower()
    timeframe = str(candidate.get("timeframe") or "1h")
    higher_timeframe = str((candidate.get("confirmation") or {}).get("higher_timeframe") or "1h")
    bullish = direction == "alcista" or strategy_type in {"equity_long", "long_call", "bull_call_debit_spread", "bull_put_credit_spread"}
    expected_chart_bias = "bullish" if bullish else "bearish"
    expected_patterns = (
        ["breakout", "uptrend", "bounce", "pullback", "flag", "bandera", "ruptura", "alcista"]
        if bullish
        else ["breakdown", "downtrend", "reversal", "dump", "selloff", "bajista", "resistance", "bear"]
    )
    return {
        "symbol": str(candidate.get("symbol") or "").upper(),
        "direction": direction or ("alcista" if bullish else "bajista"),
        "timeframe": timeframe,
        "higher_timeframe": higher_timeframe,
        "expected_chart_bias": expected_chart_bias,
        "expected_patterns": expected_patterns,
    }


def _camera_validation_profile(*, timeframe: str, strategy_type: str, family: str) -> dict[str, Any]:
    tf = str(timeframe or "1h").lower()
    base_ocr = {
        "5m": 84.0,
        "15m": 78.0,
        "1h": 72.0,
        "4h": 68.0,
        "1d": 65.0,
    }.get(tf, 72.0)
    base_alignment = {
        "5m": 80.0,
        "15m": 76.0,
        "1h": 72.0,
        "4h": 70.0,
        "1d": 68.0,
    }.get(tf, 72.0)

    if family == "equity":
        return {
            "validation_mode": "structure_timing",
            "min_ocr_confidence_pct": base_ocr,
            "min_alignment_score_pct": base_alignment,
            "require_pattern_confirmation": tf in {"5m", "15m", "1h"},
            "require_price_evidence": tf in {"5m", "15m"},
        }
    if family == "simple":
        return {
            "validation_mode": "directional_option_confirmation",
            "min_ocr_confidence_pct": max(base_ocr, 78.0),
            "min_alignment_score_pct": max(base_alignment, 76.0),
            "require_pattern_confirmation": True,
            "require_price_evidence": True,
        }
    if family in {"debit", "credit", "options"}:
        return {
            "validation_mode": "defined_risk_structure_confirmation",
            "min_ocr_confidence_pct": max(base_ocr, 84.0),
            "min_alignment_score_pct": max(base_alignment, 80.0),
            "require_pattern_confirmation": True,
            "require_price_evidence": True,
        }
    return {
        "validation_mode": "generic_visual_confirmation",
        "min_ocr_confidence_pct": base_ocr,
        "min_alignment_score_pct": base_alignment,
        "require_pattern_confirmation": True,
        "require_price_evidence": False,
    }


class StrategySelectorService:
    def __init__(self, tracker: StrategyTracker, learning: AdaptiveLearningService | None = None):
        self.tracker = tracker
        self.learning = learning or AdaptiveLearningService()

    def _legs_from_probability(self, probability_payload: dict[str, Any] | None) -> list[StrategyLeg]:
        if not probability_payload:
            return []
        return [
            StrategyLeg(
                side=str(leg.get("side") or "long"),  # type: ignore[arg-type]
                option_type=str(leg.get("option_type") or "call"),  # type: ignore[arg-type]
                strike=_safe_float(leg.get("strike"), 0.0),
                premium_mid=_safe_float(leg.get("premium_mid"), 0.0),
                expiration=leg.get("expiration"),
                dte=int(leg.get("dte")) if leg.get("dte") is not None else None,
                symbol=str(leg.get("symbol") or ""),
                bid=_safe_float(leg.get("bid"), 0.0),
                ask=_safe_float(leg.get("ask"), 0.0),
                volume=_safe_float(leg.get("volume"), 0.0),
                open_interest=_safe_float(leg.get("open_interest"), 0.0),
                implied_volatility=_safe_float(leg.get("implied_volatility"), 0.0),
            )
            for leg in (probability_payload.get("selected_legs") or [])
        ]

    def _break_even_points(self, prices: list[float], pnls: list[float]) -> list[float]:
        points: list[float] = []
        for idx in range(1, min(len(prices), len(pnls))):
            prev_price = prices[idx - 1]
            curr_price = prices[idx]
            prev_pnl = pnls[idx - 1]
            curr_pnl = pnls[idx]
            if abs(prev_pnl) <= 1e-9:
                points.append(round(prev_price, 2))
            if prev_pnl * curr_pnl < 0:
                span = curr_price - prev_price
                delta = curr_pnl - prev_pnl
                crossing = curr_price if abs(delta) <= 1e-9 else prev_price + ((0.0 - prev_pnl) / delta) * span
                points.append(round(crossing, 2))
        deduped: list[float] = []
        for value in sorted(points):
            if not deduped or abs(value - deduped[-1]) > 0.1:
                deduped.append(value)
        return deduped

    def _risk_profile(
        self,
        *,
        candidate: dict[str, Any],
        selected: dict[str, Any],
        size_plan: dict[str, Any],
        probability_payload: dict[str, Any] | None,
    ) -> dict[str, Any]:
        strategy_type = str(selected.get("strategy_type") or "")
        meta = _strategy_meta(strategy_type)
        spot = _safe_float(
            (probability_payload or {}).get("market_snapshot", {}).get("spot"),
            _safe_float(candidate.get("price"), 0.0),
        )
        size = max(int(_safe_float(size_plan.get("suggested_size"), 1.0)), 1)
        if spot <= 0:
            return {}

        if meta["family"] == "equity":
            per_unit_risk = max(_safe_float(size_plan.get("per_unit_risk_usd"), 0.0), spot * 0.01)
            direction_mult = 1.0 if strategy_type == "equity_long" else -1.0
            prices = [max(0.01, spot + move * per_unit_risk) for move in (-2, -1, 0, 1, 2)]
            pnls = [round((price - spot) * size * direction_mult, 2) for price in prices]
            predicted_move_pct = max(abs(_safe_float(candidate.get("predicted_move_pct"), 0.0)), 1.0)
            target_distance = max((spot * predicted_move_pct) / 100.0, per_unit_risk)
            target_price = max(0.01, spot + (target_distance if direction_mult > 0 else -target_distance))
            estimated_target_pnl = round(abs(target_price - spot) * size, 2)
            max_loss = round(per_unit_risk * size, 2)
            reward_to_risk = round(estimated_target_pnl / max(max_loss, 1e-6), 2)
            return {
                "mode": "equity_estimated",
                "spot": round(spot, 2),
                "entry_reference_price": round(spot, 2),
                "stop_reference_price": round(max(0.01, spot - (per_unit_risk * direction_mult)), 2),
                "target_reference_price": round(target_price, 2),
                "break_even_points": [round(spot, 2)],
                "max_loss_usd": max_loss,
                "max_profit_usd": None,
                "max_profit_open_ended": True,
                "estimated_target_profit_usd": estimated_target_pnl,
                "reward_to_risk": reward_to_risk,
                "scenarios": [
                    {"label": label, "price": round(price, 2), "pnl_usd": pnl}
                    for label, price, pnl in zip(
                        ["caida fuerte", "retroceso", "base", "avance", "avance fuerte"],
                        prices,
                        pnls,
                    )
                ],
                "notes": [
                    "perfil estimado por riesgo por accion y objetivo previsto",
                    "la ganancia maxima sigue abierta mientras el precio continue a favor",
                ],
            }

        legs = self._legs_from_probability(probability_payload)
        if not legs:
            return {
                "mode": "unavailable",
                "spot": round(spot, 2),
                "notes": ["no se pudo construir el perfil porque faltan patas de opciones"],
            }

        strike_values = [leg.strike for leg in legs if leg.strike > 0]
        lo_anchor = min([spot, *strike_values]) if strike_values else spot
        hi_anchor = max([spot, *strike_values]) if strike_values else spot
        lo = max(0.01, lo_anchor * 0.82)
        hi = max(lo + 0.5, hi_anchor * 1.18)
        grid = [lo + ((hi - lo) * idx / 60.0) for idx in range(61)]
        pnls = [round(_strategy_payoff(strategy_type, price, legs, spot) * 100.0 * size, 2) for price in grid]
        break_even_points = self._break_even_points(grid, pnls)
        max_profit = round(max(pnls), 2) if pnls else 0.0
        max_loss = round(abs(min(pnls)), 2) if pnls else 0.0
        reward_to_risk = round(max_profit / max(max_loss, 1e-6), 2) if max_profit > 0 else None
        scenario_prices = [max(0.01, spot * (1.0 + move)) for move in (-0.10, -0.05, 0.0, 0.05, 0.10)]
        scenario_pnls = [
            round(_strategy_payoff(strategy_type, price, legs, spot) * 100.0 * size, 2)
            for price in scenario_prices
        ]
        return {
            "mode": "options_payoff",
            "spot": round(spot, 2),
            "entry_reference_price": round(spot, 2),
            "break_even_points": break_even_points,
            "max_loss_usd": max_loss,
            "max_profit_usd": max_profit,
            "max_profit_open_ended": False,
            "estimated_target_profit_usd": round(max(scenario_pnls), 2) if scenario_pnls else 0.0,
            "reward_to_risk": reward_to_risk,
            "capital_at_risk_usd": round(_capital_at_risk(strategy_type, legs, spot) * 100.0 * size, 2),
            "scenarios": [
                {"label": label, "price": round(price, 2), "pnl_usd": pnl}
                for label, price, pnl in zip(
                    ["caida fuerte", "retroceso", "base", "avance", "avance fuerte"],
                    scenario_prices,
                    scenario_pnls,
                )
            ],
            "notes": [
                "perfil derivado del payoff de la estructura y sus patas",
                "los break-even se estiman por cruce de pnl sobre una malla de precios",
            ],
        }

    def _candidate_structures(
        self,
        candidate: dict[str, Any],
        *,
        account_scope: str,
        allow_equity: bool,
        allow_credit: bool,
        prefer_defined_risk: bool,
    ) -> list[dict[str, Any]]:
        direction = str(candidate.get("direction") or "").lower()
        bullish = direction == "alcista"
        timeframe = str(candidate.get("timeframe") or "1h")
        profile = _timeframe_profile(timeframe)
        selection_score = _safe_float(candidate.get("selection_score"), 0.0)
        local_win = _safe_float(candidate.get("local_win_rate_pct"), 0.0)
        predicted_move = _safe_float(candidate.get("predicted_move_pct"), 0.0)
        confirmed = str((candidate.get("confirmation") or {}).get("direction") or "").lower() == direction
        strategy_key = str(candidate.get("strategy_key") or "")
        symbol = str(candidate.get("symbol") or "").upper()
        order_flow = candidate.get("order_flow") or {}
        order_flow_score = _safe_float(order_flow.get("score_pct"), 50.0)
        order_flow_direction = str(order_flow.get("direction") or "neutral").lower()
        order_flow_confidence = _safe_float(order_flow.get("confidence_pct"), 0.0)

        learning_context = self.learning.context(
            symbol=symbol,
            direction=direction,
            account_scope=account_scope,
        )
        symbol_bias = _safe_float(learning_context.get("symbol_bias"), 0.0)
        directional_bias = _safe_float(learning_context.get("directional_symbol_bias"), 0.0)

        options: list[dict[str, Any]] = []
        base_signal = (selection_score * 0.55) + (local_win * 0.45)
        flow_aligned = order_flow_direction == direction and order_flow_direction in {"alcista", "bajista"}
        flow_opposed = order_flow_direction in {"alcista", "bajista"} and order_flow_direction != direction

        def _adaptive_bias_for(strategy_type: str) -> float:
            strategy_bias = self.learning.strategy_bias(strategy_type, account_scope=account_scope)
            return round(symbol_bias + directional_bias + strategy_bias, 2)

        def _adaptive_note(adaptive_bias: float, positive: str, negative: str) -> list[str]:
            if adaptive_bias >= 1.0:
                return [positive]
            if adaptive_bias <= -1.0:
                return [negative]
            return []

        if allow_equity:
            equity_bonus = 0.0
            camera_precision = profile["camera_fit"] + 8.0
            if timeframe in {"1h", "4h", "1d"}:
                equity_bonus += 10.0
            if predicted_move <= 3.5:
                equity_bonus += 6.0
            if confirmed:
                equity_bonus += 8.0
            if profile["camera_fit"] >= 80:
                equity_bonus += 8.0
            if flow_aligned:
                equity_bonus += min(8.0 + (order_flow_confidence * 0.04), 12.0)
            elif flow_opposed:
                equity_bonus -= min(10.0 + (order_flow_confidence * 0.04), 14.0)
            strategy_type = "equity_long" if bullish else "equity_short"
            adaptive_bias = _adaptive_bias_for(strategy_type)
            options.append({
                "strategy_type": strategy_type,
                "vehicle_type": "equity",
                "score": round(base_signal + equity_bonus + adaptive_bias, 2),
                "camera_precision_pct": round(camera_precision, 1),
                "why": [
                    "estructura visualmente simple para validar con camara",
                    "mejor lectura de precio en temporalidad media/lenta",
                    "permite control directo del tamano por efectivo",
                    *(["order flow intradia acompaña la direccion principal"] if flow_aligned else []),
                    *(["order flow intradia contradice y exige mas disciplina"] if flow_opposed else []),
                    *_adaptive_note(
                        adaptive_bias,
                        "el diario reciente favorece esta estructura en este activo",
                        "el diario reciente enfria esta estructura y reduce conviccion",
                    ),
                ],
                "adaptive_bias": adaptive_bias,
            })

        simple_bonus = 6.0 if timeframe in {"5m", "15m"} or strategy_key == "ml_directional" else 2.0
        simple_bonus += 5.0 if predicted_move >= 2.0 else 0.0
        simple_bonus += 2.0 if confirmed else -4.0
        if flow_aligned:
            simple_bonus += min(6.0 + (order_flow_confidence * 0.03), 10.0)
        elif flow_opposed:
            simple_bonus -= min(8.0 + (order_flow_confidence * 0.03), 12.0)
        strategy_type = "long_call" if bullish else "long_put"
        adaptive_bias = _adaptive_bias_for(strategy_type)
        options.append({
            "strategy_type": strategy_type,
            "vehicle_type": "simple_option",
            "score": round(base_signal + simple_bonus + adaptive_bias, 2),
            "camera_precision_pct": round(profile["camera_fit"] - (8.0 if timeframe == "5m" else 4.0), 1),
            "why": [
                "captura desplazamiento con lectura direccional clara",
                "util cuando el setup es rapido o tipo breakout",
                *(["order flow acompaña la aceleracion del setup"] if flow_aligned else []),
                *(["order flow no acompaña; reducir conviccion"] if flow_opposed else []),
                *_adaptive_note(
                    adaptive_bias,
                    "aprendizaje reciente suma conviccion sobre esta opcion simple",
                    "aprendizaje reciente recomienda mas prudencia en esta opcion simple",
                ),
            ],
            "adaptive_bias": adaptive_bias,
        })

        debit_bonus = 8.0 if timeframe in {"1h", "4h"} else 3.0
        debit_bonus += 5.0 if confirmed else 0.0
        debit_bonus += 4.0 if 1.2 <= predicted_move <= 3.8 else -1.0
        debit_bonus += 3.0 if prefer_defined_risk else 0.0
        if flow_aligned:
            debit_bonus += min(5.0 + (order_flow_confidence * 0.025), 8.5)
        elif flow_opposed:
            debit_bonus -= min(7.0 + (order_flow_confidence * 0.03), 10.0)
        strategy_type = "bull_call_debit_spread" if bullish else "bear_put_debit_spread"
        adaptive_bias = _adaptive_bias_for(strategy_type)
        options.append({
            "strategy_type": strategy_type,
            "vehicle_type": "defined_risk_debit",
            "score": round(base_signal + debit_bonus + adaptive_bias, 2),
            "camera_precision_pct": round(profile["camera_fit"] + 2.0, 1),
            "why": [
                "riesgo totalmente definido",
                "buena relacion precision/costo para entradas direccionales",
                "mejor equilibrio para paper antes de ir a live",
                *(["order flow confirma la presion a favor"] if flow_aligned else []),
                *(["order flow contradice; spread solo con mas confirmacion"] if flow_opposed else []),
                *_adaptive_note(
                    adaptive_bias,
                    "el diario reciente favorece esta estructura definida",
                    "el diario reciente recomienda reducir agresividad en esta estructura",
                ),
            ],
            "adaptive_bias": adaptive_bias,
        })

        if allow_credit:
            credit_bonus = 10.0 if timeframe in {"4h", "1d"} else 2.0
            credit_bonus += 7.0 if confirmed else -10.0
            credit_bonus += 6.0 if predicted_move <= 2.8 else -8.0
            credit_bonus += 4.0 if prefer_defined_risk else 0.0
            if flow_aligned:
                credit_bonus += min(4.0 + (order_flow_confidence * 0.02), 7.0)
            elif flow_opposed:
                credit_bonus -= min(9.0 + (order_flow_confidence * 0.035), 12.0)
            strategy_type = "bull_put_credit_spread" if bullish else "bear_call_credit_spread"
            adaptive_bias = _adaptive_bias_for(strategy_type)
            options.append({
                "strategy_type": strategy_type,
                "vehicle_type": "defined_risk_credit",
                "score": round(base_signal + credit_bonus + adaptive_bias, 2),
                "camera_precision_pct": round(profile["camera_fit"] - 2.0, 1),
                "why": [
                    "vende prima con riesgo definido",
                    "mejor cuando la confirmacion superior es fuerte y el desplazamiento esperado es moderado",
                    "adecuado para cuentas con buen buying power",
                    *(["order flow sostiene la tesis de continuidad"] if flow_aligned else []),
                    *(["order flow no acompaña; el credito pierde calidad"] if flow_opposed else []),
                    *_adaptive_note(
                        adaptive_bias,
                        "el diario reciente favorece esta estructura de credito",
                        "el diario reciente enfria esta estructura de credito",
                    ),
                ],
                "adaptive_bias": adaptive_bias,
            })

        return sorted(options, key=lambda item: float(item.get("score") or 0.0), reverse=True)

    def _option_risk_per_unit(self, strategy_type: str, probability_payload: dict[str, Any] | None) -> float | None:
        if not probability_payload:
            return None
        legs = [
            StrategyLeg(
                side=str(leg.get("side") or "long"),  # type: ignore[arg-type]
                option_type=str(leg.get("option_type") or "call"),  # type: ignore[arg-type]
                strike=_safe_float(leg.get("strike"), 0.0),
                premium_mid=_safe_float(leg.get("premium_mid"), 0.0),
                expiration=leg.get("expiration"),
                dte=int(leg.get("dte")) if leg.get("dte") is not None else None,
                symbol=str(leg.get("symbol") or ""),
                bid=_safe_float(leg.get("bid"), 0.0),
                ask=_safe_float(leg.get("ask"), 0.0),
                volume=_safe_float(leg.get("volume"), 0.0),
                open_interest=_safe_float(leg.get("open_interest"), 0.0),
                implied_volatility=_safe_float(leg.get("implied_volatility"), 0.0),
            )
            for leg in (probability_payload.get("selected_legs") or [])
        ]
        spot = _safe_float((probability_payload.get("market_snapshot") or {}).get("spot"), 0.0)
        if not legs or spot <= 0:
            return None
        return round(_capital_at_risk(strategy_type, legs, spot) * 100.0, 4)

    def _size_plan(
        self,
        *,
        candidate: dict[str, Any],
        selected: dict[str, Any],
        balances: dict[str, Any],
        account_scope: str,
        risk_budget_pct: float,
        probability_payload: dict[str, Any] | None,
    ) -> dict[str, Any]:
        timeframe = str(candidate.get("timeframe") or "1h")
        profile = _timeframe_profile(timeframe)
        price = _safe_float(candidate.get("price"), 0.0)
        cash = _safe_float(balances.get("cash"), 0.0)
        option_bp = _safe_float(balances.get("option_buying_power"), 0.0)
        meta = _strategy_meta(str(selected["strategy_type"]))
        capital_base = cash if meta["family"] == "equity" else max(option_bp, cash)
        adaptive_multiplier = self.learning.risk_multiplier(account_scope=account_scope)
        requested_risk_pct = _clip(risk_budget_pct * adaptive_multiplier, 0.25, 2.0)
        effective_risk_pct = min(requested_risk_pct, profile["risk_pct"])
        risk_budget_usd = round(capital_base * (effective_risk_pct / 100.0), 2)

        if meta["family"] == "equity":
            stop_pct = profile["equity_stop_pct"]
            per_share_risk = max(price * stop_pct, 0.01)
            shares_by_risk = math.floor(risk_budget_usd / per_share_risk) if per_share_risk > 0 else 1
            max_notional_pct = 0.05 if timeframe == "5m" else 0.08 if timeframe in {"15m", "1h"} else 0.12
            max_cash_shares = math.floor((cash * max_notional_pct) / max(price, 0.01)) if cash > 0 else 1
            size = max(1, min(shares_by_risk or 1, max_cash_shares or 1, 500))
            return {
                "adaptive_risk_multiplier": adaptive_multiplier,
                "requested_risk_pct": round(requested_risk_pct, 3),
                "risk_budget_pct": effective_risk_pct,
                "risk_budget_usd": risk_budget_usd,
                "capital_base": round(capital_base, 2),
                "capital_source": "cash",
                "per_unit_risk_usd": round(per_share_risk, 4),
                "suggested_size": size,
                "estimated_notional_usd": round(size * price, 2),
                "notes": "tamano por riesgo por accion y tope de efectivo disponible",
            }

        per_contract_risk = self._option_risk_per_unit(str(selected["strategy_type"]), probability_payload)
        if per_contract_risk is None or per_contract_risk <= 0:
            per_contract_risk = max(abs(_safe_float((probability_payload or {}).get("net_premium"), 0.0)) * 100.0, 150.0)
        contracts = max(1, min(math.floor(risk_budget_usd / per_contract_risk) if per_contract_risk > 0 else 1, 12))
        return {
            "adaptive_risk_multiplier": adaptive_multiplier,
            "requested_risk_pct": round(requested_risk_pct, 3),
            "risk_budget_pct": effective_risk_pct,
            "risk_budget_usd": risk_budget_usd,
            "capital_base": round(capital_base, 2),
            "capital_source": "option_buying_power" if option_bp > 0 else "cash",
            "per_unit_risk_usd": round(per_contract_risk, 4),
            "suggested_size": contracts,
            "estimated_notional_usd": round(contracts * per_contract_risk, 2),
            "notes": "tamano por riesgo maximo de la estructura seleccionada",
        }

    def _entry_plan(
        self,
        *,
        candidate: dict[str, Any],
        selected: dict[str, Any],
        probability_payload: dict[str, Any] | None,
    ) -> dict[str, Any]:
        timeframe = str(candidate.get("timeframe") or "1h")
        direction = str(candidate.get("direction") or "").lower()
        higher_tf = str((candidate.get("confirmation") or {}).get("higher_timeframe") or "1h")
        strategy_type = str(selected.get("strategy_type") or "")
        reference_price = _safe_float(
            candidate.get("price"),
            _safe_float((probability_payload or {}).get("market_snapshot", {}).get("spot"), 0.0),
        )
        predicted_move = _safe_float(candidate.get("predicted_move_pct"), 0.0)
        confidence = _safe_float((probability_payload or {}).get("win_rate_pct"), _safe_float(candidate.get("local_win_rate_pct"), 0.0))
        max_adverse_drift_pct = {
            "5m": 0.20,
            "15m": 0.25,
            "1h": 0.35,
            "4h": 0.50,
            "1d": 0.75,
        }.get(timeframe.lower(), 0.35)
        max_spread_pct = {
            "5m": 0.15,
            "15m": 0.18,
            "1h": 0.25,
            "4h": 0.35,
            "1d": 0.45,
        }.get(timeframe.lower(), 0.25)
        if strategy_type in {"equity_long", "equity_short"}:
            entry_style = "ruptura confirmada en cierre" if timeframe in {"5m", "15m"} else "entrada en continuidad con confirmacion superior"
            trigger = "rompimiento del maximo/minimo de la vela señal" if timeframe in {"5m", "15m"} else "continuacion tras mantener soporte/resistencia"
            dte = "no aplica"
        elif strategy_type in {"long_call", "long_put"}:
            entry_style = "entrada puntual para capturar desplazamiento"
            trigger = "solo si el activo mantiene direccion y volumen tras abrir el grafico"
            dte = "14 a 30 dias"
        elif strategy_type in {"bull_call_debit_spread", "bear_put_debit_spread"}:
            entry_style = "spread de debito con confirmacion antes del ticket"
            trigger = "tomar cuando el riesgo definido mejora la relacion costo/precision"
            dte = "21 a 45 dias"
        else:
            entry_style = "spread de credito solo con lectura estable"
            trigger = "solo si la zona corta queda fuera del desplazamiento esperado"
            dte = "30 a 45 dias"
        return {
            "entry_style": entry_style,
            "trigger_rule": trigger,
            "confirmation_rule": f"la direccion en {higher_tf} debe seguir {direction or 'coherente'}",
            "chart_focus": "vela señal, zona de ruptura, volumen y confirmacion superior",
            "dte_window": dte,
            "entry_reference_price": round(reference_price, 4) if reference_price > 0 else None,
            "expected_move_pct": round(predicted_move, 2),
            "confidence_reference_pct": round(confidence, 2),
            "max_adverse_drift_pct": round(max_adverse_drift_pct, 2),
            "max_spread_pct": round(max_spread_pct, 2),
        }

    def _confidence_breakdown(
        self,
        *,
        candidate: dict[str, Any],
        selected: dict[str, Any],
        probability_payload: dict[str, Any] | None,
        camera_visual_fit_pct: float,
        adaptive_context: dict[str, Any],
    ) -> dict[str, Any]:
        local_win = _safe_float(candidate.get("local_win_rate_pct"), 0.0)
        selection_score = _safe_float(candidate.get("selection_score"), 0.0)
        relative_strength = _safe_float(candidate.get("relative_strength_pct"), 0.0)
        probability_win = _safe_float((probability_payload or {}).get("win_rate_pct"), local_win)
        confirmation_conf = _safe_float((candidate.get("confirmation") or {}).get("confidence_pct"), 0.0)
        order_flow = candidate.get("order_flow") or {}
        return {
            "local_win_rate_pct": round(local_win, 2),
            "selector_score_pct": round(selection_score, 2),
            "relative_strength_pct": round(relative_strength, 2),
            "probability_win_rate_pct": round(probability_win, 2),
            "confirmation_confidence_pct": round(confirmation_conf, 2),
            "order_flow_score_pct": round(_safe_float(order_flow.get("score_pct"), 50.0), 2),
            "order_flow_confidence_pct": round(_safe_float(order_flow.get("confidence_pct"), 0.0), 2),
            "camera_visual_fit_pct": round(camera_visual_fit_pct, 2),
            "final_structure_score": round(_safe_float(selected.get("score"), 0.0), 2),
            "adaptive_bias_pct": round(_safe_float(adaptive_context.get("total_bias"), 0.0), 2),
        }

    def _automation_ready(
        self,
        *,
        account_scope: str,
        camera_visual_fit_pct: float,
        selected: dict[str, Any],
    ) -> dict[str, Any]:
        strategy_type = str(selected.get("strategy_type") or "")
        charts_ready = True
        camera_ready = camera_visual_fit_pct >= 70.0
        ticket_ready = True
        operation_preview_ready = True
        paper_submit_ready = account_scope == "paper"
        recommended_autonomous_path = (
            "scanner -> selector -> abrir graficos -> validar con camara -> preview operacional -> ticket"
            if strategy_type in {"equity_long", "equity_short", "bull_call_debit_spread", "bear_put_debit_spread"}
            else "scanner -> selector -> abrir graficos -> validar con camara -> ticket supervisado"
        )
        return {
            "charts_ready": charts_ready,
            "camera_ready": camera_ready,
            "ticket_ready": ticket_ready,
            "operation_preview_ready": operation_preview_ready,
            "paper_submit_ready": paper_submit_ready,
            "recommended_autonomous_path": recommended_autonomous_path,
        }

    def _exit_plan(self, strategy_type: str, timeframe: str) -> dict[str, Any]:
        if strategy_type in {"equity_long", "equity_short"}:
            return {
                "primary_take_profit": "tomar parcial en 1R y gestionar resto con trailing",
                "risk_invalidation": "cerrar si invalida estructura en timeframe principal",
                "time_stop": f"salir si {timeframe} pierde confirmacion superior",
            }
        if strategy_type in {"long_call", "long_put"}:
            return {
                "primary_take_profit": "salida parcial entre 30% y 45% sobre la prima",
                "risk_invalidation": "stop si la prima cae 20%-25% o se invalida la estructura",
                "time_stop": "cerrar si el movimiento no aparece en 2-3 velas del timeframe principal",
            }
        if strategy_type in {"bull_call_debit_spread", "bear_put_debit_spread"}:
            return {
                "primary_take_profit": "buscar 35%-50% del valor maximo esperable del spread",
                "risk_invalidation": "salir si pierde 25%-35% del debito o falla la confirmacion",
                "time_stop": "si el activo se lateraliza y erosiona el setup, salir temprano",
            }
        return {
            "primary_take_profit": "recomprar entre 40% y 60% del credito maximo",
            "risk_invalidation": "salir si alcanza 1.5x el credito o si se rompe el lado corto",
            "time_stop": "reducir o cerrar si la probabilidad cae de forma sostenida",
        }

    def proposal(
        self,
        *,
        candidate: dict[str, Any],
        account_scope: str = "paper",
        account_id: str | None = None,
        chart_provider: str = "tradingview",
        prefer_defined_risk: bool = True,
        allow_equity: bool = True,
        allow_credit: bool = True,
        risk_budget_pct: float = 0.75,
    ) -> dict[str, Any]:
        summary = self.tracker.build_summary(account_scope=account_scope, account_id=account_id)  # type: ignore[arg-type]
        balances = summary.get("balances") or {}
        account_session = summary.get("account_session") or {}
        candidate_structures = self._candidate_structures(
            candidate,
            account_scope=account_scope,
            allow_equity=allow_equity,
            allow_credit=allow_credit,
            prefer_defined_risk=prefer_defined_risk,
        )
        if not candidate_structures:
            raise ValueError("No se pudo generar una estructura sugerida")

        selected = dict(candidate_structures[0])
        probability_payload = None
        warnings: list[str] = []
        strategy_type = str(selected["strategy_type"])
        if strategy_type not in {"equity_long", "equity_short"}:
            try:
                probability_payload = get_winning_probability(
                    symbol=str(candidate.get("symbol") or ""),
                    strategy_type=strategy_type,  # type: ignore[arg-type]
                    account_scope=account_scope,  # type: ignore[arg-type]
                    account_id=account_id,
                ).to_dict()
                selected["probability_win_rate_pct"] = probability_payload.get("win_rate_pct")
                selected["score"] = round(
                    float(selected["score"]) * 0.7 + _safe_float(probability_payload.get("win_rate_pct"), 0.0) * 0.3,
                    2,
                )
            except Exception as exc:
                warnings.append(f"No se pudo validar la estructura con Monte Carlo: {exc}")

        candidate_structures = sorted(candidate_structures, key=lambda item: float(item.get("score") or 0.0), reverse=True)
        size_plan = self._size_plan(
            candidate=candidate,
            selected=selected,
            balances=balances,
            account_scope=account_scope,
            risk_budget_pct=_clip(risk_budget_pct, 0.25, 2.0),
            probability_payload=probability_payload,
        )
        higher_timeframe = (candidate.get("confirmation") or {}).get("higher_timeframe")
        chart_plan = _chart_plan(str(candidate.get("symbol") or ""), str(candidate.get("timeframe") or "1h"), higher_timeframe, chart_provider)
        meta = _strategy_meta(strategy_type)
        camera_profile = _timeframe_profile(str(candidate.get("timeframe") or "1h"))
        camera_visual_fit_pct = round(camera_profile["camera_fit"] + (8.0 if meta["family"] == "equity" else 4.0 if meta["family"] in {"debit", "credit"} else 0.0), 1)
        expected_visual = _expected_visual_signature(candidate, strategy_type)
        validation_profile = _camera_validation_profile(
            timeframe=str(candidate.get("timeframe") or "1h"),
            strategy_type=strategy_type,
            family=str(meta.get("family") or ""),
        )
        camera_plan = {
            "provider": "direct_nexus",
            "required": True,
            "visual_fit_pct": camera_visual_fit_pct,
            "expected_visual": expected_visual,
            "validation_profile": validation_profile,
            "checkpoints": [
                "abrir el grafico principal y el de confirmacion",
                "centrar la zona de disparo para la camara",
                "capturar contexto antes del ticket",
                "verificar que la estructura de velas siga intacta",
            ],
        }
        entry_plan = self._entry_plan(
            candidate=candidate,
            selected=selected,
            probability_payload=probability_payload,
        )
        adaptive_context = self.learning.context(
            symbol=str(candidate.get("symbol") or "").upper(),
            direction=str(candidate.get("direction") or "").lower(),
            strategy_type=strategy_type,
            account_scope=account_scope,
        )
        confidence_breakdown = self._confidence_breakdown(
            candidate=candidate,
            selected=selected,
            probability_payload=probability_payload,
            camera_visual_fit_pct=camera_visual_fit_pct,
            adaptive_context=adaptive_context,
        )
        automation_ready = self._automation_ready(
            account_scope=account_scope,
            camera_visual_fit_pct=camera_visual_fit_pct,
            selected=selected,
        )
        order_seed = {
            "symbol": candidate.get("symbol"),
            "strategy_type": strategy_type,
            "asset_class": meta["asset_class"],
            "side": meta["side"],
            "position_effect": meta["position_effect"],
            "order_type": meta["order_type"],
            "duration": "day",
            "size": size_plan["suggested_size"],
            "account_scope": account_scope,
            "account_id": account_id,
            "tag": f"selector:{candidate.get('strategy_key') or 'idea'}:{candidate.get('timeframe') or 'tf'}",
            "entry_reference_price": entry_plan.get("entry_reference_price"),
            "entry_expected_move_pct": entry_plan.get("expected_move_pct"),
            "entry_confidence_reference_pct": entry_plan.get("confidence_reference_pct"),
            "max_entry_drift_pct": entry_plan.get("max_adverse_drift_pct"),
            "max_entry_spread_pct": entry_plan.get("max_spread_pct"),
            "chart_plan": chart_plan,
            "camera_plan": camera_plan,
        }

        risk_profile = self._risk_profile(
            candidate=candidate,
            selected=selected,
            size_plan=size_plan,
            probability_payload=probability_payload,
        )
        playbook = build_strategy_playbook(
            candidate=candidate,
            selected={**selected, "meta": meta},
            size_plan=size_plan,
            entry_plan=entry_plan,
            exit_plan=self._exit_plan(strategy_type, str(candidate.get("timeframe") or "1h")),
            risk_profile=risk_profile,
            adaptive_context=adaptive_context,
            chart_plan=chart_plan,
            camera_plan=camera_plan,
            account_scope=account_scope,
        )

        return {
            "generated_at": datetime.utcnow().isoformat(),
            "account_scope": account_scope,
            "account_session": account_session,
            "balances": balances,
            "candidate": candidate,
            "selected": {
                **selected,
                "meta": meta,
            },
            "alternatives": candidate_structures[:3],
            "probability": probability_payload,
            "size_plan": size_plan,
            "chart_plan": chart_plan,
            "camera_plan": camera_plan,
            "entry_plan": entry_plan,
            "confidence_breakdown": confidence_breakdown,
            "adaptive_context": adaptive_context,
            "risk_profile": risk_profile,
            "automation_ready": automation_ready,
            "exit_plan": self._exit_plan(strategy_type, str(candidate.get("timeframe") or "1h")),
            "playbook": playbook,
            "order_seed": order_seed,
            "warnings": warnings,
        }
