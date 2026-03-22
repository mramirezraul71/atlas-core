"""Módulo 4B — Validación de Triggers Visuales.

El robot ATLAS valida señales de trading cruzando:
  1. Precio API (Tradier REST/stream)
  2. Precio extraído por OCR desde la cámara Insta360
  3. Color dominante del chart (bullish=verde / bearish=rojo)
  4. Patrón técnico detectado por Qwen2-VL

Si la confirmación visual falla → señal marcada como no confirmada
pero el sistema puede operar igual con menor confianza (configurable).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger("atlas.strategy.visual")


@dataclass
class VisualValidation:
    price_match: bool = False
    color_match: bool = False
    pattern_match: bool = False
    overall_confidence: float = 0.0
    ocr_price: Optional[float] = None
    api_price: float = 0.0
    chart_color: str = ""
    pattern: str = ""
    reason: str = ""


class VisualTriggerValidator:
    """Valida señales cruzando datos OCR con datos de API.

    Uso::

        vtv = VisualTriggerValidator()
        validation = vtv.validate(
            signal_type="BUY",
            api_price=185.50,
            ocr_result=cam.latest_result(),
            expected_pattern="bandera"
        )
        if validation.overall_confidence > 0.7:
            proceed_with_trade()
    """

    PRICE_TOLERANCE   = 0.005    # 0.5%
    COLOR_MAP = {
        "BUY":  ["bullish", "green"],
        "SELL": ["bearish", "red"],
    }

    def validate(
        self,
        signal_type: str,
        api_price: float,
        ocr_result,
        expected_pattern: str = "",
    ) -> VisualValidation:
        v = VisualValidation(api_price=api_price)

        if ocr_result is None:
            v.reason = "no_ocr_result"
            return v

        # Precio más cercano al API
        prices = getattr(ocr_result, "prices", [])
        v.chart_color = getattr(ocr_result, "chart_color", "unknown")
        v.pattern = getattr(ocr_result, "pattern_detected", "")

        # Match de precio
        if prices:
            best = min(prices, key=lambda p: abs(p - api_price))
            v.ocr_price = best
            diff = abs(best - api_price) / api_price if api_price > 0 else 1.0
            v.price_match = diff <= self.PRICE_TOLERANCE

        # Match de color
        expected_colors = self.COLOR_MAP.get(signal_type.upper(), [])
        v.color_match = v.chart_color in expected_colors

        # Match de patrón
        if expected_pattern:
            v.pattern_match = (
                expected_pattern.lower() in v.pattern.lower() or
                v.pattern.lower() in expected_pattern.lower()
            )
        else:
            v.pattern_match = True   # sin restricción de patrón

        # Confianza compuesta
        weights = {"price": 0.6, "color": 0.3, "pattern": 0.1}
        score = (
            weights["price"]   * (1.0 if v.price_match else 0.0) +
            weights["color"]   * (1.0 if v.color_match else 0.0) +
            weights["pattern"] * (1.0 if v.pattern_match else 0.0)
        )
        v.overall_confidence = round(score, 3)

        if v.overall_confidence < 0.6:
            v.reason = (
                f"baja_confirmacion_visual "
                f"(precio={'✓' if v.price_match else '✗'} "
                f"color={'✓' if v.color_match else '✗'} "
                f"patron={'✓' if v.pattern_match else '✗'})"
            )
        else:
            v.reason = "visual_confirmado"

        logger.debug(
            "Visual validation %s → %.2f | precio=%s color=%s patron=%s",
            signal_type, v.overall_confidence,
            "✓" if v.price_match else "✗",
            "✓" if v.color_match else "✗",
            "✓" if v.pattern_match else "✗",
        )
        return v
