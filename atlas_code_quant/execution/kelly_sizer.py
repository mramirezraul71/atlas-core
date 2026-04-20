"""Atlas Code-Quant — Kelly Criterion risk-constrained para position sizing logarítmico.

Implementa el Criterio de Kelly en su variante fraccionada y risk-constrained,
maximizando el crecimiento logarítmico de la riqueza (E[ln(W)]) con control
de drawdown. Recomendación Grok/xAI: PPO + Kelly risk-constrained = máximos retornos.

Referencia: Kelly (1956) — "A New Interpretation of Information Rate"
Variante: Half-Kelly / Quarter-Kelly para reducir volatilidad 25-50%.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Sequence

logger = logging.getLogger("quant.execution.kelly")

# ── Constantes de seguridad ───────────────────────────────────────────────────
_MIN_SAMPLES      = 6       # Mínimo de trades para calcular Kelly válido
_MAX_KELLY_RAW    = 1.0     # Nunca más del 100% del capital en una posición
_DEFAULT_FRACTION = 0.25    # Quarter-Kelly — conservador para cripto/volátil


@dataclass
class KellyResult:
    """Resultado del cálculo de Kelly para un activo/estrategia."""
    symbol: str
    strategy: str
    kelly_full: float           # f* Kelly completo
    kelly_fraction: float       # f* * fracción aplicada (ej: 0.25)
    position_pct: float         # Porcentaje final del capital a invertir
    win_rate: float             # p — probabilidad de ganar
    win_loss_ratio: float       # b — ganancia media / pérdida media (odds)
    samples: int                # Número de trades usados
    confidence: float           # Confianza en el cálculo (sqrt normalizado)
    capped: bool = False        # Si fue limitado por el cap máximo
    note: str = ""


@dataclass
class KellySizer:
    """Calcula el tamaño de posición óptimo usando Kelly Criterion.

    Args:
        fraction: Fracción de Kelly a aplicar (0.25=quarter, 0.5=half).
                  Quarter-Kelly recomendado para cripto/alta volatilidad.
        max_position_pct: Cap máximo de posición (sin importar Kelly).
        min_samples: Mínimo de trades históricos para calcular Kelly.
        atr_sl_multiplier: Multiplicador de ATR para stop-loss dinámico.
        atr_tp_multiplier: Multiplicador de ATR para take-profit dinámico.

    Example::
        sizer = KellySizer(fraction=0.25, max_position_pct=0.20)
        pnls = [120.0, -50.0, 80.0, -30.0, 200.0, -40.0, 90.0]
        result = sizer.compute("BTC/USDT", "ma_cross", pnls)
        size = sizer.position_size(capital=10_000, price=65_000, atr=800, result=result)
    """
    fraction: float = _DEFAULT_FRACTION
    max_position_pct: float = 0.20
    min_samples: int = _MIN_SAMPLES
    atr_sl_multiplier: float = 1.5
    atr_tp_multiplier: float = 3.0
    _cache: dict = field(default_factory=dict, repr=False)

    def compute(
        self,
        symbol: str,
        strategy: str,
        pnls: Sequence[float],
    ) -> KellyResult:
        """Calcula la fracción de Kelly a partir del historial de PnLs.

        Fórmula Kelly básica:
            f* = (p * b - q) / b
        donde:
            p = win_rate (probabilidad de ganar)
            q = 1 - p   (probabilidad de perder)
            b = avg_win / avg_loss (odds ratio)

        Args:
            symbol: Ticker/par del activo.
            strategy: Nombre de la estrategia.
            pnls: Lista de PnLs realizados (positivos=ganancia, negativos=pérdida).

        Returns:
            KellyResult con la fracción óptima y metadatos.
        """
        cache_key = f"{symbol}:{strategy}:{len(pnls)}"
        if cache_key in self._cache:
            return self._cache[cache_key]

        n = len(pnls)
        if n < self.min_samples:
            result = KellyResult(
                symbol=symbol, strategy=strategy,
                kelly_full=0.0, kelly_fraction=0.0,
                position_pct=self.max_position_pct * 0.5,  # default conservador
                win_rate=0.5, win_loss_ratio=1.0,
                samples=n, confidence=0.0,
                note=f"Muestras insuficientes ({n}<{self.min_samples}) — usando default conservador",
            )
            self._cache[cache_key] = result
            return result

        wins  = [p for p in pnls if p > 0]
        loses = [p for p in pnls if p <= 0]

        p = len(wins) / n                               # win rate
        q = 1.0 - p                                     # loss rate

        avg_win  = sum(wins) / len(wins)   if wins  else 0.0
        avg_loss = abs(sum(loses) / len(loses)) if loses else 1e-6

        b = avg_win / avg_loss if avg_loss > 0 else avg_win  # odds ratio

        # Kelly completo: f* = (p*b - q) / b
        if b <= 0:
            kelly_full = 0.0
        else:
            kelly_full = (p * b - q) / b

        kelly_full = max(0.0, min(kelly_full, _MAX_KELLY_RAW))

        # Fracción aplicada (quarter-Kelly por defecto)
        kelly_frac = kelly_full * self.fraction

        # Cap por máximo de posición configurado
        capped = kelly_frac > self.max_position_pct
        position_pct = min(kelly_frac, self.max_position_pct)

        # Confianza: sqrt(n) normalizado — crece con más muestras
        confidence = min(1.0, math.sqrt(n) / math.sqrt(100))

        result = KellyResult(
            symbol=symbol, strategy=strategy,
            kelly_full=round(kelly_full, 6),
            kelly_fraction=round(kelly_frac, 6),
            position_pct=round(position_pct, 6),
            win_rate=round(p, 4),
            win_loss_ratio=round(b, 4),
            samples=n,
            confidence=round(confidence, 4),
            capped=capped,
            note=f"{'¼' if self.fraction == 0.25 else '½'}-Kelly aplicado",
        )
        self._cache[cache_key] = result
        logger.debug(
            "[Kelly] %s/%s → f*=%.4f frac=%.4f pos=%.2f%% (p=%.2f b=%.2f n=%d)",
            symbol, strategy,
            kelly_full, kelly_frac, position_pct * 100,
            p, b, n,
        )
        return result

    def position_size(
        self,
        capital: float,
        price: float,
        result: KellyResult,
        atr: float | None = None,
        max_risk_per_trade_pct: float = 0.01,
    ) -> float:
        """Calcula el número de unidades a comprar dado el capital y Kelly result.

        Si se provee ATR, aplica sizing adicional ajustado por volatilidad:
            units = (capital * position_pct) / price
        Luego verifica que el riesgo ATR-based no exceda el 1% del capital.

        Args:
            capital: Capital disponible en USD.
            price: Precio actual del activo.
            result: KellyResult con position_pct calculado.
            atr: Average True Range actual (opcional, para riesgo adicional).

        Returns:
            Número de unidades a comprar (float).
        """
        return self.max_units(
            capital=capital,
            price=price,
            atr=atr,
            position_pct=result.position_pct,
            max_risk_per_trade_pct=max_risk_per_trade_pct,
        )

    def max_units(
        self,
        capital: float,
        price: float,
        atr: float | None = None,
        position_pct: float | None = None,
        max_risk_per_trade_pct: float = 0.01,
    ) -> float:
        """Devuelve el máximo de unidades permitido por capital, Kelly y riesgo.

        Esta helper permite reutilizar el mismo cap conservador tanto para
        sizing Kelly puro como para fallbacks seguros cuando no hay suficiente
        historial estadístico.
        """
        if price <= 0 or capital <= 0:
            return 0.0

        effective_pct = self.max_position_pct if position_pct is None else float(position_pct)
        effective_pct = max(0.0, min(effective_pct, self.max_position_pct))

        usd_to_allocate = capital * effective_pct
        units = usd_to_allocate / price

        if atr and atr > 0 and max_risk_per_trade_pct > 0:
            risk_per_unit = atr * self.atr_sl_multiplier
            if risk_per_unit > 0:
                max_units_by_risk = (capital * max_risk_per_trade_pct) / risk_per_unit
                units = min(units, max_units_by_risk)

        return max(0.0, units)

    def atr_stops(
        self,
        price: float,
        atr: float,
        side: str = "long",
    ) -> tuple[float, float]:
        """Calcula stop-loss y take-profit dinámicos basados en ATR.

        Stop-loss  = price - atr * sl_mult  (long) | price + atr * sl_mult  (short)
        Take-profit = price + atr * tp_mult  (long) | price - atr * tp_mult  (short)

        Args:
            price: Precio de entrada.
            atr: Average True Range actual.
            side: "long" o "short".

        Returns:
            Tupla (stop_loss, take_profit).
        """
        sl_dist = atr * self.atr_sl_multiplier
        tp_dist = atr * self.atr_tp_multiplier

        if side == "long":
            return (price - sl_dist, price + tp_dist)
        else:
            return (price + sl_dist, price - tp_dist)

    def invalidate_cache(self) -> None:
        """Limpia la caché de resultados Kelly (llamar al reentrenar modelos)."""
        self._cache.clear()


# ── Singleton global ──────────────────────────────────────────────────────────
_sizer: KellySizer | None = None


def get_kelly_sizer(
    fraction: float = _DEFAULT_FRACTION,
    max_position_pct: float = 0.20,
) -> KellySizer:
    """Retorna la instancia global del KellySizer (lazy singleton)."""
    global _sizer
    if _sizer is None:
        _sizer = KellySizer(fraction=fraction, max_position_pct=max_position_pct)
    return _sizer
