"""Motor de estrategias multi-activo para ATLAS-Quant.

Implementa el núcleo del "OptionStrat interno":
  - Dataclasses para legs de opciones y lineales
  - Black-Scholes puro (sin scipy, solo math + numpy)
  - Payoff a vencimiento y PnL T+0
  - Griegas por leg y agregadas
  - Breakevens, resumen de riesgo

Filosofía de diseño
-------------------
- Sin dependencias externas más allá de numpy y math (stdlib).
- Funciones puras donde sea posible (testables, sin estado).
- Compatible con el ecosistema ATLAS: AssetClass, TradeEvent, LearningBrain.
- Preciso para opciones europeas (fórmula BS exacta).
  Para americanas usa misma fórmula como aproximación.
"""
from __future__ import annotations

import math
import numpy as np
from dataclasses import dataclass, field
from datetime import date, datetime
from typing import Dict, List, Literal, Optional, Union


# ---------------------------------------------------------------------------
# Black-Scholes puro — sin scipy, usando math.erf
# ---------------------------------------------------------------------------

def _norm_cdf(x: float) -> float:
    """CDF de la distribución normal estándar usando math.erf."""
    return (1.0 + math.erf(x / math.sqrt(2.0))) / 2.0


def _norm_pdf(x: float) -> float:
    """PDF de la distribución normal estándar."""
    return math.exp(-0.5 * x * x) / math.sqrt(2.0 * math.pi)


def bs_price(
    S: float,       # precio subyacente
    K: float,       # strike
    T: float,       # tiempo hasta vencimiento en años (>0)
    r: float,       # tasa libre de riesgo (anual, e.g. 0.05)
    sigma: float,   # volatilidad implícita anual (e.g. 0.25)
    option_type: str,  # "call" | "put"
) -> float:
    """Precio Black-Scholes para opción europea."""
    if T <= 0:
        # A vencimiento: valor intrínseco
        if option_type == "call":
            return max(0.0, S - K)
        return max(0.0, K - S)

    if sigma <= 0:
        sigma = 1e-9

    sqrt_T = math.sqrt(T)
    d1 = (math.log(S / K) + (r + 0.5 * sigma ** 2) * T) / (sigma * sqrt_T)
    d2 = d1 - sigma * sqrt_T

    if option_type == "call":
        return S * _norm_cdf(d1) - K * math.exp(-r * T) * _norm_cdf(d2)
    else:
        return K * math.exp(-r * T) * _norm_cdf(-d2) - S * _norm_cdf(-d1)


def bs_greeks(
    S: float,
    K: float,
    T: float,
    r: float,
    sigma: float,
    option_type: str,
) -> Dict[str, float]:
    """Griegas BS para una opción. Devuelve delta, gamma, theta, vega, rho."""
    if T <= 1e-9:
        # A vencimiento las griegas son discontinuas; aproximar a 0
        return {"delta": 0.0, "gamma": 0.0, "theta": 0.0, "vega": 0.0, "rho": 0.0}

    if sigma <= 0:
        sigma = 1e-9

    sqrt_T = math.sqrt(T)
    d1 = (math.log(S / K) + (r + 0.5 * sigma ** 2) * T) / (sigma * sqrt_T)
    d2 = d1 - sigma * sqrt_T

    pdf_d1 = _norm_pdf(d1)
    disc = math.exp(-r * T)

    # Delta
    delta = _norm_cdf(d1) if option_type == "call" else _norm_cdf(d1) - 1.0

    # Gamma (igual para call y put)
    gamma = pdf_d1 / (S * sigma * sqrt_T)

    # Theta (por día calendario — dividir entre 365)
    if option_type == "call":
        theta = (
            -S * pdf_d1 * sigma / (2.0 * sqrt_T)
            - r * K * disc * _norm_cdf(d2)
        ) / 365.0
    else:
        theta = (
            -S * pdf_d1 * sigma / (2.0 * sqrt_T)
            + r * K * disc * _norm_cdf(-d2)
        ) / 365.0

    # Vega (por 1% de IV — ×0.01)
    vega = S * sqrt_T * pdf_d1 * 0.01

    # Rho (por 1% de tasa de interés — ×0.01)
    if option_type == "call":
        rho = K * T * disc * _norm_cdf(d2) * 0.01
    else:
        rho = -K * T * disc * _norm_cdf(-d2) * 0.01

    return {"delta": delta, "gamma": gamma, "theta": theta, "vega": vega, "rho": rho}


def implied_vol(
    spot: float,
    strike: float,
    time_to_expiry: float,
    rate: float,
    option_price: float,
    option_type: str = "call",
    *,
    tol: float = 1e-6,
    max_iter: int = 100,
) -> float | None:
    """Invierte Black–Scholes: encuentra :math:`\\sigma` tal que ``bs_price(...) ≈ option_price``.

    Usa **bisección** en un rango fijo de volatilidades ``[1e-4, 5.0]`` (anualizada).
    En cada iteración se reutiliza :func:`bs_price` (mismo modelo europeo, sin dividendos).

    **Método numérico:** búsqueda binaria sobre :math:`\\sigma` porque el precio BS
    crece monótonamente con la volatilidad (vega > 0) para calls y puts europeos.

    **Limitaciones (documentadas):**

    - Tolerancia ``tol`` se aplica al **error en precio** :math:`|C(\\sigma) - C^*|`,
      no directamente a :math:`|\\sigma - \\sigma^*|`.
    - Sin dividendos; coherente con :func:`bs_price`.
    - Si el precio de mercado está fuera del rango de precios alcanzables con
      :math:`\\sigma \\in [10^{-4}, 5]` (vía BS), se devuelve ``None`` — no se
      extrapola fuera del intervalo de volatilidad.
    - ``time_to_expiry`` debe ser **> 0** (años). Si ``T <= 0``, se devuelve ``None``.

    Args:
        spot: Precio spot del subyacente (> 0).
        strike: Strike (> 0).
        time_to_expiry: Tiempo hasta vencimiento en **años** (> 0).
        rate: Tasa libre de riesgo anual continua.
        option_price: Precio de mercado de la opción (una unidad, mismo que BS).
        option_type: ``call`` o ``put`` (insensible a mayúsculas).
        tol: Criterio de parada sobre :math:`|C(\\sigma) - C^*|`.
        max_iter: Máximo de iteraciones de bisección.

    Returns:
        Volatilidad implícita anualizada, o ``None`` si no hay solución razonable
        en el rango o los argumentos son inválidos.
    """
    if time_to_expiry <= 0.0:
        return None
    if spot <= 0.0 or strike <= 0.0 or option_price < 0.0:
        return None

    ot = str(option_type).strip().lower()
    if ot not in ("call", "put"):
        return None

    disc = math.exp(-rate * time_to_expiry)
    if ot == "call":
        price_floor = max(0.0, spot - strike * disc)
        price_ceil = spot
    else:
        price_floor = max(0.0, strike * disc - spot)
        price_ceil = strike * disc

    # Cotas libres de arbitraje (europeo); precios fuera no admiten solución BS positiva.
    if option_price < price_floor - 1e-8 or option_price > price_ceil + 1e-8:
        return None

    sig_lo = 1e-4
    sig_hi = 5.0
    p_lo = bs_price(spot, strike, time_to_expiry, rate, sig_lo, ot)
    p_hi = bs_price(spot, strike, time_to_expiry, rate, sig_hi, ot)

    if p_lo > p_hi:
        p_lo, p_hi = p_hi, p_lo

    if abs(p_lo - option_price) < tol:
        return float(sig_lo)
    if abs(p_hi - option_price) < tol:
        return float(sig_hi)

    # Precio objetivo fuera de lo alcanzable en [sig_lo, sig_hi] con BS monótono.
    if option_price < p_lo - tol * 10.0 or option_price > p_hi + tol * 10.0:
        return None

    lo_b, hi_b = sig_lo, sig_hi
    for _ in range(max(1, max_iter)):
        mid = 0.5 * (lo_b + hi_b)
        pm = bs_price(spot, strike, time_to_expiry, rate, mid, ot)
        diff = pm - option_price
        if abs(diff) < tol:
            return float(mid)
        if diff > 0.0:
            hi_b = mid
        else:
            lo_b = mid

    final = 0.5 * (lo_b + hi_b)
    p_final = bs_price(spot, strike, time_to_expiry, rate, final, ot)
    if abs(p_final - option_price) < tol * 100.0:
        return float(final)
    return None


# ---------------------------------------------------------------------------
# Dataclasses de legs
# ---------------------------------------------------------------------------

@dataclass
class OptionLeg:
    """Leg de opción (call o put, long o short)."""
    underlying_symbol: str
    expiry: date
    strike: float
    option_type: Literal["call", "put"]
    side: Literal["long", "short"]
    quantity: int = 1              # número de contratos
    premium: float = 0.0          # precio pagado (+) o cobrado (-) por contrato
    multiplier: int = 100          # típicamente 100 acciones/contrato
    iv: float = 0.25               # IV implícita estimada (para pricing T+0)
    underlying_price: float = 0.0  # spot al momento de entrada

    @property
    def sign(self) -> int:
        return 1 if self.side == "long" else -1

    @property
    def tte_years(self) -> float:
        """Años hasta vencimiento desde hoy."""
        today = datetime.utcnow().date()
        days = (self.expiry - today).days
        return max(0.0, days / 365.0)

    def payoff_at_expiry(self, S_T: float) -> float:
        """PnL en dólares a vencimiento para un precio dado S_T."""
        if self.option_type == "call":
            intrinsic = max(0.0, S_T - self.strike)
        else:
            intrinsic = max(0.0, self.strike - S_T)
        # long: recibe intrínseco − prima pagada; short: cobra prima − intrínseco
        return self.sign * (intrinsic - abs(self.premium)) * self.quantity * self.multiplier

    def current_price(self, S: float, T: float, r: float = 0.05) -> float:
        """Precio BS actual de la opción."""
        return bs_price(S, self.strike, T, r, self.iv, self.option_type)

    def pnl_today(self, S: float, T: float, r: float = 0.05) -> float:
        """PnL actual (precio actual − prima de entrada)."""
        price_now = self.current_price(S, T, r)
        return self.sign * (price_now - abs(self.premium)) * self.quantity * self.multiplier

    def greeks_vector(self, S: float, T: float, r: float = 0.05) -> Dict[str, float]:
        """Griegas para esta leg (incluye signo y cantidad)."""
        raw = bs_greeks(S, self.strike, T, r, self.iv, self.option_type)
        factor = self.sign * self.quantity * self.multiplier
        return {
            "delta": raw["delta"] * factor,
            "gamma": raw["gamma"] * factor,
            "theta": raw["theta"] * factor,
            "vega":  raw["vega"]  * factor,
            "rho":   raw["rho"]   * factor,
        }


@dataclass
class LinearLeg:
    """Leg lineal: acción, índice, ETF o futuro."""
    symbol: str
    asset_class: Literal["stock", "index", "etf", "future"]
    side: Literal["long", "short"]
    quantity: int = 1
    entry_price: float = 0.0
    multiplier: float = 1.0     # 1 para acciones; tamaño contrato para futuros (e.g. 50 para ES)

    @property
    def sign(self) -> int:
        return 1 if self.side == "long" else -1

    def payoff_at_expiry(self, S_T: float) -> float:
        """PnL en dólares."""
        return self.sign * (S_T - self.entry_price) * self.quantity * self.multiplier

    def pnl_today(self, S: float, **_) -> float:
        """PnL actual (precio S vs entry_price)."""
        return self.payoff_at_expiry(S)

    def greeks_vector(self, **_) -> Dict[str, float]:
        """Delta = ±qty×multiplier; resto 0."""
        delta = self.sign * self.quantity * self.multiplier
        return {"delta": delta, "gamma": 0.0, "theta": 0.0, "vega": 0.0, "rho": 0.0}


Leg = Union[OptionLeg, LinearLeg]


# ---------------------------------------------------------------------------
# Strategy
# ---------------------------------------------------------------------------

@dataclass
class Strategy:
    """Estrategia de trading multi-leg."""
    name: str
    legs: List[Leg]
    metadata: Dict = field(default_factory=dict)
    # Campos opcionales de contexto
    underlying: str = ""          # símbolo principal
    created_at: datetime = field(default_factory=datetime.utcnow)

    @property
    def option_legs(self) -> List[OptionLeg]:
        return [l for l in self.legs if isinstance(l, OptionLeg)]

    @property
    def linear_legs(self) -> List[LinearLeg]:
        return [l for l in self.legs if isinstance(l, LinearLeg)]

    @property
    def net_premium(self) -> float:
        """Crédito neto (+) o débito neto (-) de la posición.
        Positivo = credit spread (cobras); negativo = debit (pagas).
        """
        total = 0.0
        for leg in self.option_legs:
            # short cobra prima → +; long paga prima → -
            total += (-leg.sign) * abs(leg.premium) * leg.quantity * leg.multiplier
        return total

    @property
    def is_credit(self) -> bool:
        return self.net_premium > 0


# ---------------------------------------------------------------------------
# MarketSnapshot — datos de mercado para pricing T+0
# ---------------------------------------------------------------------------

@dataclass
class MarketSnapshot:
    """Snapshot de mercado para calcular PnL actual y griegas."""
    underlying_price: float
    risk_free_rate: float = 0.05    # tasa libre de riesgo anual
    # Override de IV por strike (opcional)
    iv_overrides: Dict[float, float] = field(default_factory=dict)
    # Días hasta vencimiento override (para escenarios)
    dte_override: Optional[int] = None

    def iv_for_strike(self, strike: float, default_iv: float) -> float:
        return self.iv_overrides.get(strike, default_iv)


# ---------------------------------------------------------------------------
# GreeksVector — griegas agregadas
# ---------------------------------------------------------------------------

@dataclass
class GreeksVector:
    delta: float = 0.0
    gamma: float = 0.0
    theta: float = 0.0    # por día
    vega:  float = 0.0    # por 1% IV
    rho:   float = 0.0    # por 1% tasa

    def __add__(self, other: "GreeksVector") -> "GreeksVector":
        return GreeksVector(
            delta=self.delta + other.delta,
            gamma=self.gamma + other.gamma,
            theta=self.theta + other.theta,
            vega=self.vega  + other.vega,
            rho=self.rho    + other.rho,
        )

    def to_dict(self) -> Dict[str, float]:
        return {
            "delta": round(self.delta, 4),
            "gamma": round(self.gamma, 6),
            "theta": round(self.theta, 4),
            "vega":  round(self.vega,  4),
            "rho":   round(self.rho,   4),
        }


# ---------------------------------------------------------------------------
# StrategyRiskSummary
# ---------------------------------------------------------------------------

@dataclass
class StrategyRiskSummary:
    strategy_name: str
    max_profit:  float        # inf si ilimitado
    max_loss:    float        # -inf si ilimitado (puts/calls short naked)
    net_premium: float        # crédito > 0, débito < 0
    breakevens:  List[float]
    greeks:      GreeksVector
    current_pnl: float = 0.0  # PnL con precios actuales
    probability_of_profit: float = 0.0  # estimación simple

    def to_dict(self) -> Dict:
        return {
            "strategy_name": self.strategy_name,
            "max_profit":  round(self.max_profit, 2) if not math.isinf(self.max_profit) else None,
            "max_loss":    round(self.max_loss, 2)   if not math.isinf(self.max_loss)   else None,
            "net_premium": round(self.net_premium, 2),
            "is_credit":   self.net_premium > 0,
            "breakevens":  [round(b, 2) for b in self.breakevens],
            "greeks":      self.greeks.to_dict(),
            "current_pnl": round(self.current_pnl, 2),
            "pop_estimate": round(self.probability_of_profit, 3),
        }


# ---------------------------------------------------------------------------
# Funciones de cálculo principales
# ---------------------------------------------------------------------------

def payoff_at_expiry(strategy: Strategy, S_grid: np.ndarray) -> np.ndarray:
    """Payoff total a vencimiento para cada precio en S_grid.

    Args:
        strategy: estrategia con sus legs
        S_grid:   array de precios del subyacente (e.g. np.linspace(80, 120, 500))

    Returns:
        array de PnL en dólares de la misma longitud que S_grid
    """
    total = np.zeros(len(S_grid))
    for leg in strategy.legs:
        for i, S_T in enumerate(S_grid):
            total[i] += leg.payoff_at_expiry(float(S_T))
    return total


def pnl_today(
    strategy: Strategy,
    market: MarketSnapshot,
    S_grid: np.ndarray,
) -> np.ndarray:
    """PnL actual (T+0) para cada precio hipotético en S_grid.

    Usa Black-Scholes para repriced las legs de opciones.
    Las legs lineales usan PnL directa.
    """
    pnl = np.zeros(len(S_grid))
    r = market.risk_free_rate

    for leg in strategy.legs:
        if isinstance(leg, OptionLeg):
            # Usar DTE override si lo hay
            if market.dte_override is not None:
                T = max(0.0, market.dte_override / 365.0)
            else:
                T = leg.tte_years
            iv = market.iv_for_strike(leg.strike, leg.iv)

            for i, S in enumerate(S_grid):
                price_now = bs_price(float(S), leg.strike, T, r, iv, leg.option_type)
                pnl[i] += leg.sign * (price_now - abs(leg.premium)) * leg.quantity * leg.multiplier
        else:
            for i, S in enumerate(S_grid):
                pnl[i] += leg.pnl_today(float(S))
    return pnl


def greeks(strategy: Strategy, market: MarketSnapshot) -> GreeksVector:
    """Griegas agregadas de toda la estrategia al precio spot actual."""
    S = market.underlying_price
    r = market.risk_free_rate
    result = GreeksVector()

    for leg in strategy.legs:
        if isinstance(leg, OptionLeg):
            T = (market.dte_override / 365.0) if market.dte_override is not None else leg.tte_years
            iv = market.iv_for_strike(leg.strike, leg.iv)
            g = leg.greeks_vector(S, T, r)
        else:
            g = leg.greeks_vector()

        result = GreeksVector(
            delta=result.delta + g["delta"],
            gamma=result.gamma + g["gamma"],
            theta=result.theta + g["theta"],
            vega= result.vega  + g["vega"],
            rho=  result.rho   + g["rho"],
        )
    return result


def breakeven_points(strategy: Strategy, S_grid: np.ndarray) -> List[float]:
    """Encuentra los puntos donde el payoff a vencimiento cruza cero.

    Detecta cruces de signo en la curva de payoff interpolando linealmente.
    """
    payoff = payoff_at_expiry(strategy, S_grid)
    bk: List[float] = []

    for i in range(len(payoff) - 1):
        y0, y1 = payoff[i], payoff[i + 1]
        if y0 == 0.0:
            bk.append(float(S_grid[i]))
        elif y1 == 0.0:
            pass  # se captura en la próxima iteración
        elif (y0 > 0) != (y1 > 0):
            # Interpolación lineal para encontrar el cero exacto
            x0, x1 = float(S_grid[i]), float(S_grid[i + 1])
            bk.append(x0 - y0 * (x1 - x0) / (y1 - y0))

    # Eliminar duplicados cercanos
    bk.sort()
    deduped: List[float] = []
    for b in bk:
        if not deduped or abs(b - deduped[-1]) > 0.01:
            deduped.append(round(b, 4))
    return deduped


def strategy_risk_summary(
    strategy: Strategy,
    market: MarketSnapshot,
    S_grid: Optional[np.ndarray] = None,
) -> StrategyRiskSummary:
    """Resumen completo de riesgo: max profit, max loss, breakevens, greeks.

    Si S_grid no se provee, se genera automáticamente alrededor del subyacente.
    """
    S = market.underlying_price
    if S_grid is None:
        spread = max(50.0, S * 0.40)   # ±40% del spot
        S_grid = np.linspace(max(0.01, S - spread), S + spread, 1000)

    payoff = payoff_at_expiry(strategy, S_grid)
    pnl_0  = pnl_today(strategy, market, S_grid)
    bk     = breakeven_points(strategy, S_grid)
    g      = greeks(strategy, market)

    max_p = float(payoff.max())
    max_l = float(payoff.min())

    # Si la estrategia tiene opciones cortas sin cobertura, pérdida puede ser ilimitada
    has_naked_short_call = any(
        isinstance(l, OptionLeg) and l.side == "short" and l.option_type == "call"
        and not _is_call_covered(strategy, l)
        for l in strategy.legs
    )
    has_naked_short_put = any(
        isinstance(l, OptionLeg) and l.side == "short" and l.option_type == "put"
        and not _is_put_covered(strategy, l)
        for l in strategy.legs
    )

    if has_naked_short_call:
        max_p_final = float("inf")  # short call: max profit = prima, pero puede subir infinito el payoff long
        # En este contexto max_profit es la prima cobrada, max_loss es infinita
        max_p_final = float(np.max(payoff))
        max_l = float("-inf")
    elif has_naked_short_put:
        max_l = max(-S * strategy.legs[0].quantity * 100, float(payoff.min()))

    # PnL actual al spot actual
    current_pnl = float(pnl_today(strategy, market, np.array([S]))[0])

    # Probabilidad de profit estimada (% de S_grid donde payoff > 0)
    pop = float(np.mean(payoff > 0))

    return StrategyRiskSummary(
        strategy_name=strategy.name,
        max_profit=max_p,
        max_loss=max_l,
        net_premium=strategy.net_premium,
        breakevens=bk,
        greeks=g,
        current_pnl=current_pnl,
        probability_of_profit=pop,
    )


def _is_call_covered(strategy: Strategy, short_call: OptionLeg) -> bool:
    """Verifica si una call corta está cubierta por otra call larga de igual o menor strike."""
    for l in strategy.legs:
        if (isinstance(l, OptionLeg) and l.side == "long" and l.option_type == "call"
                and l.underlying_symbol == short_call.underlying_symbol
                and l.strike <= short_call.strike):
            return True
        if isinstance(l, LinearLeg) and l.side == "long":
            return True  # covered call
    return False


def _is_put_covered(strategy: Strategy, short_put: OptionLeg) -> bool:
    """Verifica si una put corta está cubierta por otra put larga de mayor o igual strike."""
    for l in strategy.legs:
        if (isinstance(l, OptionLeg) and l.side == "long" and l.option_type == "put"
                and l.underlying_symbol == short_put.underlying_symbol
                and l.strike >= short_put.strike):
            return True
    return False


# ---------------------------------------------------------------------------
# Funciones de escenario — griegas/PnL con parámetros variables
# ---------------------------------------------------------------------------

def scenario_pnl(
    strategy: Strategy,
    S_values: List[float],
    dte_values: List[int],
    iv_multiplier: float = 1.0,
    r: float = 0.05,
) -> np.ndarray:
    """Matriz de PnL: filas = S_values, columnas = dte_values.

    Útil para heatmaps de escenario (precio × tiempo).

    Args:
        S_values:       lista de precios del subyacente
        dte_values:     lista de días hasta vencimiento
        iv_multiplier:  escalar para la IV (0.8 = -20% IV)
        r:              tasa libre de riesgo

    Returns:
        array de shape (len(S_values), len(dte_values))
    """
    result = np.zeros((len(S_values), len(dte_values)))
    for col, dte in enumerate(dte_values):
        snap = MarketSnapshot(
            underlying_price=S_values[0],
            risk_free_rate=r,
            dte_override=dte,
        )
        for row, S in enumerate(S_values):
            pnl_row = 0.0
            for leg in strategy.legs:
                if isinstance(leg, OptionLeg):
                    T = max(0.0, dte / 365.0)
                    iv = leg.iv * iv_multiplier
                    price_now = bs_price(float(S), leg.strike, T, r, iv, leg.option_type)
                    pnl_row += leg.sign * (price_now - abs(leg.premium)) * leg.quantity * leg.multiplier
                else:
                    pnl_row += leg.sign * (S - leg.entry_price) * leg.quantity * leg.multiplier
            result[row, col] = pnl_row
    return result


# ---------------------------------------------------------------------------
# Serialización / deserialización (para API)
# ---------------------------------------------------------------------------

def strategy_to_dict(strategy: Strategy) -> dict:
    """Serializa una estrategia a dict para JSON."""
    legs = []
    for leg in strategy.legs:
        if isinstance(leg, OptionLeg):
            legs.append({
                "leg_type": "option",
                "underlying_symbol": leg.underlying_symbol,
                "expiry": leg.expiry.isoformat(),
                "strike": leg.strike,
                "option_type": leg.option_type,
                "side": leg.side,
                "quantity": leg.quantity,
                "premium": leg.premium,
                "multiplier": leg.multiplier,
                "iv": leg.iv,
            })
        else:
            legs.append({
                "leg_type": "linear",
                "symbol": leg.symbol,
                "asset_class": leg.asset_class,
                "side": leg.side,
                "quantity": leg.quantity,
                "entry_price": leg.entry_price,
                "multiplier": leg.multiplier,
            })
    return {
        "name": strategy.name,
        "underlying": strategy.underlying,
        "legs": legs,
        "metadata": strategy.metadata,
        "net_premium": round(strategy.net_premium, 2),
        "is_credit": strategy.is_credit,
    }


def strategy_from_dict(d: dict) -> Strategy:
    """Deserializa una estrategia desde dict JSON."""
    legs: List[Leg] = []
    for leg_d in d.get("legs", []):
        if leg_d.get("leg_type") == "option":
            legs.append(OptionLeg(
                underlying_symbol=leg_d["underlying_symbol"],
                expiry=date.fromisoformat(leg_d["expiry"]),
                strike=float(leg_d["strike"]),
                option_type=leg_d["option_type"],
                side=leg_d["side"],
                quantity=int(leg_d.get("quantity", 1)),
                premium=float(leg_d.get("premium", 0.0)),
                multiplier=int(leg_d.get("multiplier", 100)),
                iv=float(leg_d.get("iv", 0.25)),
                underlying_price=float(leg_d.get("underlying_price", 0.0)),
            ))
        else:
            legs.append(LinearLeg(
                symbol=leg_d["symbol"],
                asset_class=leg_d.get("asset_class", "stock"),
                side=leg_d["side"],
                quantity=int(leg_d.get("quantity", 1)),
                entry_price=float(leg_d.get("entry_price", 0.0)),
                multiplier=float(leg_d.get("multiplier", 1.0)),
            ))
    return Strategy(
        name=d.get("name", "unnamed"),
        underlying=d.get("underlying", ""),
        legs=legs,
        metadata=d.get("metadata", {}),
    )
