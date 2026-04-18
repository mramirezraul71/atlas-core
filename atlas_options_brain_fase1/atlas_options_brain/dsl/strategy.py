"""
Atlas Options Brain  –  DSL de Estrategias (Fase 1)
=====================================================
Permite construir estrategias de opciones de forma declarativa y
legible, sin hardcodear strikes manualmente.

Estrategias implementadas:
  - IronCondor
  - BullPutSpread  (vertical crédito)
  - BearCallSpread (vertical crédito)
  - BullCallSpread (vertical débito)
  - BearPutSpread  (vertical débito)
  - CoveredCall

Uso básico:
    strategy = IronCondor.from_chain(
        chain       = chain,
        wing_delta  = 0.16,
        wing_width  = 5,
    )
    print(strategy.summary())
"""
from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import Optional

from ..models.option_contract import OptionContract, OptionType, OptionsChain
from ..models.leg import Leg, OptionRight


# ─── helpers ──────────────────────────────────────────────────────────────────

def _valid_contract(c: OptionContract) -> bool:
    """Descarta filas rotas (NaN, sin precio usable) antes de armar spreads."""
    if not math.isfinite(c.strike) or c.strike <= 0:
        return False
    if not all(math.isfinite(x) for x in (c.last_price, c.bid, c.ask)):
        return False
    if c.last_price <= 0 and c.bid <= 0 and c.ask <= 0:
        return False
    return True


def _effective_abs_delta(c: OptionContract, chain: OptionsChain) -> float:
    """
    |delta| para matching: usa greeks si vienen del proveedor; si no, proxy por moneyness
    (yfinance a menudo no trae delta → sin esto todos empatan en 0 y el min() elige strikes absurdos).
    """
    d = float(c.greeks.delta)
    if math.isfinite(d) and abs(d) > 1e-9:
        return abs(d)
    spot = float(chain.spot_price)
    if not math.isfinite(spot) or spot <= 0:
        return 0.25
    if c.option_type == OptionType.CALL:
        x = (c.strike - spot) / spot
        return max(0.02, min(0.95, 0.55 * math.exp(-max(0.0, x) * 6.0)))
    x = (spot - c.strike) / spot
    return max(0.02, min(0.95, 0.55 * math.exp(-max(0.0, x) * 6.0)))


def _closest_delta(
    chain: OptionsChain,
    target_delta: float,
    option_type: OptionType,
) -> OptionContract:
    """Selecciona el contrato cuyo |delta| (o proxy) es más cercano al target."""
    contracts = [
        c for c in chain.contracts
        if c.option_type == option_type and _valid_contract(c)
    ]
    if not contracts:
        raise ValueError(f"No hay contratos {option_type} en la cadena.")
    return min(
        contracts,
        key=lambda c: abs(_effective_abs_delta(c, chain) - target_delta),
    )


def _strike_above(
    chain: OptionsChain, reference: float, width: float,
    option_type: OptionType = OptionType.CALL,
) -> Optional[OptionContract]:
    """Strike long por encima del short: idealmente >= reference + width; si no hay, el más cercano por encima del short."""
    target = reference + width
    pool = [
        c for c in chain.contracts
        if c.option_type == option_type and _valid_contract(c)
    ]
    at_or_above = [c for c in pool if c.strike >= target]
    if at_or_above:
        return min(at_or_above, key=lambda c: c.strike)
    above_ref = [c for c in pool if c.strike > reference]
    if not above_ref:
        return None
    return min(above_ref, key=lambda c: c.strike)


def _strike_below(
    chain: OptionsChain, reference: float, width: float,
    option_type: OptionType = OptionType.PUT,
) -> Optional[OptionContract]:
    """Strike long por debajo del short put: idealmente <= reference - width; si no hay, el más cercano por debajo del short."""
    target = reference - width
    pool = [
        c for c in chain.contracts
        if c.option_type == option_type and _valid_contract(c)
    ]
    at_or_below = [c for c in pool if c.strike <= target]
    if at_or_below:
        return max(at_or_below, key=lambda c: c.strike)
    below_ref = [c for c in pool if c.strike < reference]
    if not below_ref:
        return None
    return min(below_ref, key=lambda c: abs(c.strike - target))


# ─── base ─────────────────────────────────────────────────────────────────────

@dataclass
class OptionsStrategy:
    name:        str
    legs:        list[Leg]        = field(default_factory=list)
    description: str              = ""

    @property
    def net_premium(self) -> float:
        """Positivo = crédito neto, negativo = débito neto."""
        return round(sum(leg.premium for leg in self.legs), 2)

    @property
    def net_delta(self) -> float:
        return round(sum(leg.net_delta for leg in self.legs), 4)

    @property
    def max_profit(self) -> float:
        raise NotImplementedError

    @property
    def max_loss(self) -> float:
        raise NotImplementedError

    @property
    def breakevens(self) -> list[float]:
        raise NotImplementedError

    def summary(self) -> str:
        lines = [
            f"╔══ {self.name} ══",
            f"║  {self.description}",
            "║  Legs:",
        ]
        for leg in self.legs:
            lines.append(f"║    {leg}")
        lines += [
            f"║  Net Premium  : {'+'if self.net_premium>=0 else ''}{self.net_premium}",
            f"║  Net Delta    : {self.net_delta}",
        ]
        try:
            lines.append(f"║  Max Profit   : {self.max_profit}")
            lines.append(f"║  Max Loss     : {self.max_loss}")
            lines.append(f"║  Breakevens   : {self.breakevens}")
        except NotImplementedError:
            pass
        lines.append("╚" + "═" * 30)
        return "\n".join(lines)


# ─── Iron Condor ──────────────────────────────────────────────────────────────

@dataclass
class IronCondor(OptionsStrategy):
    """
    Iron Condor = Bull Put Spread + Bear Call Spread.
    4 legs, estrategia neutral/rango, crédito neto.

    Estructura:
        Short Put  (put_short_strike)
        Long  Put  (put_short_strike - width)
        Short Call (call_short_strike)
        Long  Call (call_short_strike + width)
    """
    put_long:   Optional[Leg] = None
    put_short:  Optional[Leg] = None
    call_short: Optional[Leg] = None
    call_long:  Optional[Leg] = None

    def __post_init__(self):
        self.name = "Iron Condor"

    @classmethod
    def from_chain(
        cls,
        chain:          OptionsChain,
        wing_delta:     float = 0.16,   # delta objetivo de los short legs
        wing_width:     float = 5.0,    # puntos entre short y long
        qty:            int   = 1,
    ) -> "IronCondor":
        puts = [
            c for c in chain.contracts
            if c.option_type == OptionType.PUT and _valid_contract(c)
        ]
        calls = [
            c for c in chain.contracts
            if c.option_type == OptionType.CALL and _valid_contract(c)
        ]
        if not puts or not calls:
            raise ValueError("No se pudo construir Iron Condor: faltan strikes.")

        puts_ranked = sorted(
            puts,
            key=lambda c: abs(_effective_abs_delta(c, chain) - wing_delta),
        )
        calls_ranked = sorted(
            calls,
            key=lambda c: abs(_effective_abs_delta(c, chain) - wing_delta),
        )

        put_short_c = put_long_c = None
        for ps in puts_ranked[:20]:
            pl = _strike_below(chain, ps.strike, wing_width)
            if pl is not None:
                put_short_c, put_long_c = ps, pl
                break

        call_short_c = call_long_c = None
        for cs in calls_ranked[:20]:
            cl = _strike_above(chain, cs.strike, wing_width)
            if cl is not None:
                call_short_c, call_long_c = cs, cl
                break

        if not all([put_short_c, put_long_c, call_short_c, call_long_c]):
            raise ValueError("No se pudo construir Iron Condor: faltan strikes.")

        put_long   = Leg(put_long_c,   OptionRight.LONG,  qty)
        put_short  = Leg(put_short_c,  OptionRight.SHORT, qty)
        call_short = Leg(call_short_c, OptionRight.SHORT, qty)
        call_long  = Leg(call_long_c,  OptionRight.LONG,  qty)

        ic = cls(
            name        = "Iron Condor",
            legs        = [put_long, put_short, call_short, call_long],
            description = (f"{chain.symbol} | exp:{chain.expiration} | "
                           f"spot:{chain.spot_price} | δ≈{wing_delta} width:{wing_width}"),
            put_long    = put_long,
            put_short   = put_short,
            call_short  = call_short,
            call_long   = call_long,
        )
        return ic

    @classmethod
    def from_strikes(
        cls,
        chain:       OptionsChain,
        put_long:    float,
        put_short:   float,
        call_short:  float,
        call_long:   float,
        qty:         int = 1,
    ) -> "IronCondor":
        def get(strike, otype):
            c = chain.by_strike(strike, otype)
            if c is None:
                raise ValueError(f"Strike {strike} {otype} no encontrado en cadena.")
            return c

        legs = [
            Leg(get(put_long,   OptionType.PUT),  OptionRight.LONG,  qty),
            Leg(get(put_short,  OptionType.PUT),  OptionRight.SHORT, qty),
            Leg(get(call_short, OptionType.CALL), OptionRight.SHORT, qty),
            Leg(get(call_long,  OptionType.CALL), OptionRight.LONG,  qty),
        ]
        ic = cls(name="Iron Condor", legs=legs,
                 description=f"{chain.symbol} exp:{chain.expiration}",
                 put_long=legs[0], put_short=legs[1],
                 call_short=legs[2], call_long=legs[3])
        return ic

    @property
    def max_profit(self) -> float:
        return round(self.net_premium, 2)

    @property
    def max_loss(self) -> float:
        width = abs(self.put_short.contract.strike - self.put_long.contract.strike)
        return round(-(width * 100 * self.put_long.qty - self.net_premium), 2)

    @property
    def breakevens(self) -> list[float]:
        be_put  = round(self.put_short.contract.strike  - self.net_premium / 100, 2)
        be_call = round(self.call_short.contract.strike + self.net_premium / 100, 2)
        return [be_put, be_call]


# ─── Verticals ────────────────────────────────────────────────────────────────

@dataclass
class BullPutSpread(OptionsStrategy):
    """
    Bull Put Spread (vertical crédito alcista).
    Short Put a strike alto + Long Put a strike bajo.
    Crédito neto.  Beneficia si el subyacente sube o se mantiene.
    """
    put_long:  Optional[Leg] = None
    put_short: Optional[Leg] = None

    @classmethod
    def from_chain(
        cls,
        chain:       OptionsChain,
        short_delta: float = 0.30,
        width:       float = 5.0,
        qty:         int   = 1,
    ) -> "BullPutSpread":
        put_short_c = _closest_delta(chain, short_delta, OptionType.PUT)
        put_long_c  = _strike_below(chain, put_short_c.strike, width)
        if put_long_c is None:
            raise ValueError("No se encontró put long para el spread.")

        pl = Leg(put_long_c,  OptionRight.LONG,  qty)
        ps = Leg(put_short_c, OptionRight.SHORT, qty)
        return cls(
            name       = "Bull Put Spread",
            legs       = [pl, ps],
            description= (f"{chain.symbol} | exp:{chain.expiration} | "
                          f"short δ≈{short_delta} width:{width}"),
            put_long   = pl,
            put_short  = ps,
        )

    @property
    def max_profit(self) -> float:
        return round(self.net_premium, 2)

    @property
    def max_loss(self) -> float:
        width = abs(self.put_short.contract.strike - self.put_long.contract.strike)
        return round(-(width * 100 * self.put_long.qty - self.net_premium), 2)

    @property
    def breakevens(self) -> list[float]:
        return [round(self.put_short.contract.strike - self.net_premium / 100, 2)]


@dataclass
class BearCallSpread(OptionsStrategy):
    """
    Bear Call Spread (vertical crédito bajista).
    Short Call a strike bajo + Long Call a strike alto.
    Crédito neto.  Beneficia si el subyacente baja o se mantiene.
    """
    call_short: Optional[Leg] = None
    call_long:  Optional[Leg] = None

    @classmethod
    def from_chain(
        cls,
        chain:       OptionsChain,
        short_delta: float = 0.30,
        width:       float = 5.0,
        qty:         int   = 1,
    ) -> "BearCallSpread":
        call_short_c = _closest_delta(chain, short_delta, OptionType.CALL)
        call_long_c  = _strike_above(chain, call_short_c.strike, width)
        if call_long_c is None:
            raise ValueError("No se encontró call long para el spread.")

        cs = Leg(call_short_c, OptionRight.SHORT, qty)
        cl = Leg(call_long_c,  OptionRight.LONG,  qty)
        return cls(
            name        = "Bear Call Spread",
            legs        = [cs, cl],
            description = (f"{chain.symbol} | exp:{chain.expiration} | "
                           f"short δ≈{short_delta} width:{width}"),
            call_short  = cs,
            call_long   = cl,
        )

    @property
    def max_profit(self) -> float:
        return round(self.net_premium, 2)

    @property
    def max_loss(self) -> float:
        width = abs(self.call_long.contract.strike - self.call_short.contract.strike)
        return round(-(width * 100 * self.call_long.qty - self.net_premium), 2)

    @property
    def breakevens(self) -> list[float]:
        return [round(self.call_short.contract.strike + self.net_premium / 100, 2)]


@dataclass
class BullCallSpread(OptionsStrategy):
    """Bull Call Spread (vertical débito alcista)."""
    call_long:  Optional[Leg] = None
    call_short: Optional[Leg] = None

    @classmethod
    def from_chain(
        cls,
        chain:       OptionsChain,
        long_delta:  float = 0.50,
        width:       float = 5.0,
        qty:         int   = 1,
    ) -> "BullCallSpread":
        call_long_c  = _closest_delta(chain, long_delta, OptionType.CALL)
        call_short_c = _strike_above(chain, call_long_c.strike, width)
        if call_short_c is None:
            raise ValueError("No se encontró call short para el spread.")

        cl = Leg(call_long_c,  OptionRight.LONG,  qty)
        cs = Leg(call_short_c, OptionRight.SHORT, qty)
        return cls(
            name        = "Bull Call Spread",
            legs        = [cl, cs],
            description = (f"{chain.symbol} | exp:{chain.expiration} | "
                           f"long δ≈{long_delta} width:{width}"),
            call_long   = cl,
            call_short  = cs,
        )

    @property
    def max_profit(self) -> float:
        width = abs(self.call_short.contract.strike - self.call_long.contract.strike)
        return round(width * 100 * self.call_long.qty + self.net_premium, 2)

    @property
    def max_loss(self) -> float:
        return round(self.net_premium, 2)

    @property
    def breakevens(self) -> list[float]:
        return [round(self.call_long.contract.strike - self.net_premium / 100, 2)]


@dataclass
class BearPutSpread(OptionsStrategy):
    """Bear Put Spread (vertical débito bajista)."""
    put_long:  Optional[Leg] = None
    put_short: Optional[Leg] = None

    @classmethod
    def from_chain(
        cls,
        chain:       OptionsChain,
        long_delta:  float = 0.50,
        width:       float = 5.0,
        qty:         int   = 1,
    ) -> "BearPutSpread":
        put_long_c  = _closest_delta(chain, long_delta, OptionType.PUT)
        put_short_c = _strike_below(chain, put_long_c.strike, width)
        if put_short_c is None:
            raise ValueError("No se encontró put short para el spread.")

        pl = Leg(put_long_c,  OptionRight.LONG,  qty)
        ps = Leg(put_short_c, OptionRight.SHORT, qty)
        return cls(
            name       = "Bear Put Spread",
            legs       = [pl, ps],
            description= (f"{chain.symbol} | exp:{chain.expiration} | "
                          f"long δ≈{long_delta} width:{width}"),
            put_long   = pl,
            put_short  = ps,
        )

    @property
    def max_profit(self) -> float:
        width = abs(self.put_long.contract.strike - self.put_short.contract.strike)
        return round(width * 100 * self.put_long.qty + self.net_premium, 2)

    @property
    def max_loss(self) -> float:
        return round(self.net_premium, 2)

    @property
    def breakevens(self) -> list[float]:
        return [round(self.put_long.contract.strike + self.net_premium / 100, 2)]


# ─── Covered Call ─────────────────────────────────────────────────────────────

@dataclass
class CoveredCall(OptionsStrategy):
    """
    Covered Call.
    Asume que Atlas YA POSEE 100 acciones del subyacente.
    Solo modela el leg de la call vendida.
    Max profit = crédito + (strike - precio_compra_acciones)
    """
    call_short:  Optional[Leg]  = None
    stock_basis: float          = 0.0   # precio promedio de las acciones

    @classmethod
    def from_chain(
        cls,
        chain:       OptionsChain,
        short_delta: float = 0.30,
        stock_basis: float = 0.0,
        qty:         int   = 1,
    ) -> "CoveredCall":
        call_short_c = _closest_delta(chain, short_delta, OptionType.CALL)
        cs = Leg(call_short_c, OptionRight.SHORT, qty)

        basis = stock_basis or chain.spot_price
        return cls(
            name        = "Covered Call",
            legs        = [cs],
            description = (f"{chain.symbol} | exp:{chain.expiration} | "
                           f"short δ≈{short_delta} | basis:{basis}"),
            call_short  = cs,
            stock_basis = basis,
        )

    @property
    def max_profit(self) -> float:
        premium = self.net_premium
        cap     = (self.call_short.contract.strike - self.stock_basis) * 100 * self.call_short.qty
        return round(premium + cap, 2)

    @property
    def max_loss(self) -> float:
        # Pérdida teórica máxima = stock va a 0 menos la prima cobrada
        return round(-(self.stock_basis * 100 * self.call_short.qty - abs(self.net_premium)), 2)

    @property
    def breakevens(self) -> list[float]:
        return [round(self.stock_basis - abs(self.net_premium) / 100, 2)]
