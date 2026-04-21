"""MarketState y value types asociados.

Contrato de entrada al :class:`atlas_push.engine.DecisionEngine`. Ver
``docs/atlas_push/ARCHITECTURE.md`` §3.1.

Reglas:

- Todas las dataclasses son inmutables (``frozen=True``).
- Colecciones por orden/posición son ``tuple`` (no ``list``) para
  preservar inmutabilidad estructural.
- No se exponen detalles de broker, websocket, API externa ni de
  ejecución: ``MarketState`` es una vista normalizada del mundo.
- Ampliar estos tipos requiere decisión explícita.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Mapping


@dataclass(frozen=True)
class AccountSnapshot:
    """Snapshot de la cuenta del usuario.

    Campos derivados del contrato de ARCHITECTURE §3.1. Todos los
    montos se expresan en la divisa base de la cuenta (no se modela
    divisa aquí).
    """

    equity: float
    cash: float
    buying_power: float
    realized_pnl_today: float


@dataclass(frozen=True)
class Position:
    """Posición abierta sobre un símbolo.

    Cantidades positivas son largas; negativas, cortas. El motor
    ejecutor fija la convención; :class:`Position` solo la transporta.
    """

    symbol: str
    qty: float
    avg_price: float
    market_value: float
    unrealized_pnl: float


@dataclass(frozen=True)
class Quote:
    """Cotización normalizada para un símbolo.

    Mantenida deliberadamente mínima: bid/ask/last + timestamp. Otros
    datos (profundidad, micro, últimas N velas) viajan en
    ``MarketState.indicators`` para no inflar este tipo.
    """

    symbol: str
    bid: float
    ask: float
    last: float
    as_of: datetime


@dataclass(frozen=True)
class RiskContext:
    """Contexto de riesgo vigente al tomar la decisión.

    - ``halted``: kill-switch global. Si es ``True`` el cerebro debería
      emitir solo vetos o no-op.
    - ``reasons``: motivos legibles del estado actual (p. ej. razones
      del halt, o límites activados). Tupla para inmutabilidad.
    - ``limits``: límites numéricos (max_exposure, max_drawdown, etc.).
      Se modelan como mapping inmutable de nombre → valor numérico.
      Un ``policy_store`` mayor podría proveerlo sin cambios de API.
    """

    halted: bool = False
    reasons: tuple[str, ...] = ()
    limits: Mapping[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class MarketState:
    """Estado normalizado de mercado y cartera.

    Entrada única al :class:`atlas_push.engine.DecisionEngine`. Ver
    ARCHITECTURE §3.1 para la justificación de los campos.

    Atributos:
        as_of: Instante al que corresponde este estado.
        account: Snapshot de cuenta (equity, cash, etc.).
        positions: Posiciones abiertas (tupla inmutable).
        quotes: Cotizaciones por símbolo.
        indicators: Indicadores por símbolo (libre, por estrategia).
        risk_context: Límites y kill-switch vigentes.
        meta: Metadatos libres (origen del estado, etc.).
    """

    as_of: datetime
    account: AccountSnapshot
    positions: tuple[Position, ...] = ()
    quotes: Mapping[str, Quote] = field(default_factory=dict)
    indicators: Mapping[str, Mapping[str, object]] = field(
        default_factory=dict
    )
    risk_context: RiskContext = field(default_factory=RiskContext)
    meta: Mapping[str, object] = field(default_factory=dict)
