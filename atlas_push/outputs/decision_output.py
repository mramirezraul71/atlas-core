"""DecisionOutput y value types asociados.

Contrato de salida del :class:`atlas_push.engine.DecisionEngine`. Ver
``docs/atlas_push/ARCHITECTURE.md`` §3.2.

Reglas:

- Todas las dataclasses son inmutables (``frozen=True``).
- Las colecciones son ``tuple`` (no ``list``) para inmutabilidad
  estructural.
- Los vetos de riesgo viajan **dentro** de :class:`DecisionOutput`
  (campo ``vetoes``), no como excepciones.
- Cada :class:`LogicalOrder` lleva ``strategy_id`` y ``reason`` para
  trazabilidad end-to-end.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, Mapping

OrderSide = Literal["buy", "sell"]
OrderKind = Literal["market", "limit"]


@dataclass(frozen=True)
class LogicalOrder:
    """Orden lógica (no necesariamente directamente ejecutable).

    El motor de ejecución externo la materializa. Atlas Push no se
    conecta al broker.

    Atributos:
        symbol: Símbolo objetivo.
        side: ``"buy"`` o ``"sell"`` (:data:`OrderSide`).
        qty: Cantidad objetivo, en unidades del símbolo.
        kind: Tipo de orden (:data:`OrderKind`, ``"market"`` o
            ``"limit"``). Por defecto ``"market"``.
        limit_price: Precio límite si aplica. ``None`` si es market.
        strategy_id: Identificador de la estrategia que propuso esta
            orden. Obligatorio para trazabilidad.
        reason: Texto legible con la razón de la orden. Obligatorio.
        meta: Metadatos libres.
    """

    symbol: str
    side: OrderSide
    qty: float
    strategy_id: str
    reason: str
    kind: OrderKind = "market"
    limit_price: float | None = None
    meta: Mapping[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class TargetWeight:
    """Peso objetivo de cartera para un símbolo.

    El motor de ejecución traduce pesos a órdenes concretas. Atlas
    Push puede expresar decisiones como órdenes, pesos, o ambos.

    Atributos:
        symbol: Símbolo objetivo.
        weight: Peso objetivo en el total de cartera (p. ej. 0.25 =
            25%). No se valida rango aquí: la capa de riesgo o el
            ejecutor pueden rechazarlo.
        strategy_id: Estrategia que propone el peso.
        reason: Texto legible con la razón.
        meta: Metadatos libres.
    """

    symbol: str
    weight: float
    strategy_id: str
    reason: str
    meta: Mapping[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class RiskVeto:
    """Veto emitido por la capa de riesgo.

    Los vetos no son excepciones: son parte de
    :class:`DecisionOutput`. El consumidor (journal, ejecutor,
    dashboard) decide qué hacer con ellos.

    Atributos:
        code: Código estable del veto (p. ej. ``"max_exposure"``,
            ``"halted"``). Pensado para que un consumidor pueda
            reaccionar programáticamente.
        message: Mensaje legible.
        symbol: Símbolo afectado, si aplica. ``None`` si el veto es
            global.
        strategy_id: Estrategia afectada, si el veto aplica a una
            propuesta concreta. ``None`` si es global.
        meta: Metadatos libres.
    """

    code: str
    message: str
    symbol: str | None = None
    strategy_id: str | None = None
    meta: Mapping[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class DecisionOutput:
    """Salida del cerebro de Atlas Push.

    Ver ARCHITECTURE §3.2. Las decisiones pueden expresarse como
    órdenes, pesos objetivo, o ambos.

    Atributos:
        orders: Órdenes lógicas propuestas.
        target_weights: Pesos objetivo de cartera.
        vetoes: Vetos de riesgo aplicados o sugeridos.
        notes: Notas legibles, pensadas para el journal.
        meta: Metadatos libres.
    """

    orders: tuple[LogicalOrder, ...] = ()
    target_weights: tuple[TargetWeight, ...] = ()
    vetoes: tuple[RiskVeto, ...] = ()
    notes: tuple[str, ...] = ()
    meta: Mapping[str, object] = field(default_factory=dict)

    @classmethod
    def empty(cls) -> "DecisionOutput":
        """Salida vacía canónica: sin órdenes, pesos, vetos ni notas.

        Se reserva como el resultado del pass-through de D1. Mismo
        instance-equality no está garantizado (cada llamada construye
        una instancia nueva), pero la igualdad estructural sí
        (``DecisionOutput.empty() == DecisionOutput.empty()``).
        """
        return cls()
