"""DecisionEngine — scaffold pass-through (Paso D).

Ver ``docs/atlas_push/ARCHITECTURE.md`` §3.3.

En el paso D el motor es deliberadamente no-op:
``DecisionEngine(...).decide(state)`` devuelve
:meth:`DecisionOutput.empty` para cualquier entrada. El objetivo es
fijar el contrato tipado (entrada → salida, firma de Protocols) sin
introducir lógica de trading. Estrategias y capa de riesgo reales se
introducen en pasos posteriores.

Reglas que este scaffold respeta:

- Sin estado de negocio (:class:`DecisionEngine` no guarda estado
  entre llamadas; los únicos campos son la configuración de
  estrategias y risk manager).
- Pureza: mismo :class:`MarketState` → misma salida estructural.
- Composable: admite una secuencia de :class:`Strategy` y un
  :class:`RiskManager`, todos opcionales en D.
- No lanza excepciones por reglas de negocio. Los vetos viajarán en
  :class:`DecisionOutput.vetoes` cuando la capa de riesgo real exista.
- No importa símbolos del brain core mayor ni de ``atlas_push.intents``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

from atlas_push.outputs import DecisionOutput
from atlas_push.state import MarketState


@dataclass(frozen=True)
class StrategyProposal:
    """Placeholder tipado de propuesta de estrategia.

    Dataclass vacía e inmutable. El contenido concreto (órdenes,
    pesos, notas) se definirá cuando aparezcan estrategias reales.
    En D solo existe la forma para fijar la firma de
    :meth:`Strategy.propose`.
    """


@runtime_checkable
class Strategy(Protocol):
    """Generador de propuestas de trading.

    Implementaciones viven en ``atlas_push/strategies/`` o en
    paquetes externos (p. ej. ``atlas_code_quant``) sin herencia
    obligatoria.
    """

    def propose(self, state: MarketState) -> StrategyProposal:
        """Genera una propuesta a partir del estado actual."""
        ...


@runtime_checkable
class RiskManager(Protocol):
    """Capa central de gestión de riesgo.

    Recibe ``(state, draft)`` y devuelve un :class:`DecisionOutput`
    final, aplicando recortes o añadiendo vetos. No lanza excepciones
    por reglas de negocio; los vetos viajan en el output.
    """

    def apply(
        self, state: MarketState, draft: DecisionOutput
    ) -> DecisionOutput:
        """Aplica la capa de riesgo al borrador de decisión."""
        ...


@dataclass(frozen=True)
class DecisionEngine:
    """Motor de decisión de Atlas Push.

    Scaffold pass-through en el paso D: ``decide(state)`` devuelve
    :meth:`DecisionOutput.empty` para cualquier entrada. Las
    estrategias y el risk manager se almacenan para fijar el
    constructor público, pero no se ejecutan en D.

    Atributos:
        strategies: Secuencia inmutable de estrategias. Tupla vacía
            por defecto: el motor no ejecuta estrategias en D.
        risk: :class:`RiskManager` opcional. ``None`` por defecto:
            sin capa de riesgo activa en D.
    """

    strategies: tuple[Strategy, ...] = field(default_factory=tuple)
    risk: RiskManager | None = None

    def decide(self, state: MarketState) -> DecisionOutput:
        """Decide a partir de un :class:`MarketState`.

        En el paso D esta función es un pass-through puro: ignora el
        estado y devuelve :meth:`DecisionOutput.empty`. Se mantiene
        la firma definitiva para que los consumidores puedan
        empezar a integrarse desde ya.
        """
        # Paso D: scaffold no-op. No se invocan estrategias ni risk.
        # Cualquier lógica real llegará con estrategias en pasos
        # posteriores, respetando esta misma firma.
        _ = state  # marca de uso para linters sin consumir el estado
        return DecisionOutput.empty()
