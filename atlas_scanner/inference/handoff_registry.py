from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

from atlas_scanner.contracts import RadarDecisionHandoff


class HandoffConsumer(Protocol):
    def consume(self, handoff: RadarDecisionHandoff) -> None:
        ...


@dataclass
class NoOpHandoffConsumer:
    consumed: int = 0

    def consume(self, handoff: RadarDecisionHandoff) -> None:
        _ = handoff
        self.consumed += 1


@dataclass
class InMemoryHandoffConsumer:
    handoffs: list[RadarDecisionHandoff] = field(default_factory=list)

    def consume(self, handoff: RadarDecisionHandoff) -> None:
        self.handoffs.append(handoff)


@dataclass
class HandoffRegistry:
    _consumers: dict[str, HandoffConsumer] = field(default_factory=dict)

    def register(self, name: str, consumer: HandoffConsumer) -> None:
        self._consumers[name] = consumer

    def unregister(self, name: str) -> None:
        self._consumers.pop(name, None)

    def publish(self, handoff: RadarDecisionHandoff) -> tuple[str, ...]:
        delivered: list[str] = []
        for name, consumer in self._consumers.items():
            consumer.consume(handoff)
            delivered.append(name)
        return tuple(delivered)

    def consumers(self) -> tuple[str, ...]:
        return tuple(self._consumers.keys())
