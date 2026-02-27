from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Iterable, Optional


@dataclass(frozen=True)
class RetryPolicy:
    """
    Política unificada de reintentos/backoff.

    - Si `fixed_steps` está definido, usa esos delays por intento.
    - Si no, usa exponencial: initial_delay * factor^attempt, cap en max_delay.
    - jitter: 0.0..0.5 (porcentaje del delay) para evitar sincronización.
    """

    max_attempts: int = 5
    initial_delay: float = 5.0
    factor: float = 2.0
    max_delay: float = 300.0
    jitter: float = 0.10
    fixed_steps: Optional[tuple[float, ...]] = None

    @staticmethod
    def fixed(steps: Iterable[float], *, max_attempts: int | None = None, jitter: float = 0.0) -> "RetryPolicy":
        xs = tuple(float(x) for x in steps if float(x) >= 0.0)
        return RetryPolicy(
            max_attempts=int(max_attempts or max(1, len(xs) or 1)),
            fixed_steps=xs,
            jitter=float(jitter),
        )

    def delay_for_attempt(self, attempt_index: int) -> float:
        i = max(0, int(attempt_index))
        if self.fixed_steps:
            base = float(self.fixed_steps[min(i, len(self.fixed_steps) - 1)])
        else:
            base = float(self.initial_delay) * (float(self.factor) ** i)
            base = min(base, float(self.max_delay))
        j = float(self.jitter or 0.0)
        if j > 0:
            base = base + (base * j * random.random())
        return max(0.0, float(base))

