"""Políticas por modo y límites (defaults seguros, en memoria)."""
from __future__ import annotations

import logging
from copy import deepcopy
from typing import Any

logger = logging.getLogger("atlas.brain.policy_store")

_DEFAULT_POLICIES: dict[str, Any] = {
    "default_mode": "semi",
    "allowed_manual_actions": ["observe", "status", "speak", "notify", "confirm", "reject"],
    "quant_block_on_high_risk": True,
    "body_block_on_critical": True,
    "system_health_block_nonessential_on_degraded": True,
}


class PolicyStore:
    """Wrapper simple sobre dict con defaults seguros."""

    def __init__(self, initial: dict[str, Any] | None = None) -> None:
        self._data = deepcopy(_DEFAULT_POLICIES)
        if initial:
            self._data.update(initial)
        logger.debug("PolicyStore inicializado: default_mode=%s", self._data.get("default_mode"))

    def get(self, key: str, default: Any = None) -> Any:
        return self._data.get(key, default)

    def as_dict(self) -> dict[str, Any]:
        return deepcopy(self._data)

    def update(self, patch: dict[str, Any]) -> None:
        self._data.update(patch)
