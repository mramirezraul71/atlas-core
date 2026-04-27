"""Factory básica de estrategias (F1)."""
from __future__ import annotations

from typing import Any

try:  # compat con import absoluto/relativo existente en repo
    from atlas_code_quant.strategies.base import BaseStrategy
except Exception:  # pragma: no cover
    from strategies.base import BaseStrategy  # type: ignore


def _optional_registry() -> dict[str, type[BaseStrategy]]:
    registry: dict[str, type[BaseStrategy]] = {}
    try:
        from atlas_code_quant.strategies.ma_cross import MACrossStrategy  # type: ignore

        registry["ma_cross"] = MACrossStrategy
    except Exception:
        try:
            from strategies.ma_cross import MACrossStrategy  # type: ignore

            registry["ma_cross"] = MACrossStrategy
        except Exception:
            pass
    try:
        from atlas_code_quant.strategies.rl_strategy import RLStrategy  # type: ignore

        registry["rl"] = RLStrategy
    except Exception:
        try:
            from strategies.rl_strategy import RLStrategy  # type: ignore

            registry["rl"] = RLStrategy
        except Exception:
            pass
    return registry


class StrategyFactory:
    """Registro conservador para instanciar estrategias por nombre."""

    _registry = _optional_registry()

    @classmethod
    def register(cls, key: str, strategy_cls: type[BaseStrategy]) -> None:
        cls._registry[key] = strategy_cls

    @classmethod
    def create(cls, key: str, **kwargs: Any) -> BaseStrategy:
        if key not in cls._registry:
            raise KeyError(f"Unknown strategy key: {key}")
        return cls._registry[key](**kwargs)
