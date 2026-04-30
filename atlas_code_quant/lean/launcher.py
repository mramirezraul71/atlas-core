"""Launcher stub para LEAN (F1)."""
from __future__ import annotations

from dataclasses import dataclass, field

from .config import LeanConfig


@dataclass(slots=True)
class LeanRunResult:
    ok: bool
    mode: str
    command: list[str] = field(default_factory=list)
    message: str = "LEAN no ejecutado (F1 scaffold)."


class LeanLauncher:
    """Prepara ejecución futura sin lanzar procesos reales en F1."""

    def __init__(self, config: LeanConfig | None = None) -> None:
        self.config = config or LeanConfig()

    def plan_backtest(self, algorithm_name: str) -> LeanRunResult:
        cmd = ["lean", "backtest", algorithm_name]
        if not self.config.enabled:
            return LeanRunResult(
                ok=False,
                mode=self.config.mode,
                command=cmd,
                message="ATLAS_LEAN_ENABLED=false; ejecución bloqueada por seguridad.",
            )
        return LeanRunResult(ok=True, mode=self.config.mode, command=cmd, message="planned")
