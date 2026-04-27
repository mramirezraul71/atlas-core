"""Launcher LEAN external mode (F4).

Diseño:
- ``LeanLauncher.plan_backtest`` produce un comando reproducible y un descriptor
  de plan, **sin lanzar el subprocess** salvo opt-in explícito.
- ``LeanLauncher.run_backtest`` solo ejecuta si ``ATLAS_LEAN_ENABLED=true`` y
  ``mode='external'``. Otros modos retornan resultado bloqueado con motivo.
- Resultado siempre auditable: incluye comando, modo, latencia, exit_code.
"""
from __future__ import annotations

import shlex
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .config import LeanConfig


@dataclass(slots=True)
class LeanRunResult:
    """Resultado auditable de una corrida LEAN (real o planificada)."""

    ok: bool
    mode: str
    command: list[str] = field(default_factory=list)
    message: str = "LEAN no ejecutado (F4 scaffold)."
    exit_code: int | None = None
    latency_sec: float = 0.0
    stdout_tail: str = ""
    stderr_tail: str = ""


@dataclass(slots=True)
class LeanBacktestParams:
    algorithm: str
    symbol: str = "SPY"
    start: str = "2025-01-01"
    end: str = "2025-04-25"
    cash: int = 25_000
    project_dir: str = ""
    output_dir: str = ""

    def to_cli_args(self) -> list[str]:
        args: list[str] = []
        if self.project_dir:
            args += ["--project", self.project_dir]
        if self.output_dir:
            args += ["--output", self.output_dir]
        return args


class LeanLauncher:
    """Wrapper LEAN external mode con dry-run obligatorio por defecto."""

    def __init__(self, config: LeanConfig | None = None) -> None:
        self.config = config or LeanConfig.from_env()

    # ── planificación ──────────────────────────────────────────────────────
    def plan_backtest(self, algorithm_or_params: str | LeanBacktestParams) -> LeanRunResult:
        """Devuelve el comando que se ejecutaría, sin lanzar nada."""
        params = (
            algorithm_or_params
            if isinstance(algorithm_or_params, LeanBacktestParams)
            else LeanBacktestParams(algorithm=algorithm_or_params)
        )
        cmd = [self.config.binary, "backtest", params.algorithm] + params.to_cli_args()
        if not self.config.enabled:
            return LeanRunResult(
                ok=False,
                mode=self.config.mode,
                command=cmd,
                message="ATLAS_LEAN_ENABLED=false; planificado sin ejecutar.",
            )
        return LeanRunResult(
            ok=True,
            mode=self.config.mode,
            command=cmd,
            message="planned",
        )

    # ── ejecución ──────────────────────────────────────────────────────────
    def run_backtest(self, params: LeanBacktestParams) -> LeanRunResult:
        """Ejecuta LEAN external real solo si está habilitado y modo external."""
        plan = self.plan_backtest(params)
        if not plan.ok:
            return plan
        if not self.config.is_external():
            plan.ok = False
            plan.message = (
                f"mode={self.config.mode} no soportado por F4 (solo 'external')."
            )
            return plan

        t0 = time.perf_counter()
        try:
            proc = subprocess.run(
                plan.command,
                capture_output=True,
                text=True,
                timeout=self.config.timeout_sec,
                check=False,
            )
            latency = time.perf_counter() - t0
            return LeanRunResult(
                ok=proc.returncode == 0,
                mode=self.config.mode,
                command=plan.command,
                message="executed" if proc.returncode == 0 else "lean_returned_nonzero",
                exit_code=proc.returncode,
                latency_sec=round(latency, 3),
                stdout_tail=(proc.stdout or "")[-2_000:],
                stderr_tail=(proc.stderr or "")[-2_000:],
            )
        except FileNotFoundError as e:
            return LeanRunResult(
                ok=False,
                mode=self.config.mode,
                command=plan.command,
                message=f"lean_binary_not_found: {e}",
                latency_sec=round(time.perf_counter() - t0, 3),
            )
        except subprocess.TimeoutExpired:
            return LeanRunResult(
                ok=False,
                mode=self.config.mode,
                command=plan.command,
                message=f"timeout_after_{self.config.timeout_sec}s",
                latency_sec=round(time.perf_counter() - t0, 3),
            )

    # ── helpers ────────────────────────────────────────────────────────────
    @staticmethod
    def render_command(result: LeanRunResult) -> str:
        return " ".join(shlex.quote(a) for a in result.command)

    def output_paths(self, params: LeanBacktestParams) -> dict[str, Any]:
        """Rutas estándar de artefactos LEAN para un run dado."""
        out = Path(params.output_dir or self.config.output_dir or "lean-out")
        return {
            "statistics": out / "statistics.json",
            "orders": out / "orders.json",
            "equity": out / "equity.csv",
            "log": out / "lean-log.txt",
        }
