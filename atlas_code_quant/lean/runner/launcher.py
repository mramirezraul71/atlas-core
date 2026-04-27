"""Atlas Code Quant — LEAN runner / launcher (F13, MVP).

Wrapper externo de LEAN. F13 implementa la API canónica de fitness:

    run_backtest_for_strategy_intent(intent) -> StrategyFitnessResult

Modos soportados:

    * ``mock`` (default): no lanza subprocess, devuelve resultado
      determinista derivado del propio intent. Útil en CI/sandbox.
    * ``external``: lanza el binario LEAN como subprocess y luego
      lee artefactos vía :mod:`atlas_code_quant.lean.parser.results`.
      Cualquier fallo se traduce en ``StrategyFitnessResult`` con
      ``success=False`` y código de error honesto.
    * ``disabled``: ATLAS_LEAN_ENABLED=False (default) → resultado
      fitness vacío con error ``LEAN_DISABLED``.

Reglas duras:

    * NO ejecuta órdenes reales. NO toca Tradier. NO toca execution.
    * Subprocess SOLO se lanza si la flag está ON y hay binario.
    * Timeout duro y manejo de errores: nada se cuelga arriba.
    * Determinismo en modo mock para tests.
"""

from __future__ import annotations

import hashlib
import json
import logging
import os
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from atlas_code_quant.lean.config import LeanAdapterConfig, load_config_from_env
from atlas_code_quant.lean.parser.results import (
    LeanRunArtifacts,
    parse_run_artifacts,
)
from atlas_code_quant.strategies.options.intent import StrategyIntent


logger = logging.getLogger("atlas.code_quant.lean.runner")


__all__ = [
    "StrategyFitnessResult",
    "run_backtest_for_strategy_intent",
    "ERROR_DISABLED",
    "ERROR_TIMEOUT",
    "ERROR_PROCESS_FAILED",
    "ERROR_PARSE_FAILED",
    "ERROR_INVALID_INTENT",
]


ERROR_DISABLED = "LEAN_DISABLED"
ERROR_TIMEOUT = "LEAN_TIMEOUT"
ERROR_PROCESS_FAILED = "LEAN_PROCESS_FAILED"
ERROR_PARSE_FAILED = "LEAN_PARSE_FAILED"
ERROR_INVALID_INTENT = "LEAN_INVALID_INTENT"
ERROR_RESULTS_MISSING = "LEAN_RESULTS_MISSING"


@dataclass(frozen=True)
class StrategyFitnessResult:
    """Resultado canónico del backtest de un :class:`StrategyIntent`."""

    success: bool
    mode: str  # "mock" | "external" | "disabled"
    sharpe: float = 0.0
    win_rate: float = 0.0
    max_drawdown: float = 0.0
    total_return: float = 0.0
    expectancy: float = 0.0
    num_orders: int = 0
    error_code: str | None = None
    error_message: str | None = None
    artifacts: LeanRunArtifacts | None = field(default=None, repr=False, compare=False)
    config_snapshot: dict[str, Any] = field(default_factory=dict, repr=False, compare=False)

    def to_dict(self) -> dict[str, Any]:
        return {
            "success": self.success,
            "mode": self.mode,
            "sharpe": self.sharpe,
            "win_rate": self.win_rate,
            "max_drawdown": self.max_drawdown,
            "total_return": self.total_return,
            "expectancy": self.expectancy,
            "num_orders": self.num_orders,
            "error_code": self.error_code,
            "error_message": self.error_message,
            "config_snapshot": dict(self.config_snapshot),
        }


# ---------------------------------------------------------------------------
# Mock determinista
# ---------------------------------------------------------------------------


def _mock_artifacts_for_intent(intent: StrategyIntent) -> LeanRunArtifacts:
    """Genera un LeanRunArtifacts determinista a partir del intent.

    Determinismo basado en hash estable de campos clave: símbolo,
    strategy_type, número de legs y DTE de la primera leg.
    """
    digest_input = (
        f"{intent.opportunity.symbol}|{intent.strategy_type}|"
        f"{intent.num_legs}|{intent.legs[0].expiry_rel_dte if intent.legs else 0}"
    )
    digest = hashlib.sha256(digest_input.encode("utf-8")).digest()
    # Tomamos bytes específicos para derivar métricas estables en
    # rangos plausibles. F13 mock NO pretende ser realista, solo
    # estable y útil para que F14 pueda ordenar y filtrar.
    sharpe = round(((digest[0] / 255.0) * 4.0) - 1.0, 3)  # [-1, 3]
    win_rate = round((digest[1] / 255.0) * 100.0, 2)  # [0, 100]
    max_dd = -round((digest[2] / 255.0) * 30.0, 2)  # [-30, 0]
    total_ret = round(((digest[3] / 255.0) * 60.0) - 20.0, 2)  # [-20, 40]
    expectancy = round(((digest[4] / 255.0) * 2.0) - 0.5, 3)  # [-0.5, 1.5]
    num_orders = int(digest[5] % 50) + intent.num_legs
    return LeanRunArtifacts(
        sharpe=sharpe,
        win_rate=win_rate,
        max_drawdown=max_dd,
        total_return=total_ret,
        expectancy=expectancy,
        num_orders=num_orders,
        raw_statistics={"_mock": True},
        raw_orders=[],
        parse_warnings=("mock_deterministic",),
    )


# ---------------------------------------------------------------------------
# External (subprocess + filesystem)
# ---------------------------------------------------------------------------


def _read_artifact_files(results_dir: str) -> tuple[Any, Any, list[str]]:
    """Lee statistics.json y orders.json de un directorio.

    Devuelve ``(stats_payload, orders_payload, warnings)``.
    """
    warnings: list[str] = []
    base = Path(results_dir)
    stats_path = base / "statistics.json"
    orders_path = base / "orders.json"
    stats: Any = None
    orders: Any = None
    if stats_path.exists():
        try:
            stats = json.loads(stats_path.read_text("utf-8"))
        except Exception:  # noqa: BLE001
            warnings.append("statistics_json_invalid")
    else:
        warnings.append("statistics_json_missing")
    if orders_path.exists():
        try:
            orders = json.loads(orders_path.read_text("utf-8"))
        except Exception:  # noqa: BLE001
            warnings.append("orders_json_invalid")
    else:
        warnings.append("orders_json_missing")
    return stats, orders, warnings


def _run_external(
    intent: StrategyIntent, cfg: LeanAdapterConfig
) -> StrategyFitnessResult:
    if not cfg.bin_path or not cfg.results_dir:
        return _failed("external", ERROR_RESULTS_MISSING, "bin_path/results_dir missing", cfg)
    try:
        # F13 NO define el contrato CLI de LEAN definitivamente; el
        # adapter pasa un argumento JSON con los campos clave del
        # intent y deja al wrapper externo traducir a LEAN. Si LEAN
        # cambia de contrato, sólo se toca este punto.
        cmd = [
            cfg.bin_path,
            "--intent-json",
            json.dumps(intent.to_dict()),
            "--results-dir",
            cfg.results_dir,
        ]
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=cfg.timeout_sec,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return _failed("external", ERROR_TIMEOUT, "subprocess timeout", cfg)
    except FileNotFoundError as exc:
        return _failed("external", ERROR_PROCESS_FAILED, f"binary not found: {exc}", cfg)
    except Exception as exc:  # noqa: BLE001
        return _failed("external", ERROR_PROCESS_FAILED, f"subprocess error: {exc}", cfg)

    if proc.returncode != 0:
        return _failed(
            "external",
            ERROR_PROCESS_FAILED,
            f"non-zero exit {proc.returncode}: {proc.stderr.strip()[:200]}",
            cfg,
        )

    try:
        stats, orders, warnings = _read_artifact_files(cfg.results_dir)
        artifacts = parse_run_artifacts(statistics=stats, orders=orders)
    except Exception as exc:  # noqa: BLE001
        return _failed("external", ERROR_PARSE_FAILED, f"parse error: {exc}", cfg)

    return StrategyFitnessResult(
        success=True,
        mode="external",
        sharpe=artifacts.sharpe,
        win_rate=artifacts.win_rate,
        max_drawdown=artifacts.max_drawdown,
        total_return=artifacts.total_return,
        expectancy=artifacts.expectancy,
        num_orders=artifacts.num_orders,
        artifacts=artifacts,
        config_snapshot=cfg.to_dict(),
    )


def _failed(
    mode: str,
    code: str,
    message: str,
    cfg: LeanAdapterConfig | None,
) -> StrategyFitnessResult:
    return StrategyFitnessResult(
        success=False,
        mode=mode,
        error_code=code,
        error_message=message,
        config_snapshot=cfg.to_dict() if cfg is not None else {},
    )


# ---------------------------------------------------------------------------
# API pública
# ---------------------------------------------------------------------------


def run_backtest_for_strategy_intent(
    intent: StrategyIntent | None,
    *,
    config: LeanAdapterConfig | None = None,
) -> StrategyFitnessResult:
    """Punto de entrada canónico del adapter LEAN F13.

    * Defensivo: nunca lanza.
    * Si el intent es None / inválido → ``success=False``,
      ``error_code=LEAN_INVALID_INTENT``.
    * Si la flag está OFF → modo ``disabled``.
    * Si modo ``mock`` → resultado determinista a partir del intent.
    * Si modo ``external`` → subprocess + parser.
    """
    cfg = config if config is not None else load_config_from_env()
    if not isinstance(intent, StrategyIntent):
        return _failed("disabled" if not cfg.enabled else cfg.mode,
                       ERROR_INVALID_INTENT, "intent is not a StrategyIntent", cfg)
    if not intent.legs:
        return _failed("disabled" if not cfg.enabled else cfg.mode,
                       ERROR_INVALID_INTENT, "intent has no legs", cfg)

    if not cfg.enabled:
        return _failed("disabled", ERROR_DISABLED, "ATLAS_LEAN_ENABLED is False", cfg)

    if cfg.is_mock:
        try:
            artifacts = _mock_artifacts_for_intent(intent)
        except Exception as exc:  # noqa: BLE001
            return _failed("mock", ERROR_PARSE_FAILED, f"mock failure: {exc}", cfg)
        return StrategyFitnessResult(
            success=True,
            mode="mock",
            sharpe=artifacts.sharpe,
            win_rate=artifacts.win_rate,
            max_drawdown=artifacts.max_drawdown,
            total_return=artifacts.total_return,
            expectancy=artifacts.expectancy,
            num_orders=artifacts.num_orders,
            artifacts=artifacts,
            config_snapshot=cfg.to_dict(),
        )

    if cfg.is_external:
        return _run_external(intent, cfg)

    # enabled pero sin binario válido → fallo honesto.
    return _failed(
        "external", ERROR_PROCESS_FAILED, "external mode without bin_path", cfg
    )
