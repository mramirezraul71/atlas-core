"""Ejecución LEAN vía CLI QuantConnect (`lean backtest`).

No importa módulos prohibidos del test F13 (execution, risk, vision, …).
"""

from __future__ import annotations

import json
import logging
import os
import subprocess
from pathlib import Path
from typing import Any

from atlas_code_quant.lean.config import LeanAdapterConfig
from atlas_code_quant.lean.parser.results import parse_run_artifacts
from atlas_code_quant.lean.runner.launcher import (
    ERROR_PARSE_FAILED,
    ERROR_PROCESS_FAILED,
    ERROR_RESULTS_MISSING,
    ERROR_TIMEOUT,
    StrategyFitnessResult,
)
from atlas_code_quant.strategies.options.intent import StrategyIntent

logger = logging.getLogger("atlas.code_quant.lean.cli")


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


def _find_latest_statistics(project_root: Path) -> Path | None:
    root = project_root / "backtests"
    if not root.is_dir():
        return None
    candidates = sorted(
        root.glob("**/statistics.json"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    return candidates[0] if candidates else None


def _find_latest_orders(project_root: Path) -> Path | None:
    root = project_root / "backtests"
    if not root.is_dir():
        return None
    candidates = sorted(
        root.glob("**/orders.json"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    return candidates[0] if candidates else None


def run_lean_cli_backtest(
    intent: StrategyIntent,
    cfg: LeanAdapterConfig,
) -> StrategyFitnessResult:
    """Lanza ``lean backtest <algorithm>`` en ``project_root`` y parsea artefactos.

    Requiere ``cfg.use_cli``, ``cfg.project_root``, ``cfg.algorithm_name`` y
    ejecutable resuelto (``lean`` en PATH o ``cfg.lean_cli_executable``).
    """
    if not cfg.project_root or not cfg.algorithm_name:
        return StrategyFitnessResult(
            success=False,
            mode="external",
            error_code=ERROR_RESULTS_MISSING,
            error_message="project_root/algorithm_name required for CLI",
            config_snapshot=cfg.to_dict(),
        )
    exe = cfg.lean_cli_executable or "lean"
    root = Path(cfg.project_root)
    if not root.is_dir():
        return StrategyFitnessResult(
            success=False,
            mode="external",
            error_code=ERROR_PROCESS_FAILED,
            error_message=f"LEAN project_root not a directory: {root}",
            config_snapshot=cfg.to_dict(),
        )
    cmd = [exe, "backtest", str(cfg.algorithm_name)]
    if _env_bool("ATLAS_LEAN_CLI_PASS_INTENT", False):
        cmd.extend(["--intent-json", json.dumps(intent.to_dict())])
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(root),
            capture_output=True,
            text=True,
            timeout=cfg.timeout_sec,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return StrategyFitnessResult(
            success=False,
            mode="external",
            error_code=ERROR_TIMEOUT,
            error_message="lean cli timeout",
            config_snapshot=cfg.to_dict(),
        )
    except FileNotFoundError as exc:
        return StrategyFitnessResult(
            success=False,
            mode="external",
            error_code=ERROR_PROCESS_FAILED,
            error_message=str(exc),
            config_snapshot=cfg.to_dict(),
        )
    except Exception as exc:  # noqa: BLE001
        return StrategyFitnessResult(
            success=False,
            mode="external",
            error_code=ERROR_PROCESS_FAILED,
            error_message=str(exc),
            config_snapshot=cfg.to_dict(),
        )

    if proc.returncode != 0:
        tail = (proc.stderr or proc.stdout or "").strip()[:400]
        return StrategyFitnessResult(
            success=False,
            mode="external",
            error_code=ERROR_PROCESS_FAILED,
            error_message=f"lean exit {proc.returncode}: {tail}",
            config_snapshot=cfg.to_dict(),
        )

    stats_path = _find_latest_statistics(root)
    if stats_path is None and cfg.results_dir:
        alt = Path(cfg.results_dir) / "statistics.json"
        if alt.exists():
            stats_path = alt
    if stats_path is None or not stats_path.exists():
        return StrategyFitnessResult(
            success=False,
            mode="external",
            error_code=ERROR_RESULTS_MISSING,
            error_message="statistics.json not found after lean backtest",
            config_snapshot=cfg.to_dict(),
        )

    orders_path = _find_latest_orders(root)
    if orders_path is None and cfg.results_dir:
        alt_o = Path(cfg.results_dir) / "orders.json"
        if alt_o.exists():
            orders_path = alt_o

    try:
        stats_raw: Any = json.loads(stats_path.read_text("utf-8"))
    except Exception as exc:  # noqa: BLE001
        return StrategyFitnessResult(
            success=False,
            mode="external",
            error_code=ERROR_PARSE_FAILED,
            error_message=f"statistics read: {exc}",
            config_snapshot=cfg.to_dict(),
        )
    orders_raw: Any = []
    if orders_path and orders_path.exists():
        try:
            orders_raw = json.loads(orders_path.read_text("utf-8"))
        except Exception:  # noqa: BLE001
            orders_raw = []

    try:
        artifacts = parse_run_artifacts(statistics=stats_raw, orders=orders_raw)
    except Exception as exc:  # noqa: BLE001
        return StrategyFitnessResult(
            success=False,
            mode="external",
            error_code=ERROR_PARSE_FAILED,
            error_message=str(exc),
            config_snapshot=cfg.to_dict(),
        )

    snap = dict(cfg.to_dict())
    snap["lean_cli_invoked"] = True
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
        config_snapshot=snap,
    )


__all__ = ["run_lean_cli_backtest"]
