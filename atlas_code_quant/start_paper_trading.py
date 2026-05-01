#!/usr/bin/env python
"""Arranque paper trading + preflight (sesión validación).

Uso (desde raíz del repo, p. ej. C:\\ATLAS_PUSH)::

    python -m atlas_code_quant.start_paper_trading
    python -m atlas_code_quant.start_paper_trading --no-core

Variables: ajusta PYTHONPATH si hace falta; imports relativos a ``atlas_code_quant``.
"""
from __future__ import annotations

import argparse
import logging
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path

_QUANT_ROOT = Path(__file__).resolve().parent
_REPO_ROOT = _QUANT_ROOT.parent
if str(_QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(_QUANT_ROOT))

from config.paper_trading_config import PAPER_TRADING_CONFIG  # noqa: E402


def _setup_logging() -> Path:
    log_dir = _QUANT_ROOT / "logs" / "paper_trading_2026_04_15"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / f"paper_trading_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=[
            logging.FileHandler(log_file, encoding="utf-8"),
            logging.StreamHandler(),
        ],
    )
    return log_file


def preflight_check() -> bool:
    logger = logging.getLogger("atlas.paper_trading")
    logger.info("=" * 80)
    logger.info("ATLAS CODE QUANT — PAPER TRADING PREFLIGHT")
    logger.info("=" * 80)

    try:
        from config.settings import settings

        logger.info("paper_trading=%s tradier_paper_token_configured=%s",
                    settings.paper_trading,
                    bool((settings.tradier_paper_token or "").strip()))

        from operations.operation_center import OperationCenter

        op_center = OperationCenter()
        ok, reason = op_center._check_market_hours("SPY", "paper")
        logger.info("Market hours (SPY, paper): ok=%s reason=%s", ok, reason)

        from execution.tradier_execution import build_tradier_order_payload

        _ = build_tradier_order_payload  # importable
        logger.info("tradier_execution (build_tradier_order_payload): OK")

        import journal.service as journal_service_mod

        assert journal_service_mod is not None
        logger.info("journal.service: OK")

        from config.paper_trading_config import DAILY_METRICS_TEMPLATE

        _ = DAILY_METRICS_TEMPLATE
        logger.info("paper_trading_config: OK (duration_days=%s)",
                    PAPER_TRADING_CONFIG.get("duration_days"))

        logger.info("=" * 80)
        logger.info("PREFLIGHT: ALL SYSTEMS GREEN")
        logger.info("=" * 80)
        return True
    except Exception as exc:
        logger.exception("PREFLIGHT FAILED: %s", exc)
        return False


def _launch_quant_core() -> None:
    logger = logging.getLogger("atlas.paper_trading")
    env = os.environ.copy()
    sep = os.pathsep
    extra = f"{_QUANT_ROOT}{sep}{_REPO_ROOT}"
    env["PYTHONPATH"] = f"{extra}{sep}{env['PYTHONPATH']}" if env.get("PYTHONPATH") else extra
    cmd = [sys.executable, "-m", "atlas_code_quant.atlas_quant_core", "--mode", "paper"]
    logger.info("Lanzando: %s (cwd=%s)", " ".join(cmd), _REPO_ROOT)
    subprocess.Popen(cmd, cwd=str(_REPO_ROOT), env=env)


def main() -> None:
    parser = argparse.ArgumentParser(description="Paper trading preflight / start")
    parser.add_argument(
        "--no-core",
        action="store_true",
        help="No lanzar atlas_quant_core (solo preflight)",
    )
    args = parser.parse_args()

    log_file = _setup_logging()
    logger = logging.getLogger("atlas.paper_trading")
    logger.info("Log file: %s", log_file)

    if not preflight_check():
        sys.exit(1)

    logger.info("")
    logger.info("Sesión: %s → %s (%s días)",
                PAPER_TRADING_CONFIG.get("start_date"),
                PAPER_TRADING_CONFIG.get("expected_end_date"),
                PAPER_TRADING_CONFIG.get("duration_days"))
    logger.info("Métricas objetivo: 0 fuera de horario, 0 broker_order_id vacío, 0 phantoms")
    logger.info("")

    if args.no_core:
        logger.info("--no-core: no se inicia atlas_quant_core.")
        return

    _launch_quant_core()
    logger.info("Proceso core arrancado en segundo plano (Popen). Revisa logs del live loop.")


if __name__ == "__main__":
    main()
