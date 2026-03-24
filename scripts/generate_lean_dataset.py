#!/usr/bin/env python
"""generate_lean_dataset.py — CLI para generar dataset de entrenamiento LEAN.

Genera datos OHLCV sintéticos para el universo configurado y reentrena
PatternLab (motifs + TIN) con los trades resultantes.

Uso::

    # Universo completo (ETFs líquidos + índices), 5 años
    python scripts/generate_lean_dataset.py --years 5

    # Universo rápido por defecto (SPY/QQQ/IWM/AAPL/NVDA)
    python scripts/generate_lean_dataset.py --years 2

    # Símbolos custom
    python scripts/generate_lean_dataset.py --symbols SPY QQQ IWM --years 3

    # Solo generar dataset, sin reentrenar PatternLab
    python scripts/generate_lean_dataset.py --no-retrain --years 5

    # Universo completo + retrain
    python scripts/generate_lean_dataset.py --full-universe --years 5
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from pathlib import Path

# Asegurar que el workspace raíz esté en sys.path
_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s — %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("generate_lean_dataset")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Genera dataset LEAN y opcionalmente reentrena PatternLab"
    )
    parser.add_argument(
        "--symbols", nargs="*", default=None,
        metavar="SYM",
        help="Lista de símbolos (default: universo completo o default según --full-universe)"
    )
    parser.add_argument(
        "--years", type=int, default=5,
        help="Años de historia sintética a generar por símbolo (default: 5)"
    )
    parser.add_argument(
        "--full-universe", action="store_true",
        help="Usar universo completo del scanner (ETFs + índices + opcional crypto/futuros)"
    )
    parser.add_argument(
        "--output-dir", type=str, default=None,
        help="Carpeta de salida para CSV/SQLite (default: logs/lean_sim/)"
    )
    parser.add_argument(
        "--no-retrain", action="store_true",
        help="Solo generar dataset, no reentrenar PatternLab"
    )
    parser.add_argument(
        "--seed", type=int, default=42,
        help="Semilla aleatoria para reproducibilidad (default: 42)"
    )
    args = parser.parse_args()

    out_dir = args.output_dir or str(_ROOT / "logs" / "lean_sim")
    Path(out_dir).mkdir(parents=True, exist_ok=True)

    logger.info("=" * 60)
    logger.info("ATLAS-Quant LEAN Dataset Generator")
    logger.info("  years=%d | full_universe=%s | retrain=%s",
                args.years, args.full_universe, not args.no_retrain)
    logger.info("  output_dir=%s", out_dir)
    logger.info("=" * 60)

    # ── Importar LeanSimulator ────────────────────────────────────────────────
    try:
        from atlas_code_quant.backtest.lean_simulator import LeanSimulator, SimConfig
    except ImportError as exc:
        logger.error("No se pudo importar LeanSimulator: %s", exc)
        logger.error("Asegúrate de correr desde la raíz del workspace: cd C:\\ATLAS_PUSH")
        return 1

    # ── Construir simulador ───────────────────────────────────────────────────
    cfg = SimConfig(random_seed=args.seed, out_dir=out_dir)

    sim = LeanSimulator(
        symbols=args.symbols,
        config=cfg,
        use_full_universe=args.full_universe,
        out_dir=out_dir,
    )

    logger.info("Universo: %d símbolos", len(sim.symbols))
    if len(sim.symbols) <= 20:
        logger.info("  %s", ", ".join(sim.symbols))
    else:
        logger.info("  %s ... (+%d más)", ", ".join(sim.symbols[:10]), len(sim.symbols) - 10)

    # ── Generar dataset ───────────────────────────────────────────────────────
    t0 = time.perf_counter()
    try:
        dataset = sim.generate_training_dataset(years=args.years, out_dir=out_dir)
    except Exception as exc:
        logger.error("Error generando dataset: %s", exc, exc_info=True)
        return 1

    elapsed = time.perf_counter() - t0
    logger.info("Dataset generado en %.1fs:", elapsed)
    logger.info("  trades_csv   : %s", dataset.get("trades_csv"))
    logger.info("  features_csv : %s", dataset.get("features_csv"))
    logger.info("  db_path      : %s", dataset.get("db_path"))
    logger.info("  total_trades : %s", f"{dataset.get('total_trades', 0):,}")
    logger.info("  n_features   : %s", f"{dataset.get('n_features', 0):,}")
    logger.info("  sharpe_avg   : %s", dataset.get("sharpe_avg"))

    # ── Reentrenar PatternLab (opcional) ─────────────────────────────────────
    if not args.no_retrain:
        logger.info("")
        logger.info("Reentrenando PatternLab...")
        try:
            from atlas_code_quant.learning.pattern_lab import PatternLabService
            lab = PatternLabService()
            result = lab.retrain_from_lean_simulator(
                symbols=args.symbols,
                years=args.years,
                out_dir=out_dir,
                use_full_universe=args.full_universe,
            )
            logger.info("PatternLab reentrenado:")
            logger.info("  motif fitted : %s", result.get("motif", {}).get("fitted"))
            logger.info("  tin fitted   : %s", result.get("tin", {}).get("fitted"))
        except Exception as exc:
            logger.warning("PatternLab retrain falló (no crítico): %s", exc)

    logger.info("")
    logger.info("COMPLETADO. Dataset disponible en: %s", out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
