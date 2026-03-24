#!/usr/bin/env python
"""Runner CLI para el análisis diario de AtlasLearningBrain.

Uso:
    # Análisis de hoy:
    python scripts/run_daily_learning.py

    # Análisis de una fecha específica:
    python scripts/run_daily_learning.py --date 2024-03-15

    # Solo chequeo de readiness:
    python scripts/run_daily_learning.py --readiness-only

    # Con ventana personalizada (últimos N días):
    python scripts/run_daily_learning.py --lookback 180

    # Aplicar políticas automáticamente:
    python scripts/run_daily_learning.py --apply-policies

    # Exportar reporte a JSON:
    python scripts/run_daily_learning.py --output report.json
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
from datetime import date, datetime
from pathlib import Path

# Añadir raíz del paquete al path
_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s — %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("atlas.daily_learning")


def _load_brain():
    """Carga AtlasLearningBrain con configuración desde settings."""
    from atlas_code_quant.config.settings import settings
    from atlas_code_quant.learning.atlas_learning_brain import AtlasLearningBrain

    policy_path = _ROOT / "data" / "learning_policies.json"
    ml_model_path = _ROOT / "data" / "ml_signal_ranker.pkl"

    readiness_cfg = {
        "min_n_trades":     settings.readiness_min_n_trades,
        "min_months":       settings.readiness_min_months,
        "min_profit_factor": settings.readiness_min_profit_factor,
        "min_calmar":       settings.readiness_min_calmar,
        "max_dd_pct":       settings.readiness_max_dd_pct,
        "min_expectancy_r": settings.readiness_min_expectancy_r,
        "min_stability":    settings.readiness_min_stability,
    }

    brain = AtlasLearningBrain(
        policy_path=policy_path,
        ml_model_path=ml_model_path,
        readiness_cfg=readiness_cfg,
        ml_weight=settings.learning_brain_ml_weight,
        stats_weight=settings.learning_brain_stats_weight,
        retrain_every_n=settings.learning_brain_retrain_every_n,
    )

    # Cargar historial desde DB si está disponible
    n_loaded = brain.load_trades_from_db()
    logger.info("Historial cargado: %d trades desde DB", n_loaded)

    return brain


def _print_metrics(label: str, m) -> None:
    print(f"\n  {label}:")
    print(f"    n={m.n_trades}, WR={m.winrate:.1%}, PF={m.profit_factor:.2f}, "
          f"E[R]={m.expectancy_r:.3f}R, MaxDD={m.max_drawdown_r:.2f}R, "
          f"Calmar={m.calmar_ratio:.2f}")


def run_daily_analysis(
    analysis_date: date,
    lookback_days: int,
    apply_policies: bool,
    output_path: Path | None,
) -> None:
    brain = _load_brain()

    print(f"\n{'='*60}")
    print(f"  ATLAS Learning Brain — Análisis Diario {analysis_date.isoformat()}")
    print(f"  Lookback: {lookback_days} días")
    print(f"{'='*60}")

    report = brain.run_daily_analysis(analysis_date, lookback_days=lookback_days)

    print(f"\n📊 MÉTRICAS GLOBALES (n={report.n_trades_analyzed} trades):")
    _print_metrics("Global", report.global_metrics)

    if report.by_setup:
        print(f"\n📋 POR SETUP:")
        for setup, m in sorted(report.by_setup.items()):
            if m.n_trades >= 5:
                _print_metrics(setup, m)

    if report.by_regime:
        print(f"\n🌡️  POR RÉGIMEN:")
        for regime, m in sorted(report.by_regime.items()):
            if m.n_trades >= 5:
                _print_metrics(regime, m)

    if report.success_patterns:
        print(f"\n✅ PATRONES DE ÉXITO:")
        for p in report.success_patterns:
            print(f"    • {p}")

    if report.error_patterns:
        print(f"\n⚠️  PATRONES DE ERROR:")
        for p in report.error_patterns:
            print(f"    • {p}")

    print(f"\n📐 ESTABILIDAD TEMPORAL: {report.stability_score:.3f} "
          f"({'OK' if report.stability_score >= 0.70 else 'DEGRADADO'})")

    if report.proposed_policies:
        print(f"\n🔧 POLÍTICAS PROPUESTAS ({len(report.proposed_policies)}):")
        for action in report.proposed_policies:
            print(f"    • [{action.action.upper()}] {action.setup_type}: {action.reason}")

        if apply_policies:
            snapshot = brain.update_policies(report)
            print(f"\n  ✓ Políticas aplicadas — disabled={snapshot.disabled_setups}, "
                  f"multipliers={snapshot.size_multipliers}")
        else:
            print(f"\n  (Para aplicar: usar --apply-policies)")

    if report.warnings:
        print(f"\n⚠️  ADVERTENCIAS:")
        for w in report.warnings:
            print(f"    • {w}")

    if output_path:
        _export_report(report, output_path)

    print()


def run_readiness_check() -> None:
    brain = _load_brain()

    print(f"\n{'='*60}")
    print(f"  ATLAS Learning Brain — Readiness Check")
    print(f"{'='*60}")

    report = brain.is_system_ready_for_live()

    status_icon = "✅ LISTO" if report.ready else "❌ NO LISTO"
    print(f"\n{status_icon} — {report.summary}")

    print(f"\n  Criterios evaluados ({report.n_trades_evaluated} trades):")
    for c in report.criteria:
        icon = "  ✓" if c.passed else "  ✗"
        print(f"{icon} {c.name}: {c.value}{c.unit} (req ≥ {c.threshold}{c.unit})")

    if report.next_step:
        print(f"\n  → Siguiente paso: {report.next_step}")

    print()


def _export_report(report, path: Path) -> None:
    """Exporta el reporte como JSON."""
    def _m_to_dict(m):
        return {
            "n_trades": m.n_trades,
            "winrate": m.winrate,
            "profit_factor": m.profit_factor,
            "expectancy_r": m.expectancy_r,
            "max_drawdown_r": m.max_drawdown_r,
            "calmar_ratio": m.calmar_ratio,
            "sharpe_r": m.sharpe_r,
        }

    data = {
        "analysis_date": report.analysis_date.isoformat(),
        "n_trades_analyzed": report.n_trades_analyzed,
        "stability_score": report.stability_score,
        "global_metrics": _m_to_dict(report.global_metrics),
        "by_setup": {k: _m_to_dict(v) for k, v in report.by_setup.items()},
        "by_regime": {k: _m_to_dict(v) for k, v in report.by_regime.items()},
        "by_asset_class": {k: _m_to_dict(v) for k, v in report.by_asset_class.items()},
        "success_patterns": report.success_patterns,
        "error_patterns": report.error_patterns,
        "proposed_policies": [
            {
                "setup_type": a.setup_type,
                "action": a.action,
                "reason": a.reason,
                "size_multiplier": a.size_multiplier,
                "confidence": a.confidence,
            }
            for a in report.proposed_policies
        ],
        "warnings": report.warnings,
        "generated_at": report.generated_at.isoformat(),
    }

    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    logger.info("Reporte exportado a %s", path)
    print(f"  Reporte exportado: {path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="AtlasLearningBrain — runner de análisis diario"
    )
    parser.add_argument(
        "--date", type=str, default=None,
        help="Fecha del análisis (YYYY-MM-DD). Default: hoy."
    )
    parser.add_argument(
        "--lookback", type=int, default=90,
        help="Días hacia atrás a analizar (default 90)."
    )
    parser.add_argument(
        "--readiness-only", action="store_true",
        help="Solo ejecutar chequeo de readiness para live."
    )
    parser.add_argument(
        "--apply-policies", action="store_true",
        help="Aplicar automáticamente las políticas propuestas."
    )
    parser.add_argument(
        "--output", type=str, default=None,
        help="Ruta para exportar reporte JSON."
    )

    args = parser.parse_args()

    if args.readiness_only:
        run_readiness_check()
        return

    analysis_date = date.today()
    if args.date:
        try:
            analysis_date = date.fromisoformat(args.date)
        except ValueError:
            logger.error("Fecha inválida: %s (usar YYYY-MM-DD)", args.date)
            sys.exit(1)

    output_path = Path(args.output) if args.output else None

    run_daily_analysis(
        analysis_date=analysis_date,
        lookback_days=args.lookback,
        apply_policies=args.apply_policies,
        output_path=output_path,
    )

    # Siempre mostrar readiness al final
    run_readiness_check()


if __name__ == "__main__":
    main()
