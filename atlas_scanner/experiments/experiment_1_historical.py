from __future__ import annotations

import argparse
import json
import pickle
from dataclasses import asdict, dataclass, replace
from datetime import date
from pathlib import Path
from typing import Any

from atlas_scanner.api.backtest import scan_backtest_offline
from atlas_scanner.backtest.evaluation import EvaluationRequest, evaluate_historical_backtest
from atlas_scanner.config_loader import (
    ComponentWeights,
    OfflineScoringConfig,
    ScanConfig,
    build_default_scan_config_offline,
)
from atlas_scanner.data.openbb_gamma_oi import OpenBBGammaOIProvider
from atlas_scanner.data.openbb_vol_macro import OpenBBVolMacroProvider


@dataclass(frozen=True)
class ExperimentConfig:
    name: str
    start_date: date
    end_date: date
    universe_symbols: tuple[str, ...]
    score_threshold: float
    top_k: int
    scan_filters: dict[str, float | int]
    output_dir: Path


def _parse_date(value: str) -> date:
    return date.fromisoformat(value)


def _build_frozen_scan_config() -> ScanConfig:
    base = build_default_scan_config_offline()
    scoring = replace(
        base.scoring,
        component_weights=ComponentWeights(
            vol=0.20,
            gamma=0.20,
            oi_flow=0.20,
            price=0.20,
            macro=0.20,
        ),
    )
    return replace(base, scoring=scoring)


def _default_experiment_config() -> ExperimentConfig:
    return ExperimentConfig(
        name="experiment_1_scanner_historical",
        start_date=date(2025, 7, 1),
        end_date=date(2025, 12, 31),
        universe_symbols=("SPY", "QQQ", "IWM", "AAPL", "TSLA"),
        score_threshold=0.65,
        top_k=10,
        scan_filters={
            "min_volume_20d": 1_000_000,
            "max_bid_ask_spread": 0.10,
            "min_liquidity_score": 0.30,
        },
        output_dir=Path("artifacts/atlas_scanner/experiment_1"),
    )


def _result_summary(
    *,
    experiment: ExperimentConfig,
    backtest_meta: dict[str, Any],
    evaluation_meta: dict[str, Any],
    evaluation_result: Any,
) -> dict[str, Any]:
    top_symbols = []
    for item in sorted(
        evaluation_result.symbol_summaries,
        key=lambda symbol_summary: symbol_summary.avg_score if symbol_summary.avg_score is not None else -1.0,
        reverse=True,
    )[:5]:
        top_symbols.append(
            {
                "symbol": item.symbol,
                "num_observations": item.num_observations,
                "avg_score": item.avg_score,
                "top_k_frequency": item.top_k_frequency,
                "threshold_pass_frequency": item.threshold_pass_frequency,
            }
        )
    return {
        "experiment": {
            "name": experiment.name,
            "start_date": experiment.start_date.isoformat(),
            "end_date": experiment.end_date.isoformat(),
            "universe_symbols": experiment.universe_symbols,
            "score_threshold": experiment.score_threshold,
            "top_k": experiment.top_k,
            "scan_filters": experiment.scan_filters,
        },
        "backtest_meta": backtest_meta,
        "evaluation_meta": evaluation_meta,
        "top_symbols_by_avg_score": top_symbols,
    }


def run_experiment(config: ExperimentConfig) -> dict[str, Any]:
    scan_config = _build_frozen_scan_config()
    vol_macro_provider = OpenBBVolMacroProvider()
    gamma_oi_provider = OpenBBGammaOIProvider()

    backtest_result = scan_backtest_offline(
        start_date=config.start_date,
        end_date=config.end_date,
        universe_symbols=config.universe_symbols,
        config=scan_config,
        vol_macro_provider=vol_macro_provider,
        gamma_oi_provider=gamma_oi_provider,
        score_threshold=config.score_threshold,
        scan_filters=config.scan_filters,
    )
    evaluation_result = evaluate_historical_backtest(
        EvaluationRequest(
            backtest_result=backtest_result,
            top_k=config.top_k,
            score_threshold=config.score_threshold,
        )
    )

    output_dir = config.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "backtest_result.pkl").open("wb") as fh:
        pickle.dump(backtest_result, fh)
    with (output_dir / "evaluation_result.pkl").open("wb") as fh:
        pickle.dump(evaluation_result, fh)

    summary = _result_summary(
        experiment=config,
        backtest_meta=backtest_result.meta,
        evaluation_meta=evaluation_result.meta,
        evaluation_result=evaluation_result,
    )
    with (output_dir / "summary.json").open("w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2, sort_keys=True, default=str)
    with (output_dir / "evaluation_result.json").open("w", encoding="utf-8") as fh:
        json.dump(asdict(evaluation_result), fh, indent=2, sort_keys=True, default=str)

    return summary


def _build_cli_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run scanner historical experiment 1.")
    default = _default_experiment_config()
    parser.add_argument("--start-date", default=default.start_date.isoformat(), help="YYYY-MM-DD")
    parser.add_argument("--end-date", default=default.end_date.isoformat(), help="YYYY-MM-DD")
    parser.add_argument(
        "--universe",
        default=",".join(default.universe_symbols),
        help="Comma-separated symbols, e.g. SPY,QQQ,IWM,AAPL,TSLA",
    )
    parser.add_argument("--score-threshold", type=float, default=default.score_threshold)
    parser.add_argument("--top-k", type=int, default=default.top_k)
    parser.add_argument("--output-dir", default=str(default.output_dir))
    return parser


def main() -> None:
    parser = _build_cli_parser()
    args = parser.parse_args()
    symbols = tuple(symbol.strip().upper() for symbol in args.universe.split(",") if symbol.strip())
    experiment = replace(
        _default_experiment_config(),
        start_date=_parse_date(args.start_date),
        end_date=_parse_date(args.end_date),
        universe_symbols=symbols,
        score_threshold=float(args.score_threshold),
        top_k=int(args.top_k),
        output_dir=Path(args.output_dir),
    )
    summary = run_experiment(experiment)
    print("Experiment completed")
    print(json.dumps(summary["evaluation_meta"], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

