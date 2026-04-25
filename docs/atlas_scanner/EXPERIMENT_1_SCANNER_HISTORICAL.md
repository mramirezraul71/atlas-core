# Experiment 1 - Scanner Historical Stability

## Purpose

Validate scanner stability, data coverage, and component consistency over a bounded historical window.

This is a scanner/data experiment, not a trading or PnL experiment.

## Fixed setup

- Date range: `2025-07-01` to `2025-12-31`
- Universe: `SPY`, `QQQ`, `IWM`, `AAPL`, `TSLA`
- Frozen component weights:
  - `vol = 0.20`
  - `gamma = 0.20`
  - `oi_flow = 0.20`
  - `price = 0.20`
  - `macro = 0.20`
- Score threshold: `0.65`
- Top-k for evaluation: `10`
- Filters:
  - `min_volume_20d = 1_000_000`
  - `max_bid_ask_spread = 0.10`
  - `min_liquidity_score = 0.30`
- Providers:
  - Vol/Macro: `OpenBBVolMacroProvider`
  - Gamma/OI: `OpenBBGammaOIProvider`

## Reproducible run

CLI:

```bash
python -m atlas_scanner.experiments.experiment_1_historical
```

Optional overrides:

```bash
python -m atlas_scanner.experiments.experiment_1_historical \
  --start-date 2025-07-01 \
  --end-date 2025-12-31 \
  --universe SPY,QQQ,IWM,AAPL,TSLA \
  --score-threshold 0.65 \
  --top-k 10 \
  --output-dir artifacts/atlas_scanner/experiment_1
```

## Artifacts

Generated under `artifacts/atlas_scanner/experiment_1`:

- `backtest_result.pkl`
- `evaluation_result.pkl`
- `evaluation_result.json`
- `summary.json`

## Metrics to inspect

Global:
- `num_days`
- `num_symbols_seen`
- `num_ranked_total`
- `avg_ranked_per_day`
- `avg_candidates_per_day`
- `provider_status_counts`

By symbol:
- `num_observations`
- `avg_score`, `min_score`, `max_score`
- `top_k_frequency`
- `threshold_pass_frequency`

By component:
- `avg_score`
- `positive_ratio`, `neutral_ratio`, `negative_ratio`, `unavailable_ratio`
- `coverage_ratio`

## Determinism check

Run the same command twice with the same config and compare:
- per-day ranking order
- per-symbol scores
- component explainability states

The outputs should be stable for equal inputs.

## Known limitations

- No strategy execution, no fills, no slippage, no PnL.
- Provider coverage may be constrained by OpenBB data availability/schema.
- This experiment measures scanner behavior, not trading performance.
- In local smoke runs, `provider_status_counts` may appear as `no_backend` (OpenBB backend unavailable/misconfigured) or `empty` (backend reachable but empty payloads); both preserve fail-soft behavior.
- Offline scanner currently runs with fixture-backed universe mode (`universe_mode = "fixtures"`). A provider-backed universe mode is intentionally reserved as a future extension for real vendor-driven symbol discovery.

## Result notes (to fill after run)

- Provider coverage summary:
- Symbols with highest average score:
- Components with highest unavailable ratio:
- Main stability observations:
- Concrete follow-up actions:

