# ATLAS Scanner Backtest and Historical Evaluation

## Purpose and scope

This document describes the implemented backtest and historical evaluation subarchitecture for `atlas_scanner`.

Scope:
- Batch historical orchestration over the offline scanner.
- Walk-forward orchestration mode.
- High-level API facade for backtest entry points.
- Historical evaluation metrics for scanner outputs.

Out of scope:
- Live strategy execution.
- Order routing.
- Trading PnL metrics.

---

## Architectural position

The current scanner stack is layered as follows:

1. **Offline runner** (`atlas_scanner/runner/offline.py`)  
   Single date-level evaluation engine.
2. **Backtest engine** (`atlas_scanner/backtest/engine.py`)  
   Historical orchestration across dates/modes.
3. **Backtest API facade** (`atlas_scanner/api/backtest.py`)  
   Ergonomic high-level entry points.
4. **Historical evaluation** (`atlas_scanner/backtest/evaluation.py`)  
   Descriptive metrics over `BacktestResult`.

This preserves separation of concerns: execution logic remains in the runner; orchestration in engine; user-facing call ergonomics in API; metrics analysis in evaluation.

---

## Offline runner as evaluation engine

### Implemented

`run_offline_scan(...)` is the base evaluation engine and remains the single path for:
- universe filtering,
- provider integration (vol/macro and gamma/OI),
- multifactor scoring,
- component explainability,
- provider observability metadata.

Current optional inputs used by historical orchestration:
- `as_of_date`
- `symbol_allowlist`
- existing filters:
  - `min_volume_20d`
  - `max_bid_ask_spread`
  - `min_liquidity_score`
  - `min_ref_price`
  - `max_ref_price`
  - `max_event_risk`

Current outputs used upstream:
- `OfflineScanResult.ranked_symbols`
- `OfflineScanResult.candidate_opportunities`
- `OfflineScanResult.meta["providers"]`
- explainability already embedded in scored/candidate structures.

### Deferred / not implemented

- Live execution coupling (intentionally absent).

---

## Batch backtest orchestration

### Implemented

Module: `atlas_scanner/backtest/engine.py`

Contracts:
- `BacktestRequest`
- `DatedScanResult`
- `BacktestResult`

Entry points:
- `run_backtest(request: BacktestRequest) -> BacktestResult`
- `run_walk_forward(request: BacktestRequest, window_days: int) -> BacktestResult`

Behavior:
- Iterates target dates.
- Calls `run_offline_scan(...)` for each date.
- Aggregates date-level outputs into `BacktestResult.results`.
- Produces basic aggregate metadata in `BacktestResult.meta`:
  - day counts
  - symbol/ranked/candidate counts
  - provider status counts
  - basic symbol/component score summaries.

### Deferred / not implemented

- Persistence of run artifacts.
- Parallel execution or caching.

---

## Walk-forward mode

### Implemented

`run_walk_forward(...)` uses rolling evaluation start:
- first evaluated date = `start_date + (window_days - 1)`
- then daily until `end_date`.

The mode still delegates each step to `run_offline_scan(...)`, so scoring/explainability/observability remain identical to batch backtest mode.

### Deferred / not implemented

- Advanced window protocols (multiple schemes, nested CV).

---

## Backtest API facade

### Implemented

Module: `atlas_scanner/api/backtest.py`

High-level facade:
- `BacktestScanner`
- `scan_backtest_offline(...)`
- `scan_walk_forward_offline(...)`

Responsibilities:
- Accept explicit API parameters.
- Build `BacktestRequest`.
- Delegate to `run_backtest(...)` / `run_walk_forward(...)`.
- Return `BacktestResult` as-is.

Non-responsibilities:
- No scoring logic.
- No orchestration duplication.
- No result rewriting.

### Deferred / not implemented

- Additional API surfaces for reporting/export.

---

## Historical evaluation layer

### Implemented

Module: `atlas_scanner/backtest/evaluation.py`

Contracts:
- `EvaluationRequest`
- `SymbolEvaluationSummary`
- `ComponentEvaluationSummary`
- `HistoricalEvaluationResult`

Entry point:
- `evaluate_historical_backtest(request: EvaluationRequest) -> HistoricalEvaluationResult`

Input model:
- Consumes `BacktestResult` only.
- Does not call providers directly.
- Does not execute runner logic.

Metrics currently implemented:

Global (`HistoricalEvaluationResult.meta`):
- `num_days`
- `num_symbols_seen`
- `num_ranked_total`
- `avg_ranked_per_day`
- `avg_candidates_per_day`
- `provider_status_counts`
- evaluation context (`top_k`, `score_threshold`)

Symbol summaries:
- observations
- avg/min/max score
- top-k frequency
- threshold pass frequency (when threshold is provided)

Component summaries:
- observations
- avg score (available observations)
- ratio per status (`positive`, `neutral`, `negative`, `unavailable`)
- coverage ratio

### Why evaluation is separate from engine

`engine.py` orchestrates historical runs; `evaluation.py` analyzes already-produced outputs.  
Keeping them separate avoids mixing orchestration concerns with analytical semantics and keeps metrics pure/testable.

### Deferred / not implemented

- Trading/PnL metrics.
- Auto-calibration or parameter optimization.

---

## Implemented contracts

Implemented and currently active:
- `run_offline_scan(...)` (runner layer)
- `BacktestRequest`, `DatedScanResult`, `BacktestResult` (engine contracts)
- `run_backtest(...)`, `run_walk_forward(...)` (engine entry points)
- `BacktestScanner`, `scan_backtest_offline(...)`, `scan_walk_forward_offline(...)` (API facade)
- `EvaluationRequest`, `HistoricalEvaluationResult`
- `SymbolEvaluationSummary`, `ComponentEvaluationSummary`
- `evaluate_historical_backtest(...)` (evaluation entry point)

---

## Data flow

### Scanner historical execution flow

`scan_backtest_offline(...)` / `scan_walk_forward_offline(...)`  
→ build `BacktestRequest`  
→ `run_backtest(...)` / `run_walk_forward(...)`  
→ per date: `run_offline_scan(...)`  
→ scored symbols + explainability + provider metadata  
→ `BacktestResult`

### Historical evaluation flow

`BacktestResult`  
→ `evaluate_historical_backtest(...)`  
→ global/symbol/component metric summaries  
→ `HistoricalEvaluationResult`

---

## Design decisions

### Implemented

- Offline runner remains the single evaluation engine.
- Backtest engine orchestrates dates/modes only.
- API facade simplifies access only.
- Evaluation layer computes descriptive scanner metrics only.
- Explainability and provider observability are preserved by reusing runner outputs.

### Deferred / not implemented

- Any coupling to live strategy execution.
- Any assumption that ranking implies trading alpha.

---

## Deferred / not implemented

- No persistence/reporting storage layer.
- No chart generation.
- No export serialization workflows (CSV/JSON reporting pipeline).
- No trading strategy metrics (PnL, fills, slippage, execution quality).
- No automatic calibration/parameter optimization.
- No real production `GammaOIProvider` adapter (dummy adapter still used for gamma/OI baseline).
- No live integration path.

---

## Recommended next phase

Primary recommendation:
- Implement a real `GammaOIProvider` adapter with tests equivalent in rigor to current Vol/Macro and historical workflows.

Secondary recommendation:
- Define a minimal reporting/export contract for historical evaluation outputs once provider realism is in place.

