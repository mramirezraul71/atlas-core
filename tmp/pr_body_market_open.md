## Summary

- Integrates `atlas_code_quant/config/market_open_config.json` as the operational source of truth for risk and schedule parameters resolved on `TradingConfig` (`settings`).
- Defines clear precedence **per field**: explicit non-empty **environment variable** > **JSON** > **code/dataclass defaults** (documented in `_precedence` on the JSON and in `_apply_market_open_operational_config`).
- Applies from JSON (when env does not override): `risk.max_positions` → `market_open_max_positions`; `risk.kelly_fraction` → `kelly_fraction`; `risk.max_risk_per_trade` → `equity_kelly_max_risk_per_trade_pct`; `risk.max_position_pct` → `kelly_max_position_pct`; `risk.daily_loss_limit` → `market_open_daily_loss_limit_pct`; `schedule_et` (`market_open`, `market_close`, `scan_interval_min`) → schedule / scan fields used with the existing market-hours path.
- Adds `_sync_risk_fields_from_explicit_env` so env wins reliably at load time despite dataclass defaults being fixed at import for `_fenv`/`_ienv`-based fields.
- Improves observability: `market_open_operational` snapshot on `/operation/readiness` (fast) and diagnostic payloads via `_market_open_operational_config_snapshot()` in `api/main.py` only.
- **Out of scope / unchanged by design:** paper safety market-hours gate and `max_open_positions` **guard logic** (still driven by `settings.market_open_max_positions`), Kelly **sizing code** in `kelly_sizer` / `signal_executor`, ML runtime, scanner, execution, journal.

## Testing

```bash
python -m pytest atlas_code_quant/tests/test_market_open_config_runtime.py atlas_code_quant/tests/test_auto_cycle_audit_fixes.py -q
python -m pytest atlas_code_quant/tests/test_equity_sizing_kelly.py atlas_code_quant/tests/test_operation_center_core_contract.py -q
```

**New / key tests** (`test_market_open_config_runtime.py`):

- `test_market_open_config_loads_risk_fields`
- `test_market_open_config_invalid_values_fall_back_safely`
- `test_market_open_config_applies_max_positions_guard`
- `test_market_open_config_applies_kelly_fraction`
- `test_market_open_config_missing_file_no_crash`
- `test_env_overrides_json_for_kelly`

**Results:** 9 passed (first command block) and 9 passed (second command block), as run on the branch.

## Risks

- Misconfigured JSON or env can change effective risk caps (mitigated by per-field validation, clamps, and isolated invalid-field handling with warnings).
- Operators should treat JSON/env changes as **production-relevant**: they alter effective limits and fractions without changing gate/sizing **code paths**.
- This PR is primarily a **configuration source** change: behavior shifts when resolved values differ from previous literals/defaults (e.g. `operation_center` default `max_risk_per_trade_pct` now aligns with `settings.equity_kelly_max_risk_per_trade_pct` when state does not override).
- `refresh_market_open_operational_config()` re-reads JSON at runtime; callers should understand it mutates the global `settings` singleton—use deliberately.

## Follow-ups

- Second-wave remainder: integrate **self-audit** into the operational cycle where the audit protocol prescribes it.
- Second-wave remainder: **atlas_core** canonical / reproducible wiring for cross-repo contracts.
- Optional governance: **enforce** `market_open_daily_loss_limit_pct` (and related circuit-breaker semantics) in execution or portfolio guards, building on the now-centralized value.
- Optional: document env keys (`QUANT_MARKET_OPEN_*`, `QUANT_KELLY_*`, etc.) in operator runbooks next to `market_open_config.json`.
- Optional: extend observability (metrics or structured logs) when `market_open_config_loaded` is false in production.
