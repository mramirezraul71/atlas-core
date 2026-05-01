# PDT Rule Elimination Migration

**Effective:** SEC approval 14-04-2026

## Changes

### Deprecated Gates
- [x] Remove/blocking behavior from `check_pdt_limit()` and mark as deprecated
- [x] Remove `$25K` minimum capital requirement as a regulatory gate
- [x] Remove PDT EOD blocking flow from Tradier execution path

### New Operational Gates
- [x] Add `check_daily_trade_limit()` (50/day live, 100/day paper)
- [x] Add `check_intraday_exposure()` (max 50% equity)
- [x] Add `check_intraday_drawdown()` (max 10% loss)
- [x] Add `check_consecutive_loss_cooldown()` (3+ losses)

### Tests
- [x] Add `tests/test_operational_gates_post_pdt.py`
- [x] Keep existing suite green with compatibility payloads (`pdt_status`)
- [ ] Update/remove legacy PDT controller tests in a dedicated follow-up PR

## Status
- [x] Code changes complete
- [x] Tests for migration scope passing
- [x] Docs updated
- [ ] Merged to `journal-forensic-20260411-quant`
