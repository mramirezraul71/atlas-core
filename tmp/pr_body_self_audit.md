## Summary

- Integrates a thin `operational_self_audit` runner invoked at defined points in the paper cycle.
- Reuses existing configuration/protocol snapshots; does **not** change paper safety gate semantics, execution, journal, Kelly, ML or market_open_config logic.
- Produces a structured `SelfAuditResult` (severity, checks, scope) and optionally exposes a compact self-audit summary in readiness when enabled.

## Testing

- `python -m pytest atlas_code_quant/tests/test_operational_self_audit.py -q` → 5 passed.
- `python -m pytest atlas_code_quant/tests/test_auto_cycle_audit_fixes.py -q` → 3 passed (regression on auto-cycle paths).

## Risks

- False positives could add log noise; mitigated by severity levels and a `QUANT_OPERATIONAL_SELF_AUDIT_ENABLED` flag.
- Self-audit in paper must **never** block submit by default; this PR keeps live disabled and does not introduce new hard vetoes.

## Follow-ups

- Consider wiring BLOCK semantics only when/if live is explicitly approved.
- Tighten checks against the full audit protocol once the paper-only path is stable.
- Extend documentation for operators on how to interpret self-audit summaries.
