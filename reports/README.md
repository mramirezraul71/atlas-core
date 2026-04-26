# Reports Folder Guide

This folder contains audit and analysis outputs for ATLAS.

## What should stay versioned

- Stable markdown reports that document architecture, audits, and decisions.
- Date-based reports that are part of historical record.
- Any report explicitly requested for long-term traceability.

## What stays local (ignored by Git)

The repository ignores high-churn runtime artifacts to keep history clean:

- `reports/*_scorecard.json`
- `reports/*_scorecard_latest.md`
- `reports/evidence_*.png`
- `reports/atlas_quant_implementation_scorecard_20*.md`

Related quant runtime files are also local-only:

- `atlas_code_quant/data/learning/adaptive_policy_snapshot.json`
- `atlas_code_quant/data/options_paper_journal.jsonl`

## Regenerating scorecards

Use the official builder script:

```powershell
python atlas_code_quant/scripts/build_trading_implementation_scorecard.py
```

Optional guardrail tests before generating:

```powershell
python atlas_code_quant/scripts/build_trading_implementation_scorecard.py --run-pytest
```

The script writes fresh local outputs under `reports/` and should not create Git noise after current ignore rules.

## Temporary exception (only if strictly required)

If you must include a normally ignored artifact in a one-off investigation, add it with force:

```powershell
git add -f reports/evidence_example.png
```

Use this only for exceptional incident documentation.
