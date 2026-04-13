# Plantilla — Audit Report XGBoost Signal

Este documento describe la estructura esperada de `audit_report.md` / `audit_report.json`
generados por `learning.xgboost_signal.audit_report.generate_audit_report`.

## Secciones obligatorias

1. **ALERTS** — Marcadores `[OK]`, `[WARN]`, `[CRITICAL]` por línea.
2. **Baseline** — Comparación AUC vs `local_win_rate_pct` (predictor naive).
3. **IC** — IC Spearman del modelo vs resumen del `ic_signal_tracker` cuando exista.

## JSON (`audit_report.json`)

- `metadata`: versión, fecha, `n_trades_used`, `phase`, versión xgboost.
- `feature_importances`: lista `{feature, importance, rank}`.
- `walk_forward_results`: folds con métricas y `overfitting_alert`.
- `baseline_comparison`: `baseline_auc`, `model_auc`, `beats_baseline`.
- `calibration`: método (p. ej. Platt en Fase 2).
- `ic_comparison`: `xgboost_ic`, `ic_tracker_ic`, `ic_tracker_status`.
- `alerts`: strings legibles.

## Notas

- Fase 0: sin entrenamiento; informes pueden reflejar `n_trades_used=0`.
- Bloqueo operativo solo en Fase 2 y con `beats_baseline=true` en metadatos del modelo.
