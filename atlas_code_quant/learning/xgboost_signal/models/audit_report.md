# XGBoost Signal — Audit Report

Generado: 2026-04-17T05:07:14.267379+00:00

## ALERTS
[WARN] Estado general del informe.
[WARN] Dataset con diversidad insuficiente de target_win; XGBoost permanece en modo pass-through hasta nuevas etiquetas positivas.

## Baseline
{
  "baseline_feature": "local_win_rate_pct",
  "baseline_auc": 0.0,
  "model_auc": 0.0,
  "improvement": 0.0,
  "beats_baseline": false
}

## IC
{
  "xgboost_ic": 0.0,
  "ic_tracker_ic": null,
  "ic_tracker_status": "insufficient_data"
}