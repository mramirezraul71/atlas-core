"""Informe auditable JSON + Markdown para el módulo XGBoost."""
from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import xgboost as xgb

from config.settings import settings


def _ic_tracker_snapshot() -> dict[str, Any]:
    try:
        from learning.ic_signal_tracker import get_ic_tracker

        snap = get_ic_tracker().summary()
        overall = snap.get("overall") or {}
        rho = overall.get("rho")
        return {
            "ic_tracker_ic": rho,
            "ic_tracker_status": "ok" if rho is not None else "insufficient_data",
            "raw": {"signals_with_outcome": snap.get("signals_with_outcome"), "total_signals": snap.get("total_signals")},
        }
    except Exception:
        return {"ic_tracker_ic": None, "ic_tracker_status": "insufficient_data"}


def generate_audit_report(
    *,
    walk_forward: dict[str, Any] | None = None,
    feature_importances: list[dict[str, Any]] | None = None,
    baseline_comparison: dict[str, Any] | None = None,
    n_trades_used: int = 0,
    phase: int = 0,
    extra_alerts: list[str] | None = None,
) -> tuple[Path, Path]:
    """Escribe audit_report.json y audit_report.md bajo xgboost_model_dir."""
    model_dir = Path(settings.xgboost_model_dir)
    model_dir.mkdir(parents=True, exist_ok=True)
    json_path = model_dir / "audit_report.json"
    md_path = model_dir / "audit_report.md"

    meta_path = model_dir / "xgboost_model_meta.json"
    meta: dict[str, Any] = {}
    if meta_path.is_file():
        meta = json.loads(meta_path.read_text(encoding="utf-8"))

    wf = walk_forward or meta.get("walk_forward") or {"folds": []}
    base_cmp = baseline_comparison or {
        "baseline_feature": "local_win_rate_pct",
        "baseline_auc": float(meta.get("baseline_auc") or 0.0),
        "model_auc": float(meta.get("test_auc") or 0.0),
        "improvement": float((meta.get("test_auc") or 0) - (meta.get("baseline_auc") or 0)),
        "beats_baseline": bool(meta.get("beats_baseline", False)),
    }

    fi = feature_importances or []
    if not fi and meta.get("feature_names"):
        fi = [{"feature": n, "importance": 0.0, "rank": i + 1} for i, n in enumerate(meta["feature_names"][:20])]

    ic_xgb = 0.0
    folds = wf.get("folds") or []
    if folds:
        try:
            ic_xgb = float(sum(f.get("metrics", {}).get("ic_spearman", 0) for f in folds) / max(1, len(folds)))
        except Exception:
            ic_xgb = 0.0

    ic_tr = _ic_tracker_snapshot()
    alerts: list[str] = list(extra_alerts or [])
    for f in folds:
        if f.get("overfitting_alert"):
            alerts.append(f"Overfitting fold {f.get('fold')}: train_auc - test_auc > 0.15")

    payload = {
        "metadata": {
            "model_version": "1.0.0",
            "training_date": datetime.now(timezone.utc).isoformat(),
            "n_trades_used": n_trades_used or int(meta.get("n_real_trades") or 0),
            "phase": phase or int(meta.get("phase") or 0),
            "atlas_version": "ATLAS_PUSH",
            "xgboost_version": getattr(xgb, "__version__", "unknown"),
        },
        "feature_importances": fi,
        "walk_forward_results": folds,
        "baseline_comparison": base_cmp,
        "calibration": {"method": "platt_scaling" if (meta.get("phase") or 0) >= 2 else "none", "calibration_curve_data": []},
        "ic_comparison": {
            "xgboost_ic": ic_xgb,
            "ic_tracker_ic": ic_tr.get("ic_tracker_ic"),
            "ic_tracker_status": ic_tr.get("ic_tracker_status", "insufficient_data"),
        },
        "alerts": alerts,
    }
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    ok = "[OK]" if not alerts else "[WARN]"
    crit = "[CRITICAL]" if any("Overfitting" in a for a in alerts) else ""
    lines = [
        "# XGBoost Signal — Audit Report",
        "",
        f"Generado: {payload['metadata']['training_date']}",
        "",
        "## ALERTS",
        f"{ok} Estado general del informe.",
    ]
    if alerts:
        for a in alerts:
            tag = "[CRITICAL]" if "Overfitting" in a else "[WARN]"
            lines.append(f"{tag} {a}")
    else:
        lines.append("[OK] Sin alertas de overfitting registradas.")
    lines.extend(
        [
            "",
            "## Baseline",
            json.dumps(base_cmp, indent=2, ensure_ascii=False),
            "",
            "## IC",
            json.dumps(payload["ic_comparison"], indent=2, ensure_ascii=False),
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")
    return json_path, md_path
