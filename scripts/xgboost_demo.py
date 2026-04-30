#!/usr/bin/env python3
"""
Demo pipeline XGBoost Signal con journal sintético si hay <30 trades reales.
Ejecutar desde repo: python scripts/xgboost_demo.py
"""
from __future__ import annotations

import json
import os
import sqlite3
import sys
import tempfile
from datetime import datetime, timedelta, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
QUANT = ROOT / "atlas_code_quant"
sys.path.insert(0, str(QUANT))

os.environ.setdefault("QUANT_XGBOOST_ENABLED", "true")

from config.settings import settings  # noqa: E402
from learning.xgboost_signal.audit_report import generate_audit_report  # noqa: E402
from learning.xgboost_signal.feature_builder import FeatureBuilder  # noqa: E402
from learning.xgboost_signal.model_trainer import (  # noqa: E402
    _count_real_trades,
    get_training_phase,
    train_if_ready,
)
from learning.xgboost_signal.model_loader import XGBoostModelLoader  # noqa: E402
from learning.xgboost_signal.signal_scorer import XGBoostSignalScorer  # noqa: E402
from learning.xgboost_signal.exit_advisor import XGBoostExitAdvisor  # noqa: E402


def _ensure_schema(conn: sqlite3.Connection) -> None:
    conn.execute(
        """
        CREATE TABLE IF NOT EXISTS trading_journal (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            journal_key TEXT,
            strategy_type TEXT,
            symbol TEXT,
            win_rate_at_entry REAL,
            current_win_rate_pct REAL,
            iv_rank REAL,
            realized_pnl REAL,
            entry_time TEXT,
            exit_time TEXT,
            status TEXT,
            entry_notional REAL,
            unrealized_pnl REAL,
            attribution_json TEXT,
            post_mortem_json TEXT,
            broker_order_ids_json TEXT
        )
        """
    )
    conn.commit()


def _seed_synthetic(conn: sqlite3.Connection, n: int = 500) -> None:
    conn.execute("DELETE FROM trading_journal")
    base = datetime(2025, 1, 1, tzinfo=timezone.utc)
    for i in range(n):
        win = 1 if (i % 3) != 0 else 0
        pnl = 12.5 if win else -8.0
        et = base + timedelta(days=i % 120, hours=i % 24)
        xt = et + timedelta(hours=10)
        conn.execute(
            """
            INSERT INTO trading_journal (
                journal_key, strategy_type, symbol, win_rate_at_entry, current_win_rate_pct,
                iv_rank, realized_pnl, entry_time, exit_time, status, entry_notional,
                unrealized_pnl, attribution_json, post_mortem_json, broker_order_ids_json
            ) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
            """,
            (
                f"demo:{i}",
                "equity_long",
                "DEMO",
                55.0 + (i % 10),
                52.0 + (i % 8),
                30.0,
                pnl,
                et.isoformat(),
                xt.isoformat(),
                "closed",
                1000.0,
                0.0,
                json.dumps({"predicted_move_pct": 0.02, "confidence_pct": 65}),
                json.dumps({}),
                json.dumps([f"ord-{i}"]),
            ),
        )
    conn.commit()


def main() -> None:
    work = Path(tempfile.mkdtemp(prefix="xgb_demo_"))
    db = work / "journal.sqlite3"
    conn = sqlite3.connect(str(db))
    _ensure_schema(conn)
    n_real = _count_real_trades(conn)
    if n_real < 30:
        print(f"Journal vacío o pocos trades reales ({n_real}); generando 500 sintéticos.")
        _seed_synthetic(conn, 500)
        n_real = _count_real_trades(conn)
    conn.close()

    # Redirigir settings a DB temporal
    settings.journal_db_path = db
    settings.xgboost_model_dir = work / "models"
    settings.xgboost_model_dir.mkdir(parents=True, exist_ok=True)

    phase = get_training_phase(n_real)
    print(f"Fase detectada: {phase} | trades reales (según criterio): {n_real}")

    fb = FeatureBuilder()
    sample = {"selection_score_pct": 72, "local_win_rate_pct": 58, "regime": "trending"}
    print("Features pre-trade (muestra):", fb.build_pretrade_features(sample).head(8).to_dict())

    res = train_if_ready()
    print("Train:", res)

    XGBoostModelLoader.reset_instance()
    XGBoostSignalScorer.reset_instance()
    sc = XGBoostSignalScorer.get_instance().score(sample)
    print("Score:", sc)

    adv = XGBoostExitAdvisor.get_instance().advise_exit(
        {"entry_time": datetime.now(timezone.utc).isoformat(), "entry_notional": 1000, "unrealized_pnl": -5},
        {"adverse_drift_pct": 0.01},
    )
    print("Exit advice:", adv)

    jp, mp = generate_audit_report(phase=phase, n_trades_used=n_real)
    print("Audit:", jp, mp)
    meta_path = settings.xgboost_model_dir / "xgboost_model_meta.json"
    if meta_path.is_file():
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        print("Top features (meta):", meta.get("feature_names", [])[:5])


if __name__ == "__main__":
    main()
