"""
lotto_quant.data.database
=========================

DuckDB persistence layer for Atlas Lotto-Quant.

DuckDB gives us fast analytical queries on time-series snapshots.
SQLite is used as a fallback when DuckDB isn't available
(e.g. minimal CI environments).

Tables
------
    scratch_off_snapshots  — time-series of prize states per game
    ev_signals             — historical EV calculations & signal events
    kelly_recommendations  — bankroll allocation history
    alert_log              — every alert dispatched
    markov_predictions     — model predictions vs. actual outcomes
"""

from __future__ import annotations

import json
import logging
import sqlite3
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

from .. import config

logger = logging.getLogger(__name__)


CREATE_TABLES_SQL = [
    """
    CREATE TABLE IF NOT EXISTS scratch_off_snapshots (
        snapshot_id     VARCHAR PRIMARY KEY,
        game_id         VARCHAR NOT NULL,
        game_name       VARCHAR NOT NULL,
        ticket_price    DOUBLE,
        snapshot_ts     TIMESTAMP,
        data_json       VARCHAR,
        ev_gross        DOUBLE,
        ev_adjusted     DOUBLE,
        depletion_ratio DOUBLE,
        anomaly_score   DOUBLE
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS ev_signals (
        signal_id       VARCHAR PRIMARY KEY,
        game_id         VARCHAR,
        signal_ts       TIMESTAMP,
        signal_type     VARCHAR,
        signal_strength VARCHAR,
        ev_net          DOUBLE,
        recommended_allocation DOUBLE,
        alert_sent      BOOLEAN DEFAULT FALSE
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS kelly_recommendations (
        rec_id          VARCHAR PRIMARY KEY,
        game_id         VARCHAR,
        rec_ts          TIMESTAMP,
        bankroll        DOUBLE,
        full_kelly      DOUBLE,
        final_fraction  DOUBLE,
        position_usd    DOUBLE,
        n_tickets       INTEGER,
        capped_by       VARCHAR
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS alert_log (
        alert_id        VARCHAR PRIMARY KEY,
        alert_ts        TIMESTAMP,
        game_id         VARCHAR,
        message         VARCHAR,
        channel         VARCHAR,
        delivered       BOOLEAN
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS markov_predictions (
        pred_id         VARCHAR PRIMARY KEY,
        game_id         VARCHAR,
        pred_ts         TIMESTAMP,
        tickets_until_ev_positive INTEGER,
        confidence      DOUBLE,
        anomaly_score   DOUBLE
    )
    """,
]


class LottoQuantDB:
    """Thin persistence wrapper that prefers DuckDB and degrades to SQLite."""

    def __init__(self, path: Optional[str] = None):
        self.path = Path(path or config.DB_PATH)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.backend = self._open_connection()
        self._ensure_schema()

    # ── connection ─────────────────────────────────────────────────
    def _open_connection(self):
        try:
            import duckdb  # type: ignore
            self._engine = "duckdb"
            return duckdb.connect(str(self.path))
        except ImportError:
            logger.warning("DuckDB unavailable — falling back to SQLite at %s",
                           config.DB_FALLBACK_SQLITE)
            self.path = Path(config.DB_FALLBACK_SQLITE)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._engine = "sqlite"
            return sqlite3.connect(str(self.path))

    def _ensure_schema(self) -> None:
        cur = self._cursor()
        for sql in CREATE_TABLES_SQL:
            cur.execute(sql)
        self._commit()

    def _cursor(self):
        if self._engine == "duckdb":
            return self.backend
        return self.backend.cursor()

    def _commit(self) -> None:
        if self._engine == "sqlite":
            self.backend.commit()

    # ── insertions ─────────────────────────────────────────────────
    def insert_snapshot(
        self,
        game_id: str,
        game_name: str,
        ticket_price: float,
        data: Dict[str, Any],
        ev_gross: float,
        ev_adjusted: float,
        depletion_ratio: float,
        anomaly_score: float,
    ) -> str:
        snap_id = str(uuid.uuid4())
        cur = self._cursor()
        cur.execute(
            """
            INSERT INTO scratch_off_snapshots
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                snap_id,
                game_id,
                game_name,
                ticket_price,
                datetime.utcnow(),
                json.dumps(data, default=str),
                ev_gross,
                ev_adjusted,
                depletion_ratio,
                anomaly_score,
            ),
        )
        self._commit()
        return snap_id

    def insert_signal(
        self,
        game_id: str,
        signal_type: str,
        signal_strength: str,
        ev_net: float,
        recommended_allocation: float,
        alert_sent: bool = False,
    ) -> str:
        sid = str(uuid.uuid4())
        cur = self._cursor()
        cur.execute(
            """
            INSERT INTO ev_signals
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                sid,
                game_id,
                datetime.utcnow(),
                signal_type,
                signal_strength,
                ev_net,
                recommended_allocation,
                alert_sent,
            ),
        )
        self._commit()
        return sid

    def insert_kelly(
        self,
        game_id: str,
        bankroll: float,
        full_kelly: float,
        final_fraction: float,
        position_usd: float,
        n_tickets: int,
        capped_by: str,
    ) -> str:
        rid = str(uuid.uuid4())
        cur = self._cursor()
        cur.execute(
            """
            INSERT INTO kelly_recommendations
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                rid,
                game_id,
                datetime.utcnow(),
                bankroll,
                full_kelly,
                final_fraction,
                position_usd,
                n_tickets,
                capped_by,
            ),
        )
        self._commit()
        return rid

    def insert_alert(
        self,
        game_id: str,
        message: str,
        channel: str,
        delivered: bool,
    ) -> str:
        aid = str(uuid.uuid4())
        cur = self._cursor()
        cur.execute(
            """
            INSERT INTO alert_log
            VALUES (?, ?, ?, ?, ?, ?)
            """,
            (
                aid,
                datetime.utcnow(),
                game_id,
                message,
                channel,
                delivered,
            ),
        )
        self._commit()
        return aid

    def insert_markov_prediction(
        self,
        game_id: str,
        tickets_until_ev_positive: Optional[int],
        confidence: float,
        anomaly_score: float,
    ) -> str:
        pid = str(uuid.uuid4())
        cur = self._cursor()
        cur.execute(
            """
            INSERT INTO markov_predictions
            VALUES (?, ?, ?, ?, ?, ?)
            """,
            (
                pid,
                game_id,
                datetime.utcnow(),
                tickets_until_ev_positive,
                confidence,
                anomaly_score,
            ),
        )
        self._commit()
        return pid

    # ── queries ────────────────────────────────────────────────────
    def latest_snapshots(self, limit: int = 50) -> List[Dict[str, Any]]:
        cur = self._cursor()
        cur.execute(
            "SELECT * FROM scratch_off_snapshots "
            "ORDER BY snapshot_ts DESC LIMIT ?",
            (limit,),
        )
        rows = cur.fetchall()
        cols = [
            "snapshot_id", "game_id", "game_name", "ticket_price",
            "snapshot_ts", "data_json", "ev_gross", "ev_adjusted",
            "depletion_ratio", "anomaly_score",
        ]
        return [dict(zip(cols, r)) for r in rows]

    def signals_since(self, hours: int = 24) -> List[Dict[str, Any]]:
        cur = self._cursor()
        # Compatible with both SQLite and DuckDB:
        cur.execute(
            "SELECT * FROM ev_signals "
            "WHERE signal_ts >= ? "
            "ORDER BY signal_ts DESC",
            (datetime.utcnow().isoformat(timespec="seconds"),),
        )
        rows = cur.fetchall()
        cols = [
            "signal_id", "game_id", "signal_ts", "signal_type",
            "signal_strength", "ev_net", "recommended_allocation", "alert_sent",
        ]
        return [dict(zip(cols, r)) for r in rows]

    # ── teardown ───────────────────────────────────────────────────
    def close(self) -> None:
        try:
            self.backend.close()
        except Exception:
            pass
