"""paper/paper_broker.py — ATLAS Paper Trading Broker (local, sin dependencias externas).

Motor de paper trading completamente autónomo:
  - Capital virtual configurable (default $25,000)
  - Fills instantáneos al precio de mercado ± slippage configurable
  - Persistencia SQLite en data/paper_account.db
  - Equity curve con snapshot por trade
  - Thread-safe, singleton global

Endpoints API:
    GET  /paper/account        — resumen de cuenta
    GET  /paper/positions      — posiciones abiertas
    GET  /paper/orders         — blotter (historial de órdenes)
    GET  /paper/equity-curve   — curva de equity
    POST /paper/fill           — registrar fill (llamado por autonomous_loop)
    POST /paper/close          — cerrar posición
    POST /paper/reset          — resetear cuenta (capital inicial)

Uso::

    broker = get_paper_broker()
    result = broker.fill(
        symbol="XOP",
        side="buy",
        qty=100,
        price=131.20,
        strategy="CALL_VERTICAL",
        signal_score=0.799,
        score_tier="FULL",
    )
    print(result.order_id, result.pnl)
"""
from __future__ import annotations

import json
import logging
import sqlite3
import threading
import time
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

logger = logging.getLogger("atlas.paper.broker")

# ── Rutas ──────────────────────────────────────────────────────────────────────
_BASE_DIR  = Path(__file__).resolve().parent.parent          # atlas_code_quant/
_DATA_DIR  = _BASE_DIR / "data"
_DB_PATH   = _DATA_DIR / "paper_account.db"

# ── Constantes ─────────────────────────────────────────────────────────────────
DEFAULT_CAPITAL    = 25_000.0     # capital inicial por defecto ($25k → umbral PDT)
DEFAULT_SLIPPAGE   = 0.0002       # 0.02% slippage simulado por lado
DEFAULT_COMMISSION = 0.65         # $0.65 por contrato/acción (similar a Tradier)

_ORDER_STATUS_FILLED   = "filled"
_ORDER_STATUS_REJECTED = "rejected"
_ORDER_STATUS_OPEN     = "open"
_ORDER_STATUS_CLOSED   = "closed"


# ── Dataclasses ────────────────────────────────────────────────────────────────

@dataclass
class PaperOrderResult:
    """Resultado de un fill o rechazo."""
    order_id:     str   = ""
    symbol:       str   = ""
    side:         str   = ""          # buy | sell | sell_short | buy_to_cover
    qty:          int   = 0
    fill_price:   float = 0.0
    commission:   float = 0.0
    status:       str   = _ORDER_STATUS_FILLED
    reject_reason:str   = ""
    pnl:          float = 0.0        # P&L realizado (solo en cierres)
    strategy:     str   = ""
    signal_score: float = 0.0
    score_tier:   str   = ""
    option_strategy: str = ""
    timestamp:    float = field(default_factory=time.time)


@dataclass
class PaperPosition:
    """Posición abierta en la cuenta paper."""
    symbol:       str
    side:         str          # long | short
    qty:          int
    avg_price:    float
    current_price:float
    strategy:     str   = ""
    option_strategy: str = ""
    signal_score: float = 0.0
    score_tier:   str   = ""
    opened_at:    float = field(default_factory=time.time)
    order_id:     str   = ""

    @property
    def unrealized_pnl(self) -> float:
        mult = 1.0 if self.side == "long" else -1.0
        return round(mult * (self.current_price - self.avg_price) * self.qty, 2)

    @property
    def unrealized_pnl_pct(self) -> float:
        if self.avg_price <= 0:
            return 0.0
        return round(self.unrealized_pnl / (self.avg_price * self.qty) * 100, 3)


@dataclass
class PaperAccountSummary:
    """Snapshot completo de la cuenta paper."""
    initial_capital:   float
    cash:              float
    equity:            float        # cash + valor posiciones abiertas
    unrealized_pnl:    float
    realized_pnl:      float
    total_pnl:         float
    total_pnl_pct:     float
    total_trades:      int
    winning_trades:    int
    losing_trades:     int
    win_rate_pct:      float
    max_drawdown_pct:  float
    peak_equity:       float
    open_positions:    int
    last_updated:      str

    def to_dict(self) -> dict:
        return asdict(self)


# ── Esquema SQLite ─────────────────────────────────────────────────────────────

_DDL = """
CREATE TABLE IF NOT EXISTS paper_config (
    key   TEXT PRIMARY KEY,
    value TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS paper_orders (
    order_id       TEXT PRIMARY KEY,
    symbol         TEXT NOT NULL,
    side           TEXT NOT NULL,
    qty            INTEGER NOT NULL,
    fill_price     REAL NOT NULL,
    commission     REAL NOT NULL DEFAULT 0.0,
    status         TEXT NOT NULL,
    reject_reason  TEXT DEFAULT '',
    pnl            REAL DEFAULT 0.0,
    strategy       TEXT DEFAULT '',
    option_strategy TEXT DEFAULT '',
    signal_score   REAL DEFAULT 0.0,
    score_tier     TEXT DEFAULT '',
    ts             REAL NOT NULL,
    equity_after   REAL DEFAULT 0.0
);

CREATE TABLE IF NOT EXISTS paper_positions (
    symbol         TEXT PRIMARY KEY,
    side           TEXT NOT NULL,
    qty            INTEGER NOT NULL,
    avg_price      REAL NOT NULL,
    current_price  REAL NOT NULL,
    strategy       TEXT DEFAULT '',
    option_strategy TEXT DEFAULT '',
    signal_score   REAL DEFAULT 0.0,
    score_tier     TEXT DEFAULT '',
    opened_at      REAL NOT NULL,
    order_id       TEXT DEFAULT ''
);

CREATE TABLE IF NOT EXISTS paper_equity_curve (
    ts             REAL PRIMARY KEY,
    equity         REAL NOT NULL,
    cash           REAL NOT NULL,
    realized_pnl   REAL NOT NULL,
    event          TEXT DEFAULT ''
);

CREATE INDEX IF NOT EXISTS idx_orders_ts  ON paper_orders(ts DESC);
CREATE INDEX IF NOT EXISTS idx_equity_ts  ON paper_equity_curve(ts DESC);
"""


# ── PaperBroker ────────────────────────────────────────────────────────────────

class PaperBroker:
    """Motor de paper trading local con persistencia SQLite.

    Thread-safe mediante RLock interno.
    """

    def __init__(
        self,
        db_path: Path = _DB_PATH,
        initial_capital: float = DEFAULT_CAPITAL,
        slippage: float = DEFAULT_SLIPPAGE,
        commission_per_unit: float = DEFAULT_COMMISSION,
    ) -> None:
        self._db_path = db_path
        self._slippage = slippage
        self._commission = commission_per_unit
        self._lock = threading.RLock()

        _DATA_DIR.mkdir(parents=True, exist_ok=True)
        self._init_db()
        self._ensure_capital(initial_capital)
        logger.info(
            "PaperBroker iniciado — db=%s capital=%.2f",
            db_path.name, self.cash,
        )

    # ── DB helpers ─────────────────────────────────────────────────────────────

    def _conn(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self._db_path, check_same_thread=False)
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA journal_mode=WAL")
        return conn

    def _init_db(self) -> None:
        with self._conn() as conn:
            conn.executescript(_DDL)

    def _ensure_capital(self, initial_capital: float) -> None:
        """Crea la config de cuenta si no existe."""
        with self._conn() as conn:
            row = conn.execute(
                "SELECT value FROM paper_config WHERE key='initial_capital'"
            ).fetchone()
            if row is None:
                conn.execute(
                    "INSERT INTO paper_config VALUES ('initial_capital', ?)",
                    (str(initial_capital),)
                )
                conn.execute(
                    "INSERT INTO paper_config VALUES ('cash', ?)",
                    (str(initial_capital),)
                )
                conn.execute(
                    "INSERT INTO paper_config VALUES ('realized_pnl', '0.0')"
                )
                conn.execute(
                    "INSERT INTO paper_config VALUES ('peak_equity', ?)",
                    (str(initial_capital),)
                )
                # Snapshot inicial en equity curve
                conn.execute(
                    "INSERT INTO paper_equity_curve VALUES (?,?,?,?,'account_created')",
                    (time.time(), initial_capital, initial_capital, 0.0)
                )

    def _get_cfg(self, key: str) -> float:
        with self._conn() as conn:
            row = conn.execute(
                "SELECT value FROM paper_config WHERE key=?", (key,)
            ).fetchone()
            return float(row["value"]) if row else 0.0

    def _set_cfg(self, conn: sqlite3.Connection, key: str, value: float) -> None:
        conn.execute(
            "INSERT OR REPLACE INTO paper_config VALUES (?,?)", (key, str(value))
        )

    # ── Propiedades públicas ───────────────────────────────────────────────────

    @property
    def cash(self) -> float:
        return self._get_cfg("cash")

    @property
    def initial_capital(self) -> float:
        return self._get_cfg("initial_capital")

    @property
    def realized_pnl(self) -> float:
        return self._get_cfg("realized_pnl")

    # ── Fill ───────────────────────────────────────────────────────────────────

    def fill(
        self,
        symbol:        str,
        side:          str,        # buy | sell | sell_short | buy_to_cover
        qty:           int,
        price:         float,
        strategy:      str   = "",
        signal_score:  float = 0.0,
        score_tier:    str   = "",
        option_strategy: str = "",
        metadata:      dict  = None,
    ) -> PaperOrderResult:
        """Registra un fill paper con slippage y comisión simulados.

        Returns PaperOrderResult con status='filled' o 'rejected'.
        """
        with self._lock:
            order_id = str(uuid.uuid4())[:12]
            ts = time.time()

            # Aplicar slippage
            if side in ("buy", "buy_to_cover"):
                fill_price = round(price * (1 + self._slippage), 4)
            else:
                fill_price = round(price * (1 - self._slippage), 4)

            commission = round(self._commission * qty, 2)
            cost = fill_price * qty + commission

            result = PaperOrderResult(
                order_id=order_id,
                symbol=symbol,
                side=side,
                qty=qty,
                fill_price=fill_price,
                commission=commission,
                strategy=strategy,
                signal_score=signal_score,
                score_tier=score_tier,
                option_strategy=option_strategy,
                timestamp=ts,
            )

            with self._conn() as conn:
                cash = self._get_cfg("cash")

                # ── Apertura (buy / sell_short) ────────────────────────────────
                if side in ("buy", "sell_short"):
                    if cash < cost and side == "buy":
                        result.status       = _ORDER_STATUS_REJECTED
                        result.reject_reason = f"insufficient_cash cash={cash:.2f} need={cost:.2f}"
                        logger.warning("PAPER REJECTED %s: %s", symbol, result.reject_reason)
                        self._record_order(conn, result, cash)
                        return result

                    # Abrir o ampliar posición
                    pos_side = "long" if side == "buy" else "short"
                    existing = conn.execute(
                        "SELECT * FROM paper_positions WHERE symbol=?", (symbol,)
                    ).fetchone()

                    if existing and existing["side"] == pos_side:
                        # Promediar precio
                        total_qty = existing["qty"] + qty
                        avg = (existing["avg_price"] * existing["qty"] + fill_price * qty) / total_qty
                        conn.execute(
                            "UPDATE paper_positions SET qty=?,avg_price=?,current_price=? WHERE symbol=?",
                            (total_qty, round(avg, 4), fill_price, symbol)
                        )
                    else:
                        conn.execute(
                            """INSERT OR REPLACE INTO paper_positions
                               VALUES (?,?,?,?,?,?,?,?,?,?,?)""",
                            (symbol, pos_side, qty, fill_price, fill_price,
                             strategy, option_strategy, signal_score, score_tier,
                             ts, order_id)
                        )

                    new_cash = cash - (cost if side == "buy" else 0.0) - commission
                    self._set_cfg(conn, "cash", new_cash)
                    result.status = _ORDER_STATUS_FILLED
                    equity = self._calc_equity(conn, new_cash)
                    self._update_peak(conn, equity)
                    self._record_order(conn, result, equity)
                    self._snap_equity(conn, equity, new_cash, self._get_cfg("realized_pnl"),
                                      f"open_{side}_{symbol}")

                # ── Cierre (sell / buy_to_cover) ───────────────────────────────
                else:
                    close_side = "long" if side == "sell" else "short"
                    existing = conn.execute(
                        "SELECT * FROM paper_positions WHERE symbol=?", (symbol,)
                    ).fetchone()

                    if not existing or existing["side"] != close_side:
                        result.status        = _ORDER_STATUS_REJECTED
                        result.reject_reason = f"no_open_{close_side}_position"
                        self._record_order(conn, result, cash)
                        return result

                    close_qty = min(qty, existing["qty"])
                    if close_side == "long":
                        pnl = round((fill_price - existing["avg_price"]) * close_qty - commission, 2)
                    else:
                        pnl = round((existing["avg_price"] - fill_price) * close_qty - commission, 2)

                    result.pnl    = pnl
                    result.status = _ORDER_STATUS_FILLED

                    remaining = existing["qty"] - close_qty
                    if remaining <= 0:
                        conn.execute("DELETE FROM paper_positions WHERE symbol=?", (symbol,))
                    else:
                        conn.execute(
                            "UPDATE paper_positions SET qty=?,current_price=? WHERE symbol=?",
                            (remaining, fill_price, symbol)
                        )

                    proceeds  = fill_price * close_qty - commission
                    new_cash  = cash + (proceeds if side == "sell" else 0.0) + pnl
                    new_rpnl  = round(self._get_cfg("realized_pnl") + pnl, 2)
                    self._set_cfg(conn, "cash", new_cash)
                    self._set_cfg(conn, "realized_pnl", new_rpnl)

                    equity = self._calc_equity(conn, new_cash)
                    self._update_peak(conn, equity)
                    self._record_order(conn, result, equity)
                    self._snap_equity(conn, equity, new_cash, new_rpnl,
                                      f"close_{side}_{symbol}_pnl={pnl:.2f}")
                    logger.info(
                        "PAPER CLOSE %s qty=%d pnl=%.2f equity=%.2f",
                        symbol, close_qty, pnl, equity,
                    )

            logger.info(
                "PAPER FILL %s %s qty=%d @%.2f commission=%.2f status=%s",
                side.upper(), symbol, qty, fill_price, commission, result.status,
            )
            return result

    # ── Price update ───────────────────────────────────────────────────────────

    def update_price(self, symbol: str, price: float) -> None:
        """Actualiza el current_price de una posición abierta (para unrealized P&L)."""
        with self._lock:
            with self._conn() as conn:
                conn.execute(
                    "UPDATE paper_positions SET current_price=? WHERE symbol=?",
                    (round(price, 4), symbol)
                )

    # ── Queries públicas ───────────────────────────────────────────────────────

    def get_positions(self) -> list[dict]:
        with self._conn() as conn:
            rows = conn.execute("SELECT * FROM paper_positions").fetchall()
        result = []
        for r in rows:
            d = dict(r)
            mult = 1.0 if d["side"] == "long" else -1.0
            d["unrealized_pnl"]     = round(mult * (d["current_price"] - d["avg_price"]) * d["qty"], 2)
            d["unrealized_pnl_pct"] = round(d["unrealized_pnl"] / (d["avg_price"] * d["qty"]) * 100, 3) if d["avg_price"] > 0 else 0.0
            d["market_value"]       = round(d["current_price"] * d["qty"], 2)
            result.append(d)
        return result

    def get_orders(self, limit: int = 100) -> list[dict]:
        with self._conn() as conn:
            rows = conn.execute(
                "SELECT * FROM paper_orders ORDER BY ts DESC LIMIT ?", (limit,)
            ).fetchall()
        return [dict(r) for r in rows]

    def get_equity_curve(self, limit: int = 500) -> list[dict]:
        with self._conn() as conn:
            rows = conn.execute(
                "SELECT * FROM paper_equity_curve ORDER BY ts ASC"
                " LIMIT (SELECT MAX(COUNT(*), ?) FROM paper_equity_curve)",
                (limit,)
            ).fetchall()
        return [dict(r) for r in rows]

    def get_equity_curve_recent(self, limit: int = 500) -> list[dict]:
        """Últimos N puntos de la equity curve, ordenados cronológicamente."""
        with self._conn() as conn:
            rows = conn.execute(
                "SELECT * FROM (SELECT * FROM paper_equity_curve ORDER BY ts DESC LIMIT ?) "
                "ORDER BY ts ASC",
                (limit,)
            ).fetchall()
        return [dict(r) for r in rows]

    def get_account_summary(self) -> PaperAccountSummary:
        with self._lock:
            with self._conn() as conn:
                cash      = self._get_cfg("cash")
                init_cap  = self._get_cfg("initial_capital")
                rpnl      = self._get_cfg("realized_pnl")
                peak      = self._get_cfg("peak_equity")

                positions = conn.execute("SELECT * FROM paper_positions").fetchall()
                unrealized = sum(
                    (1.0 if r["side"] == "long" else -1.0)
                    * (r["current_price"] - r["avg_price"]) * r["qty"]
                    for r in positions
                )
                equity = round(cash + unrealized, 2)

                # Stats de trades cerrados
                orders = conn.execute(
                    "SELECT pnl, status FROM paper_orders WHERE status='filled'"
                ).fetchall()
                closed = [o["pnl"] for o in orders if o["pnl"] != 0.0]
                wins   = sum(1 for p in closed if p > 0)
                losses = sum(1 for p in closed if p < 0)
                total  = wins + losses

                max_dd = 0.0
                if peak > 0 and equity < peak:
                    max_dd = round((peak - equity) / peak * 100, 2)

            return PaperAccountSummary(
                initial_capital  = init_cap,
                cash             = round(cash, 2),
                equity           = equity,
                unrealized_pnl   = round(unrealized, 2),
                realized_pnl     = round(rpnl, 2),
                total_pnl        = round(equity - init_cap, 2),
                total_pnl_pct    = round((equity - init_cap) / init_cap * 100, 3) if init_cap > 0 else 0.0,
                total_trades     = total,
                winning_trades   = wins,
                losing_trades    = losses,
                win_rate_pct     = round(wins / total * 100, 1) if total > 0 else 0.0,
                max_drawdown_pct = max_dd,
                peak_equity      = round(peak, 2),
                open_positions   = len(positions),
                last_updated     = datetime.now(timezone.utc).isoformat(),
            )

    # ── Reset ──────────────────────────────────────────────────────────────────

    def reset(self, new_capital: float | None = None) -> dict:
        """Resetea la cuenta al capital inicial (o a new_capital si se especifica).

        El historial completo se conserva en paper_orders; solo se borra el
        estado activo (posiciones, equity curve, config).
        """
        with self._lock:
            capital = new_capital if new_capital and new_capital > 0 else self.initial_capital
            with self._conn() as conn:
                conn.execute("DELETE FROM paper_positions")
                conn.execute("DELETE FROM paper_equity_curve")
                conn.execute("DELETE FROM paper_config")
                # Reinsertar config limpia
                for k, v in [
                    ("initial_capital", capital),
                    ("cash",            capital),
                    ("realized_pnl",    0.0),
                    ("peak_equity",     capital),
                ]:
                    conn.execute("INSERT INTO paper_config VALUES (?,?)", (k, str(v)))
                conn.execute(
                    "INSERT INTO paper_equity_curve VALUES (?,?,?,?,'account_reset')",
                    (time.time(), capital, capital, 0.0)
                )
            logger.info("PaperBroker RESET — capital=%.2f", capital)
            return {"reset": True, "initial_capital": capital}

    # ── Helpers internos ───────────────────────────────────────────────────────

    def _calc_equity(self, conn: sqlite3.Connection, cash: float) -> float:
        positions = conn.execute("SELECT * FROM paper_positions").fetchall()
        unrealized = sum(
            (1.0 if r["side"] == "long" else -1.0)
            * (r["current_price"] - r["avg_price"]) * r["qty"]
            for r in positions
        )
        return round(cash + unrealized, 2)

    def _update_peak(self, conn: sqlite3.Connection, equity: float) -> None:
        peak = float(conn.execute(
            "SELECT value FROM paper_config WHERE key='peak_equity'"
        ).fetchone()["value"])
        if equity > peak:
            self._set_cfg(conn, "peak_equity", equity)

    def _record_order(
        self,
        conn: sqlite3.Connection,
        result: PaperOrderResult,
        equity_after: float,
    ) -> None:
        conn.execute(
            """INSERT OR IGNORE INTO paper_orders
               VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
            (
                result.order_id, result.symbol, result.side,
                result.qty, result.fill_price, result.commission,
                result.status, result.reject_reason, result.pnl,
                result.strategy, result.option_strategy,
                result.signal_score, result.score_tier,
                result.timestamp, equity_after,
            )
        )

    def _snap_equity(
        self,
        conn: sqlite3.Connection,
        equity: float,
        cash: float,
        realized_pnl: float,
        event: str,
    ) -> None:
        conn.execute(
            "INSERT OR IGNORE INTO paper_equity_curve VALUES (?,?,?,?,?)",
            (time.time(), equity, cash, realized_pnl, event)
        )


# ── Singleton global ───────────────────────────────────────────────────────────

_broker_instance: Optional[PaperBroker] = None
_broker_lock = threading.Lock()


def get_paper_broker(
    initial_capital: float = DEFAULT_CAPITAL,
    db_path: Path = _DB_PATH,
) -> PaperBroker:
    """Retorna la instancia singleton de PaperBroker."""
    global _broker_instance
    if _broker_instance is None:
        with _broker_lock:
            if _broker_instance is None:
                _broker_instance = PaperBroker(
                    db_path=db_path,
                    initial_capital=initial_capital,
                )
    return _broker_instance
