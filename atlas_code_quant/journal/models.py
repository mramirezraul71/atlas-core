"""ORM models for the trading journal."""
from __future__ import annotations

from datetime import datetime

from sqlalchemy import Boolean, DateTime, Float, Integer, String, Text
from sqlalchemy.orm import Mapped, mapped_column

from journal.db import Base


class TradingJournal(Base):
    __tablename__ = "trading_journal"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    journal_key: Mapped[str] = mapped_column(String(128), unique=True, index=True)
    account_type: Mapped[str] = mapped_column(String(16), index=True)
    account_id: Mapped[str] = mapped_column(String(32), index=True)
    strategy_id: Mapped[str] = mapped_column(String(128), index=True)
    tracker_strategy_id: Mapped[str | None] = mapped_column(String(160), nullable=True)
    strategy_type: Mapped[str] = mapped_column(String(80), index=True)
    symbol: Mapped[str] = mapped_column(String(64), index=True)
    legs_signature: Mapped[str] = mapped_column(String(160), index=True)
    legs_details: Mapped[str] = mapped_column(Text, default="[]")
    entry_price: Mapped[float | None] = mapped_column(Float, nullable=True)
    exit_price: Mapped[float | None] = mapped_column(Float, nullable=True)
    fees: Mapped[float] = mapped_column(Float, default=0.0)
    win_rate_at_entry: Mapped[float | None] = mapped_column(Float, nullable=True)
    current_win_rate_pct: Mapped[float | None] = mapped_column(Float, nullable=True)
    iv_rank: Mapped[float | None] = mapped_column(Float, nullable=True)
    thesis_rich_text: Mapped[str] = mapped_column(Text, default="")
    entry_time: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow, index=True)
    exit_time: Mapped[datetime | None] = mapped_column(DateTime, nullable=True, index=True)
    status: Mapped[str] = mapped_column(String(16), default="open", index=True)
    is_level4: Mapped[bool] = mapped_column(Boolean, default=False)
    realized_pnl: Mapped[float] = mapped_column(Float, default=0.0)
    unrealized_pnl: Mapped[float] = mapped_column(Float, default=0.0)
    mark_price: Mapped[float | None] = mapped_column(Float, nullable=True)
    spot_price: Mapped[float | None] = mapped_column(Float, nullable=True)
    entry_notional: Mapped[float | None] = mapped_column(Float, nullable=True)
    risk_at_entry: Mapped[float | None] = mapped_column(Float, nullable=True)
    greeks_json: Mapped[str] = mapped_column(Text, default="{}")
    attribution_json: Mapped[str] = mapped_column(Text, default="{}")
    post_mortem_json: Mapped[str] = mapped_column(Text, default="{}")
    post_mortem_text: Mapped[str] = mapped_column(Text, default="")
    broker_order_ids_json: Mapped[str] = mapped_column(Text, default="[]")
    raw_entry_payload_json: Mapped[str] = mapped_column(Text, default="{}")
    raw_exit_payload_json: Mapped[str] = mapped_column(Text, default="{}")
    created_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)
    updated_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    last_synced_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow, index=True)
