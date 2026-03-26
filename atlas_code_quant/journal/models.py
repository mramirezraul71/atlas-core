"""ORM models for the trading journal."""
from __future__ import annotations

import json
from datetime import datetime
from typing import TYPE_CHECKING

from sqlalchemy import Boolean, DateTime, Float, Integer, String, Text
from sqlalchemy.orm import Mapped, mapped_column

from atlas_code_quant.journal.db import Base

if TYPE_CHECKING:
    from atlas_code_quant.learning.trade_events import TradeEvent


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
    raw_exit_payload_json: Mapped[str] = mapped_column(Text, default="[]")
    created_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)
    updated_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    last_synced_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow, index=True)


# ---------------------------------------------------------------------------
# LearningTradeRecord â€” tabla dedicada para el subsistema AtlasLearningBrain
# ---------------------------------------------------------------------------

class LearningTradeRecord(Base):
    """Registro de trade cerrado para el subsistema de aprendizaje.

    Tabla separada de TradingJournal para no afectar el schema existente.
    Almacena los campos en R-mÃºltiplos necesarios para MetricsEngine y ML.
    """

    __tablename__ = "learning_trade_records"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    trade_id: Mapped[str] = mapped_column(String(128), unique=True, index=True)
    symbol: Mapped[str] = mapped_column(String(64), index=True)
    asset_class: Mapped[str] = mapped_column(String(32), index=True)
    side: Mapped[str] = mapped_column(String(8))

    entry_time: Mapped[datetime] = mapped_column(DateTime, index=True)
    exit_time: Mapped[datetime] = mapped_column(DateTime, index=True)
    timeframe: Mapped[str] = mapped_column(String(16))

    entry_price: Mapped[float] = mapped_column(Float)
    exit_price: Mapped[float] = mapped_column(Float)
    stop_loss_price: Mapped[float] = mapped_column(Float)

    r_initial: Mapped[float] = mapped_column(Float)
    r_realized: Mapped[float] = mapped_column(Float)
    mae_r: Mapped[float] = mapped_column(Float, default=0.0)
    mfe_r: Mapped[float] = mapped_column(Float, default=0.0)

    setup_type: Mapped[str] = mapped_column(String(64), index=True)
    regime: Mapped[str] = mapped_column(String(16), index=True)
    exit_type: Mapped[str] = mapped_column(String(32))

    # Indicadores tÃ©cnicos en entrada
    rsi: Mapped[float] = mapped_column(Float, default=0.0)
    macd_hist: Mapped[float] = mapped_column(Float, default=0.0)
    atr: Mapped[float] = mapped_column(Float, default=0.0)
    bb_pct: Mapped[float] = mapped_column(Float, default=0.0)
    volume_ratio: Mapped[float] = mapped_column(Float, default=1.0)
    cvd: Mapped[float] = mapped_column(Float, default=0.0)
    iv_rank: Mapped[float] = mapped_column(Float, default=0.0)
    iv_hv_ratio: Mapped[float] = mapped_column(Float, default=1.0)

    # Capital y scoring
    capital_at_entry: Mapped[float] = mapped_column(Float, default=100000.0)
    position_size: Mapped[float] = mapped_column(Float, default=0.0)
    signal_score_at_entry: Mapped[float] = mapped_column(Float, default=0.5)
    ml_score_at_entry: Mapped[float] = mapped_column(Float, default=0.5)
    stats_score_at_entry: Mapped[float] = mapped_column(Float, default=0.5)

    # Error flags como JSON array
    error_flags_json: Mapped[str] = mapped_column(Text, default="[]")

    created_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)

    @classmethod
    def from_trade_event(cls, trade: "TradeEvent") -> "LearningTradeRecord":
        """Construye un registro ORM desde un TradeEvent."""
        return cls(
            trade_id=trade.trade_id,
            symbol=trade.symbol,
            asset_class=trade.asset_class,
            side=trade.side,
            entry_time=trade.entry_time,
            exit_time=trade.exit_time,
            timeframe=trade.timeframe,
            entry_price=trade.entry_price,
            exit_price=trade.exit_price,
            stop_loss_price=trade.stop_loss_price,
            r_initial=trade.r_initial,
            r_realized=trade.r_realized,
            mae_r=trade.mae_r,
            mfe_r=trade.mfe_r,
            setup_type=trade.setup_type,
            regime=trade.regime,
            exit_type=trade.exit_type,
            rsi=trade.rsi,
            macd_hist=trade.macd_hist,
            atr=trade.atr,
            bb_pct=trade.bb_pct,
            volume_ratio=trade.volume_ratio,
            cvd=trade.cvd,
            iv_rank=trade.iv_rank,
            iv_hv_ratio=trade.iv_hv_ratio,
            capital_at_entry=trade.capital_at_entry,
            position_size=trade.position_size,
            signal_score_at_entry=trade.signal_score_at_entry,
            ml_score_at_entry=trade.ml_score_at_entry,
            stats_score_at_entry=trade.stats_score_at_entry,
            error_flags_json=json.dumps(trade.error_flags),
        )

    def to_trade_event(self) -> "TradeEvent":
        """Reconstruye un TradeEvent desde el registro ORM."""
        from atlas_code_quant.learning.trade_events import TradeEvent
        return TradeEvent(
            trade_id=self.trade_id,
            symbol=self.symbol,
            asset_class=self.asset_class,
            side=self.side,
            entry_time=self.entry_time,
            exit_time=self.exit_time,
            timeframe=self.timeframe,
            entry_price=self.entry_price,
            exit_price=self.exit_price,
            stop_loss_price=self.stop_loss_price,
            r_initial=self.r_initial,
            r_realized=self.r_realized,
            mae_r=self.mae_r,
            mfe_r=self.mfe_r,
            setup_type=self.setup_type,
            regime=self.regime,
            exit_type=self.exit_type,
            rsi=self.rsi,
            macd_hist=self.macd_hist,
            atr=self.atr,
            bb_pct=self.bb_pct,
            volume_ratio=self.volume_ratio,
            cvd=self.cvd,
            iv_rank=self.iv_rank,
            iv_hv_ratio=self.iv_hv_ratio,
            capital_at_entry=self.capital_at_entry,
            position_size=self.position_size,
            signal_score_at_entry=self.signal_score_at_entry,
            ml_score_at_entry=self.ml_score_at_entry,
            stats_score_at_entry=self.stats_score_at_entry,
            error_flags=json.loads(self.error_flags_json or "[]"),
        )

