"""Configuración de sesión paper trading (validación multi-día).

No sustituye a config.settings; documenta objetivos y umbrales de la sesión.
"""

from __future__ import annotations

PAPER_TRADING_CONFIG: dict = {
    "mode": "paper",
    "enable_all_gates": True,
    "start_date": "2026-04-15",
    "duration_days": 7,
    "expected_end_date": "2026-04-22",
    "validate_market_hours": True,
    "validate_broker_order_ids": True,
    "validate_reconciliation": True,
    "max_trades_per_day": 100,
    "max_intraday_exposure_pct": 0.5,
    "max_intraday_drawdown_pct": 0.10,
    "loss_cooldown_threshold": 3,
    "monitoring_interval_sec": 60,
    "report_interval_hours": 1,
    "daily_report_time": "16:00",
    "alert_on_blocker_failure": True,
    "alert_on_phantom": True,
    "alert_on_broker_order_id_empty": True,
    "alert_on_trades_outside_hours": True,
}

DAILY_METRICS_TEMPLATE: dict = {
    "total_trades": 0,
    "trades_outside_hours": 0,
    "broker_order_id_empty": 0,
    "phantoms_detected": 0,
    "reconciliation_failed": 0,
    "gates_blocks": {},
    "pnl_daily": 0.0,
    "drawdown_intraday": 0.0,
    "learning_enabled": True,
}
