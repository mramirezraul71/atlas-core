"""Plan de gráficos del selector (URLs TradingView/Yahoo) — import estable desde startup y tests."""
from __future__ import annotations

from typing import Any


def chart_interval_for_timeframe(timeframe: str) -> str:
    return {
        "5m": "5",
        "15m": "15",
        "1h": "60",
        "4h": "240",
        "1d": "D",
    }.get(str(timeframe or "").lower(), "60")


def build_selector_chart_plan(
    symbol: str,
    timeframe: str,
    higher_timeframe: str | None,
    chart_provider: str,
) -> dict[str, Any]:
    """Misma estructura que antes en strategy_selector._chart_plan (única fuente de verdad)."""
    provider = str(chart_provider or "tradingview").lower()
    base_symbol = str(symbol or "").upper()
    primary_interval = chart_interval_for_timeframe(timeframe)
    higher_interval = chart_interval_for_timeframe(higher_timeframe or "1h")
    overview_interval = "D"
    if provider == "yahoo":
        primary_url = f"https://finance.yahoo.com/quote/{base_symbol}/chart"
        higher_url = primary_url
        overview_url = primary_url
    else:
        primary_url = f"https://www.tradingview.com/chart/?symbol=NASDAQ%3A{base_symbol}&interval={primary_interval}"
        higher_url = f"https://www.tradingview.com/chart/?symbol=NASDAQ%3A{base_symbol}&interval={higher_interval}"
        overview_url = f"https://www.tradingview.com/chart/?symbol=NASDAQ%3A{base_symbol}&interval={overview_interval}"
    return {
        "provider": provider,
        "auto_open_supported": True,
        "targets": [
            {
                "title": f"{base_symbol} disparo {timeframe}",
                "timeframe": timeframe,
                "url": primary_url,
            },
            {
                "title": f"{base_symbol} confirmacion {higher_timeframe or '1h'}",
                "timeframe": higher_timeframe or "1h",
                "url": higher_url,
            },
            {
                "title": f"{base_symbol} contexto diario",
                "timeframe": "1d",
                "url": overview_url,
            },
        ],
        "steps": [
            "abrir grafico principal del setup",
            "abrir grafico de confirmacion superior",
            "abrir grafico de contexto diario",
            "centrar la zona de entrada en pantalla",
            "validar con la camara antes del ticket",
        ],
    }
