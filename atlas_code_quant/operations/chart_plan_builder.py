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


def chart_plan_probe_ok(
    symbol: str,
    timeframe: str,
    chart_provider: str,
) -> tuple[bool, str]:
    """Validación barata para readiness: plan generable con 3 URLs https (TradingView/Yahoo)."""
    sym = str(symbol or "").strip().upper()
    if not sym or len(sym) > 32:
        return False, "símbolo vacío o demasiado largo para chart_plan"
    try:
        plan = build_selector_chart_plan(sym, str(timeframe or "1h"), None, chart_provider)
    except Exception as exc:  # pragma: no cover - build es pura string
        return False, f"build_selector_chart_plan: {exc}"
    targets = plan.get("targets") or []
    if len(targets) < 3:
        return False, "chart_plan incompleto (<3 targets)"
    for t in targets:
        if not isinstance(t, dict):
            return False, "target inválido en chart_plan"
        url = str(t.get("url") or "").strip()
        if not url.startswith("https://"):
            return False, f"URL de gráfico inválida o no https: {url!r}"
    return True, ""


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
