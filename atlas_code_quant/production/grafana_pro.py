# ATLAS-Quant — Generador de Dashboard Grafana PRO 2026
"""Dashboard profesional dark-trading para ATLAS-Quant-Core.

Tema: negro profundo #0A0A0A + verde #00FF9D + rojo #FF3B5C
12 paneles optimizados para monitoreo de trading autónomo real.
"""
from __future__ import annotations
import json
from pathlib import Path

_DARK_BG     = "#0A0A0A"
_PANEL_BG    = "#111318"
_GREEN       = "#00FF9D"
_RED         = "#FF3B5C"
_YELLOW      = "#FFD60A"
_BLUE        = "#00C6FF"
_PURPLE      = "#A855F7"
_TEXT        = "#E8E8E8"
_TEXT_DIM    = "#6B7280"
_GRID        = "#1C2130"

_DS = {"type": "prometheus", "uid": "atlas-prom"}


def _ts_panel(pid, title, exprs, unit="short", y=0, x=0, w=12, h=9,
              fill=10, line_w=2, gradient=False, colors=None) -> dict:
    targets = []
    for i, (expr, legend) in enumerate(exprs):
        targets.append({"datasource": _DS, "expr": expr,
                        "legendFormat": legend, "refId": chr(65+i)})
    over = []
    if colors:
        for i, c in enumerate(colors):
            over.append({"matcher": {"id": "bySeriesIndex", "options": i},
                         "properties": [{"id": "color",
                                         "value": {"fixedColor": c, "mode": "fixed"}}]})
    grad_mode = "opacity" if gradient else "none"
    return {
        "id": pid, "title": title, "type": "timeseries",
        "gridPos": {"x": x, "y": y, "w": w, "h": h},
        "datasource": _DS, "targets": targets,
        "fieldConfig": {
            "defaults": {
                "unit": unit, "color": {"mode": "fixed", "fixedColor": _GREEN},
                "custom": {
                    "lineWidth": line_w, "fillOpacity": fill,
                    "gradientMode": grad_mode,
                    "lineInterpolation": "smooth",
                    "showPoints": "never",
                    "axisColorMode": "text",
                    "axisBorderShow": False,
                    "thresholdsStyle": {"mode": "off"},
                },
            },
            "overrides": over,
        },
        "options": {
            "tooltip": {"mode": "multi", "sort": "desc"},
            "legend": {"displayMode": "list", "placement": "bottom",
                       "calcs": ["lastNotNull", "max", "min"]},
        },
    }


def _stat(pid, title, expr, unit="short", y=0, x=0, w=4, h=4,
          color=_GREEN, thresholds=None) -> dict:
    steps = thresholds or [{"color": _GREEN, "value": None}]
    return {
        "id": pid, "title": title, "type": "stat",
        "gridPos": {"x": x, "y": y, "w": w, "h": h},
        "datasource": _DS,
        "targets": [{"datasource": _DS, "expr": expr,
                     "legendFormat": "", "refId": "A"}],
        "fieldConfig": {
            "defaults": {
                "unit": unit,
                "color": {"mode": "fixed", "fixedColor": color},
                "thresholds": {"mode": "absolute", "steps": steps},
                "mappings": [],
            }
        },
        "options": {
            "reduceOptions": {"calcs": ["lastNotNull"], "fields": "", "values": False},
            "orientation": "auto", "textMode": "value_and_name",
            "colorMode": "background", "graphMode": "area",
            "justifyMode": "center",
        },
    }


def _gauge(pid, title, expr, unit="percent", y=0, x=0, w=8, h=9,
           min_v=0, max_v=100, thresholds=None) -> dict:
    steps = thresholds or [
        {"color": _RED,    "value": None},
        {"color": _YELLOW, "value": 30},
        {"color": _GREEN,  "value": 65},
    ]
    return {
        "id": pid, "title": title, "type": "gauge",
        "gridPos": {"x": x, "y": y, "w": w, "h": h},
        "datasource": _DS,
        "targets": [{"datasource": _DS, "expr": expr,
                     "legendFormat": "", "refId": "A"}],
        "fieldConfig": {
            "defaults": {
                "unit": unit, "min": min_v, "max": max_v,
                "thresholds": {"mode": "absolute", "steps": steps},
                "color": {"mode": "thresholds"},
                "custom": {"neutral": 0},
            }
        },
        "options": {
            "reduceOptions": {"calcs": ["lastNotNull"]},
            "orientation": "auto", "showThresholdLabels": True,
            "showThresholdMarkers": True, "text": {},
        },
    }


def _table(pid, title, exprs, y=0, x=0, w=12, h=9) -> dict:
    targets = [{"datasource": _DS, "expr": e, "legendFormat": l,
                "refId": chr(65+i), "instant": True}
               for i, (e, l) in enumerate(exprs)]
    return {
        "id": pid, "title": title, "type": "table",
        "gridPos": {"x": x, "y": y, "w": w, "h": h},
        "datasource": _DS, "targets": targets,
        "fieldConfig": {
            "defaults": {
                "color": {"mode": "thresholds"},
                "custom": {"align": "auto", "cellOptions": {"type": "color-background"},
                           "filterable": True},
                "thresholds": {"mode": "absolute", "steps": [
                    {"color": _RED, "value": None},
                    {"color": _GREEN, "value": 0},
                ]},
            }
        },
        "options": {
            "cellHeight": "sm", "footer": {"show": False},
            "showHeader": True, "sortBy": [],
        },
        "transformations": [{"id": "merge", "options": {}}],
    }


def _row(pid, title, y=0) -> dict:
    return {
        "id": pid, "title": title, "type": "row",
        "gridPos": {"x": 0, "y": y, "w": 24, "h": 1},
        "collapsed": False, "panels": [],
    }


def generate_pro_dashboard() -> dict:
    """Genera el dashboard profesional ATLAS PRO 2026."""

    panels = []

    # ══════════════════════════════════════════════════════════════════════
    # HEADER ROW — Stats rápidos (y=0, h=4)
    # ══════════════════════════════════════════════════════════════════════
    panels.append(_row(1, "📊  ATLAS-QUANT  ·  ESTADO EN TIEMPO REAL", y=0))

    panels.append(_stat(2, "⚡ EQUITY", "atlas_equity_usd",
                        unit="currencyUSD", y=1, x=0, w=4, h=4, color=_GREEN,
                        thresholds=[{"color": _RED, "value": None},
                                    {"color": _GREEN, "value": 90_000}]))
    panels.append(_stat(3, "📉 DRAWDOWN", "atlas_drawdown_pct",
                        unit="percent", y=1, x=4, w=4, h=4, color=_YELLOW,
                        thresholds=[{"color": _GREEN, "value": None},
                                    {"color": _YELLOW, "value": 3},
                                    {"color": _RED,    "value": 7}]))
    panels.append(_stat(4, "💰 PnL HOY", "atlas_daily_pnl_usd",
                        unit="currencyUSD", y=1, x=8, w=4, h=4, color=_BLUE))
    panels.append(_stat(5, "📈 SHARPE", "atlas_sharpe_ratio",
                        unit="short", y=1, x=12, w=4, h=4, color=_PURPLE,
                        thresholds=[{"color": _RED,    "value": None},
                                    {"color": _YELLOW, "value": 1.0},
                                    {"color": _GREEN,  "value": 1.5}]))
    panels.append(_stat(6, "🔵 POSICIONES", "atlas_open_positions",
                        unit="short", y=1, x=16, w=4, h=4, color=_BLUE))
    panels.append(_stat(7, "🔴 MODO",
                        'atlas_mode == 1 ? 1 : 0',
                        unit="short", y=1, x=20, w=4, h=4,
                        color=_RED,
                        thresholds=[{"color": _GREEN, "value": None},
                                    {"color": _RED, "value": 1}]))

    # ══════════════════════════════════════════════════════════════════════
    # ROW 2 — Equity Curve + Regime Gauge (y=5)
    # ══════════════════════════════════════════════════════════════════════
    panels.append(_row(8, "💹  EQUITY & RÉGIMEN ML", y=5))

    panels.append(_ts_panel(
        9, "Curva de Equity (USD)",
        [("atlas_equity_usd", "Equity"),
         ("atlas_equity_usd * (1 - atlas_drawdown_pct/100)", "Drawdown baseline")],
        unit="currencyUSD", y=6, x=0, w=16, h=10,
        fill=20, gradient=True,
        colors=[_GREEN, _RED],
    ))

    panels.append(_gauge(
        10, "Régimen ML",
        "atlas_regime",
        unit="short", y=6, x=16, w=8, h=10,
        min_v=0, max_v=2,
        thresholds=[
            {"color": _RED,    "value": None},    # 0 = Bear
            {"color": _YELLOW, "value": 0.9},     # 1 = Sideways
            {"color": _GREEN,  "value": 1.8},     # 2 = Bull
        ]
    ))

    # ══════════════════════════════════════════════════════════════════════
    # ROW 3 — Drawdown + Sharpe Trend (y=16)
    # ══════════════════════════════════════════════════════════════════════
    panels.append(_row(11, "⚠️  RIESGO: DRAWDOWN & SHARPE", y=16))

    panels.append(_ts_panel(
        12, "Drawdown Waterfall (%)",
        [("atlas_drawdown_pct", "Drawdown actual"),
         ("8", "Límite crítico 8%"),
         ("2", "Límite diario 2%")],
        unit="percent", y=17, x=0, w=12, h=9,
        fill=30, colors=[_RED, "#FF3B5C66", "#FFD60A66"],
    ))

    panels.append(_ts_panel(
        13, "Sharpe Ratio Tendencia",
        [("atlas_sharpe_ratio", "Sharpe rolling 20"),
         ("1.5", "Objetivo mínimo live"),
         ("0", "Breakeven")],
        unit="short", y=17, x=12, w=12, h=9,
        fill=10, colors=[_PURPLE, _GREEN + "55", _TEXT_DIM],
    ))

    # ══════════════════════════════════════════════════════════════════════
    # ROW 4 — IV Rank + OCR + Kelly (y=26)
    # ══════════════════════════════════════════════════════════════════════
    panels.append(_row(14, "🎯  INDICADORES: IV RANK · OCR · KELLY", y=26))

    panels.append({
        "id": 15, "title": "IV Rank por Símbolo",
        "type": "timeseries",
        "gridPos": {"x": 0, "y": 27, "w": 8, "h": 9},
        "datasource": _DS,
        "targets": [
            {"datasource": _DS, "expr": "atlas_iv_rank",
             "legendFormat": "IV Rank", "refId": "A"},
            {"datasource": _DS, "expr": "70",
             "legendFormat": "Umbral señal (70)", "refId": "B"},
        ],
        "fieldConfig": {
            "defaults": {
                "unit": "percent", "min": 0, "max": 100,
                "color": {"mode": "continuous-RdYlGn"},
                "custom": {"lineWidth": 2, "fillOpacity": 25,
                           "lineInterpolation": "smooth",
                           "gradientMode": "scheme"},
                "thresholds": {"mode": "absolute", "steps": [
                    {"color": _GREEN, "value": None},
                    {"color": _YELLOW, "value": 50},
                    {"color": _RED,    "value": 80},
                ]},
            }
        },
        "options": {"tooltip": {"mode": "multi"},
                    "legend": {"displayMode": "list", "placement": "bottom"}},
    })

    panels.append({
        "id": 16, "title": "OCR Precisión + Latencia Cámara",
        "type": "timeseries",
        "gridPos": {"x": 8, "y": 27, "w": 8, "h": 9},
        "datasource": _DS,
        "targets": [
            {"datasource": _DS, "expr": "atlas_ocr_confidence_pct",
             "legendFormat": "Precisión OCR (%)", "refId": "A"},
            {"datasource": _DS, "expr": "atlas_cycle_ms / 10",
             "legendFormat": "Latencia ciclo (×10ms)", "refId": "B"},
        ],
        "fieldConfig": {
            "defaults": {
                "unit": "percent",
                "color": {"mode": "palette-classic"},
                "custom": {"lineWidth": 2, "fillOpacity": 10,
                           "lineInterpolation": "smooth"},
                "thresholds": {"mode": "absolute", "steps": [
                    {"color": _RED,    "value": None},
                    {"color": _YELLOW, "value": 80},
                    {"color": _GREEN,  "value": 92},
                ]},
            }
        },
        "options": {"tooltip": {"mode": "multi"},
                    "legend": {"displayMode": "list", "placement": "bottom"}},
    })

    panels.append({
        "id": 17, "title": "Kelly Allocation",
        "type": "piechart",
        "gridPos": {"x": 16, "y": 27, "w": 8, "h": 9},
        "datasource": _DS,
        "targets": [
            {"datasource": _DS,
             "expr": "atlas_open_positions > 0 ? atlas_equity_usd * 0.012 : 0",
             "legendFormat": "En posiciones", "refId": "A", "instant": True},
            {"datasource": _DS,
             "expr": "atlas_equity_usd * (1 - 0.012 * atlas_open_positions)",
             "legendFormat": "Capital libre", "refId": "B", "instant": True},
        ],
        "fieldConfig": {
            "defaults": {
                "unit": "currencyUSD",
                "color": {"mode": "palette-classic"},
            },
            "overrides": [
                {"matcher": {"id": "byName", "options": "En posiciones"},
                 "properties": [{"id": "color",
                                 "value": {"fixedColor": _GREEN, "mode": "fixed"}}]},
                {"matcher": {"id": "byName", "options": "Capital libre"},
                 "properties": [{"id": "color",
                                 "value": {"fixedColor": _GRID, "mode": "fixed"}}]},
            ]
        },
        "options": {
            "reduceOptions": {"calcs": ["lastNotNull"]},
            "pieType": "donut", "tooltip": {"mode": "single"},
            "legend": {"displayMode": "list", "placement": "right"},
            "displayLabels": ["percent", "name"],
        },
    })

    # ══════════════════════════════════════════════════════════════════════
    # ROW 5 — CVD + Volume / Self-Healing (y=36)
    # ══════════════════════════════════════════════════════════════════════
    panels.append(_row(18, "🔬  CVD · VOLUMEN · SELF-HEALING", y=36))

    panels.append(_ts_panel(
        19, "CVD + Volume Spike Alert",
        [("rate(atlas_trades_total[1m])*60", "Trades/min"),
         ("atlas_cycle_ms", "Latencia ciclo (ms)")],
        unit="short", y=37, x=0, w=12, h=9,
        fill=15, colors=[_BLUE, _YELLOW],
    ))

    panels.append({
        "id": 20, "title": "Self-Healing Events",
        "type": "timeseries",
        "gridPos": {"x": 12, "y": 37, "w": 12, "h": 9},
        "datasource": _DS,
        "targets": [
            {"datasource": _DS, "expr": "atlas_cpu_pct",
             "legendFormat": "CPU Jetson (%)", "refId": "A"},
            {"datasource": _DS, "expr": "80",
             "legendFormat": "Throttle OCR (80%)", "refId": "B"},
        ],
        "fieldConfig": {
            "defaults": {
                "unit": "percent", "min": 0, "max": 100,
                "color": {"mode": "fixed", "fixedColor": _YELLOW},
                "custom": {"lineWidth": 2, "fillOpacity": 20,
                           "lineInterpolation": "smooth"},
                "thresholds": {"mode": "absolute", "steps": [
                    {"color": _GREEN,  "value": None},
                    {"color": _YELLOW, "value": 70},
                    {"color": _RED,    "value": 85},
                ]},
            },
            "overrides": [
                {"matcher": {"id": "byName", "options": "Throttle OCR (80%)"},
                 "properties": [
                     {"id": "color",
                      "value": {"fixedColor": "#FF3B5C55", "mode": "fixed"}},
                     {"id": "custom.lineWidth", "value": 1},
                     {"id": "custom.fillOpacity", "value": 0},
                 ]},
            ]
        },
        "options": {"tooltip": {"mode": "multi"},
                    "legend": {"displayMode": "list", "placement": "bottom"}},
    })

    # ══════════════════════════════════════════════════════════════════════
    # ROW 6 — Positions Table + Robot Status (y=46)
    # ══════════════════════════════════════════════════════════════════════
    panels.append(_row(21, "🤖  POSICIONES VIVAS · ESTADO DEL ROBOT", y=46))

    panels.append(_table(
        22, "Posiciones Abiertas (Live)",
        [("atlas_equity_usd",      "Equity USD"),
         ("atlas_open_positions",  "Posiciones"),
         ("atlas_daily_pnl_usd",   "PnL Diario"),
         ("atlas_drawdown_pct",    "Drawdown %"),
         ("atlas_sharpe_ratio",    "Sharpe")],
        y=47, x=0, w=12, h=9,
    ))

    panels.append({
        "id": 23, "title": "Estado del Robot (Insta360 + HID + Sistema)",
        "type": "stat",
        "gridPos": {"x": 12, "y": 47, "w": 12, "h": 9},
        "datasource": _DS,
        "targets": [
            {"datasource": _DS, "expr": "atlas_ocr_confidence_pct",
             "legendFormat": "📷 OCR Confianza %", "refId": "A", "instant": True},
            {"datasource": _DS, "expr": "atlas_cpu_pct",
             "legendFormat": "🖥 CPU Jetson %",    "refId": "B", "instant": True},
            {"datasource": _DS, "expr": "atlas_cycle_ms",
             "legendFormat": "⚡ Ciclo ms",         "refId": "C", "instant": True},
            {"datasource": _DS, "expr": "atlas_mode",
             "legendFormat": "🔴 Modo Live",        "refId": "D", "instant": True},
            {"datasource": _DS, "expr": "atlas_regime",
             "legendFormat": "📊 Régimen",          "refId": "E", "instant": True},
        ],
        "fieldConfig": {
            "defaults": {
                "color": {"mode": "thresholds"},
                "thresholds": {"mode": "absolute", "steps": [
                    {"color": _RED, "value": None},
                    {"color": _GREEN, "value": 1},
                ]},
                "custom": {},
            }
        },
        "options": {
            "reduceOptions": {"calcs": ["lastNotNull"], "fields": "", "values": False},
            "orientation": "horizontal", "textMode": "value_and_name",
            "colorMode": "background", "graphMode": "none",
            "justifyMode": "auto",
        },
    })

    # ══════════════════════════════════════════════════════════════════════
    # DASHBOARD CONFIG
    # ══════════════════════════════════════════════════════════════════════
    return {
        "uid":           "atlas-quant-pro-2026",
        "title":         "⚡ ATLAS-QUANT PRO 2026 — Robot de Trading Autónomo",
        "description":   "Dashboard profesional de monitoreo ATLAS-Quant-Core v1.0 | Jetson Orin Nano",
        "tags":          ["atlas", "trading", "quant", "live", "production"],
        "timezone":      "America/New_York",
        "schemaVersion": 38,
        "version":       1,
        "refresh":       "5s",
        "time":          {"from": "now-4h", "to": "now"},
        "timepicker":    {"refresh_intervals": ["5s", "15s", "30s", "1m", "5m"]},
        "fiscalYearStartMonth": 0,
        "graphTooltip": 1,
        "panels":        panels,
        "templating": {
            "list": [
                {
                    "name":    "symbol",
                    "label":   "Símbolo",
                    "type":    "custom",
                    "current": {"text": "SPY", "value": "SPY"},
                    "options": [
                        {"text": s, "value": s, "selected": s == "SPY"}
                        for s in ["SPY", "QQQ", "AAPL", "TSLA", "NVDA", "MSFT", "AMZN"]
                    ],
                    "query":   "SPY,QQQ,AAPL,TSLA,NVDA,MSFT,AMZN",
                    "hide":    0,
                    "multi":   False,
                },
                {
                    "name":    "mode",
                    "label":   "Modo",
                    "type":    "custom",
                    "current": {"text": "live", "value": "1"},
                    "options": [
                        {"text": "paper", "value": "0", "selected": False},
                        {"text": "live",  "value": "1", "selected": True},
                    ],
                    "query":   "paper : 0, live : 1",
                    "hide":    0,
                    "multi":   False,
                },
            ]
        },
        "annotations": {
            "list": [
                {
                    "builtIn":    1,
                    "datasource": {"type": "grafana", "uid": "-- Grafana --"},
                    "enable":     True,
                    "hide":       True,
                    "iconColor":  _RED,
                    "name":       "Emergency Stops",
                    "type":       "dashboard",
                },
            ]
        },
        "links": [
            {"title": "📊 Atlas API",
             "url":   "http://localhost:8792/docs",
             "type":  "link", "icon": "external link",
             "targetBlank": True},
            {"title": "🔴 Emergency Stop",
             "url":   "http://localhost:8792/api/trading/emergency_stop",
             "type":  "link", "icon": "bolt",
             "targetBlank": False},
        ],
        "style": "dark",
        "__inputs": [
            {"name": "DS_ATLAS_PROMETHEUS", "label": "atlas-prometheus",
             "description": "", "type": "datasource",
             "pluginId": "prometheus", "pluginName": "Prometheus"},
        ],
        "__requires": [
            {"type": "grafana",    "id": "grafana",    "name": "Grafana",    "version": "10.0.0"},
            {"type": "datasource", "id": "prometheus", "name": "Prometheus", "version": "1.0.0"},
            {"type": "panel",      "id": "timeseries", "name": "Time series", "version": ""},
            {"type": "panel",      "id": "stat",       "name": "Stat",        "version": ""},
            {"type": "panel",      "id": "gauge",      "name": "Gauge",       "version": ""},
            {"type": "panel",      "id": "piechart",   "name": "Pie chart",   "version": ""},
            {"type": "panel",      "id": "table",      "name": "Table",       "version": ""},
        ],
    }


def save_pro_dashboard(path: Path | None = None) -> Path:
    dest = path or Path("grafana/dashboards/atlas_pro_2026.json")
    dest.parent.mkdir(parents=True, exist_ok=True)
    with open(dest, "w", encoding="utf-8") as f:
        json.dump(generate_pro_dashboard(), f, indent=2, ensure_ascii=False)
    return dest
