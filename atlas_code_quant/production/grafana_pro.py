# ATLAS-Quant - Generador de Dashboard Grafana PRO 2026
"""Dashboard dark-trading para ATLAS-Quant.

Version limpia y mantenible del tablero PRO:
- conserva la API publica `generate_pro_dashboard()` y `save_pro_dashboard()`
- mantiene foco en broker-first, riesgo, sistema y disciplina operativa
- incluye el scorecard de implantacion del proceso de trading
"""
from __future__ import annotations

import json
from pathlib import Path

_DARK_BG = "#0A0A0A"
_PANEL_BG = "#111318"
_GREEN = "#00FF9D"
_RED = "#FF3B5C"
_YELLOW = "#FFD60A"
_BLUE = "#00C6FF"
_PURPLE = "#A855F7"
_TEXT = "#E8E8E8"
_TEXT_DIM = "#6B7280"
_GRID = "#1C2130"

_DS = {"type": "prometheus", "uid": "atlas-prom"}


def _ts_panel(
    pid: int,
    title: str,
    exprs: list[tuple[str, str]],
    *,
    unit: str = "short",
    y: int = 0,
    x: int = 0,
    w: int = 12,
    h: int = 9,
    fill: int = 10,
    line_w: int = 2,
    gradient: bool = False,
    colors: list[str] | None = None,
) -> dict:
    targets = [
        {
            "datasource": _DS,
            "expr": expr,
            "legendFormat": legend,
            "refId": chr(65 + i),
        }
        for i, (expr, legend) in enumerate(exprs)
    ]
    overrides = []
    for i, color in enumerate(colors or []):
        overrides.append(
            {
                "matcher": {"id": "bySeriesIndex", "options": i},
                "properties": [{"id": "color", "value": {"fixedColor": color, "mode": "fixed"}}],
            }
        )
    return {
        "id": pid,
        "title": title,
        "type": "timeseries",
        "gridPos": {"x": x, "y": y, "w": w, "h": h},
        "datasource": _DS,
        "targets": targets,
        "fieldConfig": {
            "defaults": {
                "unit": unit,
                "color": {"mode": "fixed", "fixedColor": _GREEN},
                "custom": {
                    "lineWidth": line_w,
                    "fillOpacity": fill,
                    "gradientMode": "opacity" if gradient else "none",
                    "lineInterpolation": "smooth",
                    "showPoints": "never",
                    "axisColorMode": "text",
                    "axisBorderShow": False,
                    "thresholdsStyle": {"mode": "off"},
                },
            },
            "overrides": overrides,
        },
        "options": {
            "tooltip": {"mode": "multi", "sort": "desc"},
            "legend": {"displayMode": "list", "placement": "bottom", "calcs": ["lastNotNull", "max", "min"]},
        },
    }


def _stat(
    pid: int,
    title: str,
    expr: str,
    *,
    unit: str = "short",
    y: int = 0,
    x: int = 0,
    w: int = 4,
    h: int = 4,
    color: str = _GREEN,
    thresholds: list[dict] | None = None,
) -> dict:
    return {
        "id": pid,
        "title": title,
        "type": "stat",
        "gridPos": {"x": x, "y": y, "w": w, "h": h},
        "datasource": _DS,
        "targets": [{"datasource": _DS, "expr": expr, "legendFormat": "", "refId": "A"}],
        "fieldConfig": {
            "defaults": {
                "unit": unit,
                "color": {"mode": "fixed", "fixedColor": color},
                "thresholds": {"mode": "absolute", "steps": thresholds or [{"color": color, "value": None}]},
                "mappings": [],
            }
        },
        "options": {
            "reduceOptions": {"calcs": ["lastNotNull"], "fields": "", "values": False},
            "orientation": "auto",
            "textMode": "value_and_name",
            "colorMode": "background",
            "graphMode": "area",
            "justifyMode": "center",
        },
    }


def _gauge(
    pid: int,
    title: str,
    expr: str,
    *,
    unit: str = "percent",
    y: int = 0,
    x: int = 0,
    w: int = 8,
    h: int = 9,
    min_v: int = 0,
    max_v: int = 100,
    thresholds: list[dict] | None = None,
) -> dict:
    return {
        "id": pid,
        "title": title,
        "type": "gauge",
        "gridPos": {"x": x, "y": y, "w": w, "h": h},
        "datasource": _DS,
        "targets": [{"datasource": _DS, "expr": expr, "legendFormat": "", "refId": "A"}],
        "fieldConfig": {
            "defaults": {
                "unit": unit,
                "min": min_v,
                "max": max_v,
                "thresholds": {
                    "mode": "absolute",
                    "steps": thresholds
                    or [
                        {"color": _RED, "value": None},
                        {"color": _YELLOW, "value": 30},
                        {"color": _GREEN, "value": 65},
                    ],
                },
                "color": {"mode": "thresholds"},
                "custom": {"neutral": 0},
            }
        },
        "options": {
            "reduceOptions": {"calcs": ["lastNotNull"]},
            "orientation": "auto",
            "showThresholdLabels": True,
            "showThresholdMarkers": True,
            "text": {},
        },
    }


def _table(pid: int, title: str, exprs: list[tuple[str, str]], *, y: int = 0, x: int = 0, w: int = 12, h: int = 9) -> dict:
    targets = [
        {
            "datasource": _DS,
            "expr": expr,
            "legendFormat": legend,
            "refId": chr(65 + i),
            "instant": True,
        }
        for i, (expr, legend) in enumerate(exprs)
    ]
    return {
        "id": pid,
        "title": title,
        "type": "table",
        "gridPos": {"x": x, "y": y, "w": w, "h": h},
        "datasource": _DS,
        "targets": targets,
        "fieldConfig": {
            "defaults": {
                "color": {"mode": "thresholds"},
                "custom": {"align": "auto", "cellOptions": {"type": "color-background"}, "filterable": True},
                "thresholds": {"mode": "absolute", "steps": [{"color": _RED, "value": None}, {"color": _GREEN, "value": 0}]},
            }
        },
        "options": {"cellHeight": "sm", "footer": {"show": False}, "showHeader": True, "sortBy": []},
        "transformations": [{"id": "merge", "options": {}}],
    }


def _row(pid: int, title: str, *, y: int = 0) -> dict:
    return {
        "id": pid,
        "title": title,
        "type": "row",
        "gridPos": {"x": 0, "y": y, "w": 24, "h": 1},
        "collapsed": False,
        "panels": [],
    }


def generate_pro_dashboard() -> dict:
    """Genera el dashboard profesional ATLAS PRO 2026."""
    panels: list[dict] = []

    panels.append(_row(1, "ATLAS-QUANT · ESTADO EN TIEMPO REAL", y=0))
    panels.append(_stat(2, "EQUITY BROKER", 'atlas_broker_equity_usd{source="$source",scope="$scope",account_id=~"$account_id"}', unit="currencyUSD", y=1, x=0, w=4, h=4, color=_GREEN, thresholds=[{"color": _RED, "value": None}, {"color": _GREEN, "value": 90000}]))
    panels.append(_stat(3, "DRAWDOWN", 'atlas_broker_drawdown_pct{source="$source",scope="$scope",account_id=~"$account_id"}', unit="percent", y=1, x=4, w=4, h=4, color=_YELLOW, thresholds=[{"color": _GREEN, "value": None}, {"color": _YELLOW, "value": 3}, {"color": _RED, "value": 7}]))
    panels.append(_stat(4, "PNL ABIERTO", 'atlas_broker_open_pnl_usd{source="$source",scope="$scope",account_id=~"$account_id"}', unit="currencyUSD", y=1, x=8, w=4, h=4, color=_BLUE))
    panels.append(_stat(5, "SYNC STATUS", 'atlas_sync_status{source="$source",scope="$scope",account_id=~"$account_id"}', unit="short", y=1, x=12, w=4, h=4, color=_PURPLE, thresholds=[{"color": _RED, "value": None}, {"color": _YELLOW, "value": 2}, {"color": _GREEN, "value": 3}]))
    panels.append(_stat(6, "POSICIONES", 'atlas_broker_open_positions{source="$source",scope="$scope",account_id=~"$account_id"}', unit="short", y=1, x=16, w=4, h=4, color=_BLUE))
    panels.append(_stat(7, "GAP ATLAS", 'atlas_reconcile_gap_usd{scope="$scope",account_id=~"$account_id",comparison="atlas_internal"}', unit="currencyUSD", y=1, x=20, w=4, h=4, color=_RED, thresholds=[{"color": _GREEN, "value": None}, {"color": _YELLOW, "value": 25}, {"color": _RED, "value": 250}]))

    panels.append(_row(8, "EQUITY Y REGIMEN", y=5))
    panels.append(_ts_panel(9, "Curva de Equity Broker", [('atlas_broker_equity_usd{source="$source",scope="$scope",account_id=~"$account_id"}', "Equity"), ('atlas_broker_open_pnl_usd{source="$source",scope="$scope",account_id=~"$account_id"}', "PnL abierto")], unit="currencyUSD", y=6, x=0, w=16, h=10, fill=20, gradient=True, colors=[_GREEN, _BLUE]))
    panels.append(_gauge(10, "Regimen ML", "atlas_regime", unit="short", y=6, x=16, w=8, h=10, min_v=0, max_v=2, thresholds=[{"color": _RED, "value": None}, {"color": _YELLOW, "value": 0.9}, {"color": _GREEN, "value": 1.8}]))

    panels.append(_row(11, "RIESGO: DRAWDOWN Y SHARPE", y=16))
    panels.append(_ts_panel(12, "Drawdown Waterfall", [('atlas_broker_drawdown_pct{source="$source",scope="$scope",account_id=~"$account_id"}', "Drawdown"), ("8", "Limite critico 8%"), ("2", "Limite diario 2%")], unit="percent", y=17, x=0, w=12, h=9, fill=30, colors=[_RED, "#FF3B5C66", "#FFD60A66"]))
    panels.append(_ts_panel(13, "Sharpe Ratio", [("atlas_sharpe_ratio", "Sharpe rolling 20"), ("1.5", "Objetivo live"), ("0", "Breakeven")], unit="short", y=17, x=12, w=12, h=9, fill=10, colors=[_PURPLE, _GREEN + "55", _TEXT_DIM]))

    panels.append(_row(14, "INDICADORES DE SISTEMA", y=26))
    panels.append(_ts_panel(15, "IV Rank", [("atlas_iv_rank", "IV Rank"), ("70", "Umbral 70")], unit="percent", y=27, x=0, w=8, h=9, fill=25, colors=[_GREEN, _YELLOW]))
    panels.append(_ts_panel(16, "OCR y Latencia", [("atlas_ocr_confidence_pct", "OCR confianza"), ("atlas_cycle_ms / 10", "Latencia x10ms")], unit="percent", y=27, x=8, w=8, h=9, fill=15, colors=[_BLUE, _YELLOW]))
    panels.append(_gauge(17, "CPU Jetson", "atlas_cpu_pct", unit="percent", y=27, x=16, w=8, h=9, thresholds=[{"color": _GREEN, "value": None}, {"color": _YELLOW, "value": 70}, {"color": _RED, "value": 85}]))

    panels.append(_row(18, "ACTIVIDAD OPERATIVA", y=36))
    panels.append(_ts_panel(19, "Trades y Ciclo", [("rate(atlas_trades_total[1m])*60", "Trades/min"), ("atlas_cycle_ms", "Ciclo ms")], unit="short", y=37, x=0, w=12, h=9, fill=15, colors=[_BLUE, _YELLOW]))
    panels.append(_ts_panel(20, "Memoria y Evidencia", [("atlas_brain_delivery_ratio_pct", "Brain delivery"), ("atlas_evidence_sufficiency_score", "Evidence score")], unit="percent", y=37, x=12, w=12, h=9, fill=20, colors=[_PURPLE, _YELLOW]))

    panels.append(_row(21, "POSICIONES Y ESTADO DEL ROBOT", y=46))
    panels.append(_table(22, "Resumen de posiciones", [('atlas_broker_equity_usd{source="$source",scope="$scope",account_id=~"$account_id"}', "Equity USD"), ('atlas_broker_open_positions{source="$source",scope="$scope",account_id=~"$account_id"}', "Posiciones"), ('atlas_broker_open_pnl_usd{source="$source",scope="$scope",account_id=~"$account_id"}', "PnL Abierto"), ('atlas_broker_drawdown_pct{source="$source",scope="$scope",account_id=~"$account_id"}', "Drawdown %"), ('atlas_reconcile_gap_usd{scope="$scope",account_id=~"$account_id",comparison="atlas_internal"}', "Gap Atlas USD")], y=47, x=0, w=12, h=9))
    panels.append(_stat(23, "ROBOT STATUS", "atlas_mode", unit="short", y=47, x=12, w=12, h=9, color=_GREEN, thresholds=[{"color": _RED, "value": None}, {"color": _GREEN, "value": 1}]))

    panels.append(_row(24, "CONTROL INTERNO · SCORECARD DE IMPLANTACION", y=56))
    panels.append(_stat(25, "PROCESS SCORE", "atlas_process_compliance_score", unit="percent", y=57, x=0, w=5, h=5, color=_GREEN, thresholds=[{"color": _RED, "value": None}, {"color": _YELLOW, "value": 50}, {"color": _GREEN, "value": 80}]))
    panels.append(_stat(26, "USEFULNESS", "atlas_implementation_usefulness_score", unit="percent", y=57, x=5, w=5, h=5, color=_BLUE, thresholds=[{"color": _RED, "value": None}, {"color": _YELLOW, "value": 40}, {"color": _GREEN, "value": 70}]))
    panels.append(_stat(27, "BRAIN DELIVERY", "atlas_brain_delivery_ratio_pct", unit="percent", y=57, x=10, w=5, h=5, color=_PURPLE, thresholds=[{"color": _RED, "value": None}, {"color": _YELLOW, "value": 60}, {"color": _GREEN, "value": 85}]))
    panels.append(_stat(28, "UNTRACKED OPEN", "atlas_open_untracked_ratio_pct", unit="percent", y=57, x=15, w=4, h=5, color=_RED, thresholds=[{"color": _GREEN, "value": None}, {"color": _YELLOW, "value": 5}, {"color": _RED, "value": 20}]))
    panels.append(_stat(29, "EVIDENCE SCORE", "atlas_evidence_sufficiency_score", unit="percent", y=57, x=19, w=5, h=5, color=_YELLOW, thresholds=[{"color": _RED, "value": None}, {"color": _YELLOW, "value": 30}, {"color": _GREEN, "value": 70}]))

    return {
        "uid": "atlas-quant-pro-2026",
        "title": "ATLAS-QUANT PRO 2026 - Robot de Trading Autonomo",
        "description": "Dashboard profesional de monitoreo ATLAS-Quant-Core v1.0 | Jetson Orin Nano",
        "tags": ["atlas", "trading", "quant", "live", "production"],
        "timezone": "America/New_York",
        "schemaVersion": 38,
        "version": 1,
        "refresh": "2s",
        "time": {"from": "now-4h", "to": "now"},
        "timepicker": {"refresh_intervals": ["2s", "5s", "15s", "30s", "1m", "5m"]},
        "fiscalYearStartMonth": 0,
        "graphTooltip": 1,
        "panels": panels,
        "templating": {
            "list": [
                {
                    "name": "scope",
                    "label": "Scope",
                    "type": "custom",
                    "current": {"text": "paper", "value": "paper"},
                    "options": [{"text": scope, "value": scope, "selected": scope == "paper"} for scope in ["paper", "live"]],
                    "query": "paper,live",
                    "hide": 0,
                    "multi": False,
                },
                {
                    "name": "source",
                    "label": "Fuente",
                    "type": "custom",
                    "current": {"text": "tradier", "value": "tradier"},
                    "options": [{"text": "tradier", "value": "tradier", "selected": True}],
                    "query": "tradier",
                    "hide": 0,
                    "multi": False,
                },
                {
                    "name": "account_id",
                    "label": "Account",
                    "type": "textbox",
                    "current": {"text": ".*", "value": ".*"},
                    "hide": 0,
                },
            ]
        },
        "annotations": {
            "list": [
                {
                    "builtIn": 1,
                    "datasource": {"type": "grafana", "uid": "-- Grafana --"},
                    "enable": True,
                    "hide": True,
                    "iconColor": _RED,
                    "name": "Emergency Stops",
                    "type": "dashboard",
                }
            ]
        },
        "links": [
            {"title": "Atlas API", "url": "http://localhost:8792/docs", "type": "link", "icon": "external link", "targetBlank": True},
            {"title": "Emergency Stop", "url": "http://localhost:8792/api/trading/emergency_stop", "type": "link", "icon": "bolt", "targetBlank": False},
        ],
        "style": "dark",
        "__inputs": [
            {
                "name": "DS_ATLAS_PROMETHEUS",
                "label": "atlas-prometheus",
                "description": "",
                "type": "datasource",
                "pluginId": "prometheus",
                "pluginName": "Prometheus",
            }
        ],
        "__requires": [
            {"type": "grafana", "id": "grafana", "name": "Grafana", "version": "10.0.0"},
            {"type": "datasource", "id": "prometheus", "name": "Prometheus", "version": "1.0.0"},
            {"type": "panel", "id": "timeseries", "name": "Time series", "version": ""},
            {"type": "panel", "id": "stat", "name": "Stat", "version": ""},
            {"type": "panel", "id": "gauge", "name": "Gauge", "version": ""},
            {"type": "panel", "id": "table", "name": "Table", "version": ""},
        ],
    }


def save_pro_dashboard(path: Path | None = None) -> Path:
    dest = path or Path("grafana/dashboards/atlas_pro_2026.json")
    dest.parent.mkdir(parents=True, exist_ok=True)
    with open(dest, "w", encoding="utf-8") as handle:
        json.dump(generate_pro_dashboard(), handle, indent=2, ensure_ascii=False)
    return dest
