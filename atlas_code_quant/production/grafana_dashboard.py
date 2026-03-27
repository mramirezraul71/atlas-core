# ATLAS-Quant — Módulo 9C: Grafana Dashboard + Prometheus Metrics
"""Exposición de métricas Prometheus y dashboard Grafana pre-configurado.

Métricas expuestas (puerto 9090 /metrics):
  atlas_equity_usd           — equity actual en USD
  atlas_drawdown_pct         — drawdown actual %
  atlas_daily_pnl_usd        — PnL del día en USD
  atlas_open_positions       — posiciones abiertas
  atlas_sharpe_ratio         — Sharpe ratio rolling 20
  atlas_iv_rank              — IV Rank del símbolo principal
  atlas_ocr_confidence_pct   — confianza OCR de la cámara
  atlas_cycle_ms             — duración del último ciclo (ms)
  atlas_cpu_pct              — CPU del Jetson (%)
  atlas_trades_total         — contador de trades ejecutados
  atlas_regime               — régimen actual (0=Bear 1=Sideways 2=Bull)
  atlas_mode                 — modo activo (0=paper 1=live)

Dashboard Grafana:
  Importar grafana/dashboards/atlas.json en Grafana → :3000
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
from pathlib import Path
from typing import Optional

from atlas_code_quant.monitoring.canonical_snapshot import CanonicalSnapshotService
from atlas_code_quant.monitoring.strategy_tracker import StrategyTracker

logger = logging.getLogger("atlas.production.grafana")

_METRICS_PORT = int(os.getenv("ATLAS_METRICS_PORT", "9090"))
_DASHBOARD_PATH = Path("grafana/dashboards/atlas.json")

# ── Prometheus client (opcional) ─────────────────────────────────────────────
try:
    from prometheus_client import (
        Gauge, Counter, start_http_server, REGISTRY,
        CollectorRegistry,
    )
    _PROM_OK = True
except ImportError:
    _PROM_OK = False
    logger.warning("prometheus_client no instalado — métricas deshabilitadas")


class GrafanaDashboard:
    """Servidor de métricas Prometheus + generador de dashboard Grafana.

    Uso::

        gd = GrafanaDashboard()
        gd.start_metrics_server()      # inicia :9090
        gd.save_dashboard()            # escribe grafana/dashboards/atlas.json

        # Actualizar métricas desde LiveLoop:
        gd.update(equity=102_450, drawdown=1.8, ...)
    """

    def __init__(self) -> None:
        self._started = False
        self._lock    = threading.Lock()
        self._canonical = CanonicalSnapshotService(StrategyTracker())
        self._broker_peaks: dict[tuple[str, str], float] = {}

        if not _PROM_OK:
            return

        # Registrar métricas una sola vez (evita duplicados en tests)
        try:
            self.broker_equity = Gauge(
                "atlas_broker_equity_usd",
                "Equity broker canonica USD",
                ["source", "scope", "account_id"],
            )
            self.broker_cash = Gauge(
                "atlas_broker_cash_usd",
                "Cash broker canonico USD",
                ["source", "scope", "account_id"],
            )
            self.broker_market_value = Gauge(
                "atlas_broker_market_value_usd",
                "Market value broker canonico USD",
                ["source", "scope", "account_id"],
            )
            self.broker_open_pnl = Gauge(
                "atlas_broker_open_pnl_usd",
                "PnL abierto broker canonico USD",
                ["source", "scope", "account_id"],
            )
            self.broker_open_positions = Gauge(
                "atlas_broker_open_positions",
                "Posiciones abiertas broker canonicas",
                ["source", "scope", "account_id"],
            )
            self.broker_drawdown = Gauge(
                "atlas_broker_drawdown_pct",
                "Drawdown broker canonico %",
                ["source", "scope", "account_id"],
            )
            self.sync_status = Gauge(
                "atlas_sync_status",
                "Estado de sincronizacion canonica 0 failed 1 degraded 2 stale 3 healthy",
                ["source", "scope", "account_id"],
            )
            self.reconcile_gap_usd = Gauge(
                "atlas_reconcile_gap_usd",
                "Brecha USD entre broker canonico y capas derivadas",
                ["scope", "account_id", "comparison"],
            )
            self.reconcile_gap_positions = Gauge(
                "atlas_reconcile_gap_positions",
                "Brecha de posiciones entre broker canonico y capas derivadas",
                ["scope", "account_id", "comparison"],
            )
            self.simulator_equity = Gauge(
                "atlas_simulator_equity_usd",
                "Equity de simuladores y vistas derivadas",
                ["source", "simulator"],
            )
            self.simulator_open_positions = Gauge(
                "atlas_simulator_open_positions",
                "Posiciones o estrategias abiertas en simuladores",
                ["source", "simulator"],
            )
            self.equity        = Gauge("atlas_equity_usd",          "Equity USD")
            self.drawdown      = Gauge("atlas_drawdown_pct",         "Drawdown %")
            self.daily_pnl     = Gauge("atlas_daily_pnl_usd",        "PnL diario USD")
            self.open_pos      = Gauge("atlas_open_positions",        "Posiciones abiertas")
            self.sharpe        = Gauge("atlas_sharpe_ratio",          "Sharpe rolling 20")
            self.iv_rank       = Gauge("atlas_iv_rank",               "IV Rank 0-100")
            self.ocr_conf      = Gauge("atlas_ocr_confidence_pct",    "OCR confianza %")
            self.cycle_ms      = Gauge("atlas_cycle_ms",              "Duración ciclo ms")
            self.cpu_pct       = Gauge("atlas_cpu_pct",               "CPU Jetson %")
            self.regime        = Gauge("atlas_regime",                "Régimen 0=Bear 1=Sideways 2=Bull")
            self.mode_live     = Gauge("atlas_mode",                  "Modo 0=paper 1=live")
            self.trades_total  = Counter("atlas_trades_total",        "Total trades ejecutados")
        except Exception as exc:
            logger.warning("Error registrando métricas Prometheus: %s", exc)

    # ── Servidor de métricas ──────────────────────────────────────────────────

    def start_metrics_server(self, port: int = _METRICS_PORT) -> bool:
        """Inicia servidor HTTP Prometheus en background."""
        if not _PROM_OK:
            logger.warning("prometheus_client no disponible")
            return False
        if self._started:
            return True
        try:
            start_http_server(port)
            self._started = True
            logger.info("✓ Prometheus métricas en http://0.0.0.0:%d/metrics", port)
            return True
        except Exception as exc:
            logger.error("Error iniciando servidor Prometheus: %s", exc)
            return False

    # ── Actualización de métricas ─────────────────────────────────────────────

    def update(
        self,
        equity:       float | None = None,
        drawdown:     float | None = None,
        daily_pnl:    float | None = None,
        open_pos:     int   | None = None,
        sharpe:       float | None = None,
        iv_rank:      float | None = None,
        ocr_conf:     float | None = None,
        cycle_ms:     float | None = None,
        cpu_pct:      float | None = None,
        regime_label: str   | None = None,    # "bull" | "bear" | "sideways"
        mode:         str   | None = None,    # "paper" | "live"
        new_trade:    bool        = False,
    ) -> None:
        if not _PROM_OK or not self._started:
            return
        with self._lock:
            try:
                if equity       is not None: self.equity.set(equity)
                if drawdown     is not None: self.drawdown.set(drawdown)
                if daily_pnl    is not None: self.daily_pnl.set(daily_pnl)
                if open_pos     is not None: self.open_pos.set(open_pos)
                if sharpe       is not None: self.sharpe.set(sharpe)
                if iv_rank      is not None: self.iv_rank.set(iv_rank)
                if ocr_conf     is not None: self.ocr_conf.set(ocr_conf)
                if cycle_ms     is not None: self.cycle_ms.set(cycle_ms)
                if cpu_pct      is not None: self.cpu_pct.set(cpu_pct)
                if new_trade:                self.trades_total.inc()
                if regime_label is not None:
                    regime_map = {"bull": 2, "sideways": 1, "bear": 0}
                    self.regime.set(regime_map.get(regime_label.lower(), 1))
                if mode is not None:
                    self.mode_live.set(1 if mode == "live" else 0)
            except Exception as exc:
                logger.debug("Error actualizando métrica: %s", exc)

    def sync_from_canonical(
        self,
        *,
        account_scope: str = "paper",
        account_id: str | None = None,
        internal_portfolio=None,
    ) -> dict | None:
        """Sincroniza gauges broker-first desde Tradier."""
        if not _PROM_OK or not self._started:
            return None
        snapshot = self._canonical.build_snapshot(
            account_scope=account_scope,
            account_id=account_id,
            internal_portfolio=internal_portfolio,
        )
        self._apply_canonical_snapshot(snapshot)
        return snapshot

    def _apply_canonical_snapshot(self, snapshot: dict) -> None:
        scope = str(snapshot.get("account_scope") or "paper")
        account_id = str(snapshot.get("account_id") or "unknown")
        source = str(snapshot.get("source") or "tradier")
        balances = snapshot.get("balances") or {}
        totals = snapshot.get("totals") or {}
        reconciliation = snapshot.get("reconciliation") or {}
        simulators = snapshot.get("simulators") or {}
        labels = (source, scope, account_id)

        with self._lock:
            equity = float(balances.get("total_equity") or 0.0)
            cash = float(balances.get("cash") or 0.0)
            market_value = float(balances.get("market_value") or 0.0)
            open_pnl = float(totals.get("open_pnl") or 0.0)
            open_positions = float(totals.get("positions") or 0.0)

            self.broker_equity.labels(*labels).set(equity)
            self.broker_cash.labels(*labels).set(cash)
            self.broker_market_value.labels(*labels).set(market_value)
            self.broker_open_pnl.labels(*labels).set(open_pnl)
            self.broker_open_positions.labels(*labels).set(open_positions)

            peak_key = (scope, account_id)
            peak = max(self._broker_peaks.get(peak_key, equity), equity)
            self._broker_peaks[peak_key] = peak
            drawdown_pct = ((equity - peak) / peak * 100.0) if peak > 0 else 0.0
            self.broker_drawdown.labels(*labels).set(drawdown_pct)

            self.sync_status.labels(*labels).set(self._sync_state_code(reconciliation.get("state")))
            for item in reconciliation.get("items") or []:
                comparison = str(item.get("comparison") or "unknown")
                metric = str(item.get("metric") or "")
                if metric == "equity_usd":
                    self.reconcile_gap_usd.labels(scope, account_id, comparison).set(float(item.get("gap") or 0.0))
                elif metric == "open_positions":
                    self.reconcile_gap_positions.labels(scope, account_id, comparison).set(float(item.get("gap") or 0.0))

            for simulator_name, simulator in simulators.items():
                if not isinstance(simulator, dict):
                    continue
                source_name = str(simulator.get("source") or simulator_name)
                equity_value = simulator.get("equity")
                positions_value = simulator.get("open_positions")
                self.simulator_equity.labels(source_name, simulator_name).set(float(equity_value or 0.0))
                self.simulator_open_positions.labels(source_name, simulator_name).set(float(positions_value or 0.0))

    @staticmethod
    def _sync_state_code(state: str | None) -> int:
        mapping = {"failed": 0, "degraded": 1, "stale": 2, "healthy": 3}
        return mapping.get(str(state or "").strip().lower(), 0)

    # ── Dashboard Grafana JSON ────────────────────────────────────────────────

    def generate_dashboard_json(self) -> dict:
        """Genera dashboard JSON listo para importar en Grafana."""

        def panel(pid: int, title: str, expr: str,
                  unit: str = "short", y: int = 0, w: int = 8, h: int = 8,
                  thresholds: list | None = None, ptype: str = "timeseries") -> dict:
            base = {
                "id":      pid,
                "title":   title,
                "type":    ptype,
                "gridPos": {"x": (pid - 1) % 3 * 8, "y": y, "w": w, "h": h},
                "targets": [{
                    "datasource": {"type": "prometheus", "uid": "atlas-prom"},
                    "expr":       expr,
                    "legendFormat": title,
                    "refId":      "A",
                }],
                "fieldConfig": {
                    "defaults": {
                        "unit": unit,
                        "color": {"mode": "palette-classic"},
                        "custom": {"lineWidth": 2, "fillOpacity": 10},
                    }
                },
                "options": {"tooltip": {"mode": "multi"}},
            }
            if thresholds:
                base["fieldConfig"]["defaults"]["thresholds"] = {
                    "mode": "absolute",
                    "steps": thresholds,
                }
            return base

        def stat_panel(pid: int, title: str, expr: str,
                       unit: str = "short", y: int = 0, color: str = "green") -> dict:
            return {
                "id":      pid,
                "title":   title,
                "type":    "stat",
                "gridPos": {"x": (pid - 1) % 6 * 4, "y": y, "w": 4, "h": 4},
                "targets": [{
                    "datasource": {"type": "prometheus", "uid": "atlas-prom"},
                    "expr": expr, "legendFormat": title, "refId": "A",
                }],
                "fieldConfig": {
                    "defaults": {
                        "unit": unit,
                        "color": {"mode": "fixed", "fixedColor": color},
                        "thresholds": {"mode": "absolute", "steps": [
                            {"color": "red", "value": None},
                            {"color": color, "value": 0},
                        ]},
                    }
                },
                "options": {"reduceOptions": {"calcs": ["lastNotNull"]},
                            "orientation": "auto", "textMode": "auto",
                            "colorMode": "background"},
            }

        panels = [
            # Fila 1: Stats rápidos (y=0)
            stat_panel(1,  "Equity Broker",   "atlas_broker_equity_usd{source=\"$source\",scope=\"$scope\",account_id=~\"$account_id\"}", "currencyUSD", y=0, color="green"),
            stat_panel(2,  "Drawdown Broker", "atlas_broker_drawdown_pct{source=\"$source\",scope=\"$scope\",account_id=~\"$account_id\"}", "percent", y=0, color="orange"),
            stat_panel(3,  "PnL Abierto",     "atlas_broker_open_pnl_usd{source=\"$source\",scope=\"$scope\",account_id=~\"$account_id\"}", "currencyUSD", y=0, color="blue"),
            stat_panel(4,  "Posiciones",      "atlas_broker_open_positions{source=\"$source\",scope=\"$scope\",account_id=~\"$account_id\"}", "short", y=0, color="purple"),
            stat_panel(5,  "Sync Status",     "atlas_sync_status{source=\"$source\",scope=\"$scope\",account_id=~\"$account_id\"}", "short", y=0, color="cyan"),
            stat_panel(6,  "Gap Atlas USD",   "atlas_reconcile_gap_usd{scope=\"$scope\",account_id=~\"$account_id\",comparison=\"atlas_internal\"}", "currencyUSD", y=0, color="red"),

            # Fila 2: Curvas (y=4)
            panel(7,  "Equity Broker (USD)",   "atlas_broker_equity_usd{source=\"$source\",scope=\"$scope\",account_id=~\"$account_id\"}",
                  unit="currencyUSD", y=4, w=12, h=10),
            panel(8,  "Drawdown Broker (%)",  "atlas_broker_drawdown_pct{source=\"$source\",scope=\"$scope\",account_id=~\"$account_id\"}",
                  unit="percent", y=4, w=12, h=10,
                  thresholds=[
                      {"color": "green", "value": None},
                      {"color": "yellow", "value": 3},
                      {"color": "red",    "value": 8},
                  ]),

            # Fila 3: Indicadores (y=14)
            panel(9,  "IV Rank",              "atlas_iv_rank",
                  unit="percent", y=14, w=8, h=8),
            panel(10, "OCR Confianza (%)",    "atlas_ocr_confidence_pct",
                  unit="percent", y=14, w=8, h=8,
                  thresholds=[
                      {"color": "red",    "value": None},
                      {"color": "yellow", "value": 80},
                      {"color": "green",  "value": 92},
                  ]),
            panel(11, "Régimen ML",           "atlas_regime",
                  unit="short", y=14, w=8, h=8),

            # Fila 4: Sistema (y=22)
            panel(12, "CPU Jetson (%)",       "atlas_cpu_pct",
                  unit="percent", y=22, w=8, h=8,
                  thresholds=[
                      {"color": "green",  "value": None},
                      {"color": "yellow", "value": 70},
                      {"color": "red",    "value": 85},
                  ]),
            panel(13, "Duración Ciclo (ms)",  "atlas_cycle_ms",
                  unit="ms", y=22, w=8, h=8),
            panel(14, "Trades Totales",
                  "rate(atlas_trades_total[5m])*60",
                  unit="short", y=22, w=8, h=8),
        ]

        return {
            "uid":           "atlas-quant-main",
            "title":         "ATLAS-Quant — Robot de Trading",
            "description":   "Dashboard de monitoreo del robot de trading autónomo ATLAS",
            "tags":          ["atlas", "trading", "quant"],
            "timezone":      "America/New_York",
            "schemaVersion": 38,
            "version":       1,
            "refresh":       "2s",
            "time":          {"from": "now-1h", "to": "now"},
            "timepicker":    {},
            "panels":        panels,
            "templating":    {
                "list": [
                    {
                        "name": "scope",
                        "label": "Scope",
                        "type": "custom",
                        "current": {"text": "paper", "value": "paper"},
                        "options": [
                            {"text": "paper", "value": "paper", "selected": True},
                            {"text": "live", "value": "live", "selected": False},
                        ],
                        "query": "paper,live",
                        "hide": 0,
                        "multi": False,
                    },
                    {
                        "name": "source",
                        "label": "Source",
                        "type": "custom",
                        "current": {"text": "tradier", "value": "tradier"},
                        "options": [
                            {"text": "tradier", "value": "tradier", "selected": True},
                        ],
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
            "annotations":   {"list": []},
            "links":         [],
        }

    def save_dashboard(self, path: Path | None = None) -> Path:
        """Escribe dashboard JSON en disco."""
        dest = path or _DASHBOARD_PATH
        dest.parent.mkdir(parents=True, exist_ok=True)
        with open(dest, "w", encoding="utf-8") as f:
            json.dump(self.generate_dashboard_json(), f, indent=2)
        logger.info("Dashboard Grafana guardado: %s", dest)
        return dest

    def save_provisioning(self) -> None:
        """Escribe archivos de provisioning de Grafana (datasource + dashboard)."""
        # Datasource: Prometheus
        ds_path = Path("grafana/provisioning/datasources/atlas.yaml")
        ds_path.parent.mkdir(parents=True, exist_ok=True)
        ds_path.write_text(
            "apiVersion: 1\n"
            "datasources:\n"
            "  - name: atlas-prometheus\n"
            "    uid: atlas-prom\n"
            "    type: prometheus\n"
            "    url: http://prometheus:9090\n"
            "    access: proxy\n"
            "    isDefault: true\n"
            "    jsonData:\n"
            "      timeInterval: '2s'\n",
            encoding="utf-8"
        )

        # Dashboard provisioning
        db_path = Path("grafana/provisioning/dashboards/atlas.yaml")
        db_path.parent.mkdir(parents=True, exist_ok=True)
        db_path.write_text(
            "apiVersion: 1\n"
            "providers:\n"
            "  - name: atlas\n"
            "    type: file\n"
            "    options:\n"
            "      path: /etc/grafana/dashboards\n",
            encoding="utf-8"
        )

        logger.info("Archivos de provisioning Grafana escritos")

    # ── Dashboard PRO 2026 ────────────────────────────────────────────────────

    def save_pro_dashboard(self, path: Path | None = None) -> Path:
        """Genera y guarda atlas_pro_2026.json — dashboard dark trading profesional.

        23 paneles: equity curve, drawdown waterfall, regime gauge, IV rank,
        OCR accuracy, Kelly donut, CVD, self-healing CPU, positions table,
        robot status + variables $symbol/$mode + links Emergency Stop.
        """
        from atlas_code_quant.production.grafana_pro import save_pro_dashboard
        dest = path or Path("grafana/dashboards/atlas_pro_2026.json")
        result = save_pro_dashboard(dest)
        logger.info("Dashboard PRO 2026 guardado: %s", result)
        return result

    # ── Callback para LiveLoop ────────────────────────────────────────────────

    def make_cycle_callback(self):
        """Retorna función callback para llamar al final de cada ciclo."""
        def on_cycle(metrics_dict: dict) -> None:
            self.update(
                equity       = metrics_dict.get("equity"),
                drawdown     = metrics_dict.get("drawdown_pct"),
                daily_pnl    = metrics_dict.get("daily_pnl"),
                open_pos     = metrics_dict.get("open_positions"),
                sharpe       = metrics_dict.get("sharpe"),
                iv_rank      = metrics_dict.get("iv_rank"),
                ocr_conf     = metrics_dict.get("ocr_confidence"),
                cycle_ms     = metrics_dict.get("cycle_ms"),
                cpu_pct      = metrics_dict.get("cpu_pct"),
                regime_label = metrics_dict.get("regime"),
                mode         = metrics_dict.get("mode"),
                new_trade    = metrics_dict.get("new_trade", False),
            )
        return on_cycle
