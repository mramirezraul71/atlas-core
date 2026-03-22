"""Servidor de métricas Prometheus para ATLAS-Quant.
Arranca solo el servidor HTTP de métricas en :9090 con valores iniciales.
"""
import time
import os
import sys

# Añadir raíz al path
sys.path.insert(0, r"C:\ATLAS_PUSH")

try:
    from prometheus_client import start_http_server, Gauge, Counter, Info
except ImportError:
    print("ERROR: prometheus_client no instalado. Ejecuta: pip install prometheus-client")
    sys.exit(1)

PORT = int(os.getenv("ATLAS_METRICS_PORT", "9090"))

# ── Métricas ATLAS-Quant ──────────────────────────────────────────────────────
equity          = Gauge("atlas_equity_usd",        "Equity total en USD")
drawdown        = Gauge("atlas_drawdown_pct",       "Drawdown máximo %")
pnl_today       = Gauge("atlas_pnl_today_usd",      "PnL del día en USD")
sharpe          = Gauge("atlas_sharpe_ratio",        "Sharpe anualizado")
open_positions  = Gauge("atlas_open_positions",      "Posiciones abiertas")
mode_live       = Gauge("atlas_mode_live",           "1=live 0=paper")
kelly_fraction  = Gauge("atlas_kelly_fraction",      "Fracción Kelly actual")
calmar_ratio    = Gauge("atlas_calmar_ratio",        "Ratio Calmar retorno/DD")
win_rate        = Gauge("atlas_win_rate_pct",        "Win rate %")
total_trades    = Counter("atlas_trades_total",      "Total trades ejecutados")
ocr_accuracy    = Gauge("atlas_ocr_accuracy_pct",   "Precisión OCR %")
cpu_pct         = Gauge("atlas_cpu_usage_pct",       "CPU uso %")
regime          = Gauge("atlas_regime",             "Régimen ML: 0=bull 1=bear 2=sideways")
iv_rank         = Gauge("atlas_iv_rank",            "IV rank 0-100")
info            = Info("atlas_quant", "Información del sistema ATLAS-Quant")

# Valores iniciales realistas (paper mode)
equity.set(10000.0)
drawdown.set(0.0)
pnl_today.set(0.0)
sharpe.set(0.0)
open_positions.set(0)
mode_live.set(0)          # paper
kelly_fraction.set(0.25)
calmar_ratio.set(0.0)
win_rate.set(0.0)
ocr_accuracy.set(0.0)
cpu_pct.set(0.0)
regime.set(2)             # sideways al arrancar
iv_rank.set(50.0)
info.info({"version": "1.0.0", "mode": "paper", "symbols": "AAPL,TSLA,SPY"})

print(f"[ATLAS-Metrics] Servidor iniciado en http://localhost:{PORT}/metrics")
print("[ATLAS-Metrics] Esperando scraping de Prometheus...")
start_http_server(PORT)

# Loop: actualizar CPU real
import psutil
while True:
    try:
        cpu_pct.set(psutil.cpu_percent(interval=None))
    except Exception:
        pass
    time.sleep(5)
