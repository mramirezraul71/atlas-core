#!/bin/bash
# ATLAS-Quant entrypoint para Jetson Docker
# Modos: api | core | core-live | calibrate | test | train | backtest | shell

set -e

echo "=== ATLAS-Quant-Core Startup ==="
echo "Modo : ${1:-api}"
echo "Env  : ${ATLAS_ENV:-development}"

# ── Redis ─────────────────────────────────────────────────────────────────────
redis-server /etc/redis/redis.conf --daemonize yes
sleep 1
echo "✓ Redis iniciado"

# ── Display virtual (para pyautogui en Jetson headless) ───────────────────────
if [ -z "${DISPLAY}" ]; then
    export DISPLAY=:99
    Xvfb :99 -screen 0 1920x1080x24 &
    sleep 1
    echo "✓ Display virtual :99 iniciado"
fi

# ── PulseAudio (TTS pyttsx3/espeak-ng) ───────────────────────────────────────
if command -v pulseaudio &>/dev/null && ! pulseaudio --check 2>/dev/null; then
    pulseaudio --start --log-target=stderr 2>/dev/null || true
    echo "✓ PulseAudio iniciado"
fi

# ── Credenciales ──────────────────────────────────────────────────────────────
if [ -f "/credentials/credenciales.txt" ]; then
    echo "✓ Cargando credenciales desde volumen"
    export TRADIER_CREDENTIALS_FILE="/credentials/credenciales.txt"
fi

# ── Calibración: auto-cargar si existe, forzar si no ─────────────────────────
_SCREEN_MAP="/calibration/atlas_screen_map.json"

check_calibration() {
    if [ -f "${_SCREEN_MAP}" ]; then
        echo "✓ Mapa de calibración encontrado: ${_SCREEN_MAP}"
        export ATLAS_SCREEN_MAP="${_SCREEN_MAP}"
        return 0
    else
        echo "⚠ Mapa de calibración NO encontrado."
        return 1
    fi
}

cd /workspace

case "${1:-api}" in

    # ── API REST ──────────────────────────────────────────────────────────────
    api)
        check_calibration || echo "  → Iniciando sin calibración (solo API)"
        echo "▶ Iniciando API Code-Quant (puerto 8792)..."
        exec python3 -m uvicorn atlas_code_quant.api.main:app \
            --host 0.0.0.0 \
            --port 8792 \
            --log-level info \
            --workers 1
        ;;

    # ── Trading loop: paper (defecto seguro) ──────────────────────────────────
    core)
        if ! check_calibration; then
            echo "  → Ejecutando calibración automática primero..."
            python3 -c "
from atlas_code_quant.calibration.physical_calibration import PhysicalCalibrator
from atlas_code_quant.calibration.voice_feedback import VoiceFeedback
v = VoiceFeedback(); v.start()
cal = PhysicalCalibrator(voice=v)
m = cal.calibrate()
cal.save_map(m)
v.stop()
print('Calibración completada.')
"
        fi
        echo "▶ Iniciando ATLAS-Quant-Core en modo PAPER..."
        exec python3 -m atlas_code_quant.atlas_quant_core --mode paper
        ;;

    # ── Trading loop: LIVE real ───────────────────────────────────────────────
    core-live)
        if ! check_calibration; then
            echo "ERROR: Modo LIVE requiere calibración. Ejecuta primero: docker run ... calibrate"
            exit 1
        fi
        if [ -z "${TRADIER_LIVE_TOKEN}" ]; then
            echo "ERROR: TRADIER_LIVE_TOKEN no configurado. No se puede iniciar modo LIVE."
            exit 1
        fi
        echo "▶ Iniciando ATLAS-Quant-Core en modo LIVE..."
        export ATLAS_MODE=live
        exec python3 -m atlas_code_quant.atlas_quant_core --mode live
        ;;

    # ── Calibración física ────────────────────────────────────────────────────
    calibrate)
        echo "▶ Iniciando calibración física de pantalla..."
        mkdir -p /calibration
        exec python3 -c "
from atlas_code_quant.calibration.physical_calibration import PhysicalCalibrator
from atlas_code_quant.calibration.voice_feedback import VoiceFeedback
import os, sys

force = '--force' in sys.argv

v = VoiceFeedback()
v.start()
cal = PhysicalCalibrator(
    rtmp_url=os.getenv('ATLAS_RTMP_URL', 'rtmp://192.168.1.10/live/atlas'),
    voice=v,
)

if not force and PhysicalCalibrator.map_exists():
    m = PhysicalCalibrator.load_map()
    print(f'Mapa cargado ({len(m.monitors)} monitores). Usa --force para recalibrar.')
else:
    m = cal.calibrate()

saved = cal.save_map(m)
print(f'Mapa guardado: {saved}')
v.stop()
" "$@"
        ;;

    # ── Test runner (N ciclos paper + reporte) ────────────────────────────────
    test)
        check_calibration || echo "  → Test sin calibración (modo sintético)"
        SYMBOLS="${ATLAS_SYMBOLS:-SPY,QQQ,AAPL,TSLA,NVDA}"
        CYCLES="${ATLAS_TEST_CYCLES:-50}"
        echo "▶ Ejecutando test runner ($CYCLES ciclos | símbolos: $SYMBOLS)..."
        mkdir -p /workspace/reports
        exec python3 -m atlas_code_quant.calibration.test_runner \
            --mode paper \
            --symbols "${SYMBOLS}" \
            --cycles "${CYCLES}" \
            --output /workspace/reports
        ;;

    # ── Entrenamiento de modelos ──────────────────────────────────────────────
    train)
        echo "▶ Entrenando clasificador de régimen..."
        exec python3 -c "
from atlas_code_quant.models.regime_classifier import RegimeClassifier
clf = RegimeClassifier(use_gpu=True)
clf._train_from_history(['SPY','QQQ','IWM','AAPL','NVDA','MSFT','AMZN'])
print('Entrenamiento completado')
"
        ;;

    # ── Activación LIVE completa (M9) ─────────────────────────────────────────
    live-activation)
        echo "▶ Protocolo de activación LIVE — verificando todos los sistemas..."
        SYMBOLS="${ATLAS_SYMBOLS:-SPY,QQQ,AAPL,TSLA,NVDA}"
        CYCLES="${ATLAS_TEST_CYCLES:-50}"
        RTMP="${ATLAS_RTMP_URL:-rtmp://192.168.1.10/live/atlas}"

        if [ -z "${TRADIER_LIVE_TOKEN}" ]; then
            echo "ERROR: TRADIER_LIVE_TOKEN no configurado. No se puede activar modo LIVE."
            exit 1
        fi
        if [ -z "${TELEGRAM_BOT_TOKEN}" ] || [ -z "${TELEGRAM_CHAT_ID}" ]; then
            echo "AVISO: TELEGRAM_BOT_TOKEN / TELEGRAM_CHAT_ID no configurados."
            echo "       Las alertas Telegram estarán deshabilitadas."
        fi

        mkdir -p /calibration /workspace/reports /workspace/data/operation
        exec python3 -m atlas_code_quant.production.live_activation \
            --symbols "${SYMBOLS}" \
            --cycles "${CYCLES}" \
            --rtmp "${RTMP}" \
            --mode live
        ;;

    # ── Backtest histórico ────────────────────────────────────────────────────
    backtest)
        echo "▶ Ejecutando backtest M7..."
        exec python3 -m atlas_code_quant.scripts.backtest_m7 "$@"
        ;;

    # ── Auditoría final del sistema ───────────────────────────────────────────
    audit)
        echo "▶ Ejecutando auditoría final del sistema ATLAS-Quant..."
        mkdir -p /workspace/reports /workspace/grafana/dashboards
        exec python3 -m atlas_code_quant.atlas_quant_core --final-audit
        ;;

    # ── Grafana provisioning (generar JSON y salir) ───────────────────────────
    grafana-setup)
        echo "▶ Generando dashboard Grafana..."
        exec python3 -c "
from atlas_code_quant.production.grafana_dashboard import GrafanaDashboard
gd = GrafanaDashboard()
gd.save_dashboard()
gd.save_provisioning()
print('Dashboard guardado en grafana/dashboards/atlas.json')
print('Provisioning en grafana/provisioning/')
"
        ;;

    # ── Shell de diagnóstico ──────────────────────────────────────────────────
    shell)
        exec bash
        ;;

    *)
        exec "$@"
        ;;
esac
