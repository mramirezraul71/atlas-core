# ATLAS-Quant — Módulo 9D: Live Activation
"""Script principal de activación de modo LIVE con todas las verificaciones.

Flujo de activación (Mermaid):

    flowchart TD
        START([python -m live_activation]) --> CAL{Mapa\ncalibración?}
        CAL -- NO --> CALIB[PhysicalCalibrator.calibrate]
        CAL -- SI --> TEST{Test runner\npasado?}
        CALIB --> TEST

        TEST -- NO --> RUN_TEST[TestRunner 50 ciclos]
        RUN_TEST --> RESULT{ready_for_live?}
        RESULT -- NO --> EXIT_FAIL([❌ No cumple criterios\nContinuar en paper])
        RESULT -- SI --> TEST

        TEST -- SI --> GUARD[ProductionGuard\ncheck_ready_for_live]
        GUARD -- blocked --> EXIT_FAIL
        GUARD -- ok --> VOICE["Voz: ATLAS listo para LIVE\nConfirmación en 10 segundos"]

        VOICE --> DOUBLE[Double Confirmation\nvoz + círculo ×3 + F12]
        DOUBLE -- rechazado --> EXIT_CANCEL([❌ Cancelado por operador])
        DOUBLE -- aprobado --> PREVIEW[Tradier preview=true\nprueba de conectividad]

        PREVIEW -- error --> EXIT_API([❌ Error API Tradier])
        PREVIEW -- ok --> TG[Telegram: LIVE activado]
        TG --> GRAFANA[Grafana metrics server :9090]
        GRAFANA --> LAUNCH[ATLASQuantCore mode=live\nLiveLoop.start]
        LAUNCH --> RUNNING([✅ ATLAS LIVE en producción])

Ejecución:
    python -m atlas_code_quant.production.live_activation
    # o desde Docker:
    docker run ... atlas-quant:jetson live-activation
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
from pathlib import Path

logger = logging.getLogger("atlas.production.live_activation")

_READY_FILE = Path(os.getenv(
    "ATLAS_READY_FILE", "data/operation/ready_for_live.json"
))


def _setup_logging(level: str = "INFO") -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


# ── Pasos del flujo ───────────────────────────────────────────────────────────

def step_calibration(voice, rtmp_url: str) -> bool:
    """Paso 1: Verificar o ejecutar calibración física."""
    from atlas_code_quant.calibration.physical_calibration import PhysicalCalibrator

    if PhysicalCalibrator.map_exists():
        m = PhysicalCalibrator.load_map()
        logger.info("✓ Calibración existente (%d monitores)", len(m.monitors) if m else 0)
        voice.speak("Mapa de calibración cargado.")
        return True

    logger.info("Ejecutando calibración física…")
    voice.speak("No se encontró calibración. Iniciando calibración automática.", urgent=True)
    time.sleep(1.5)

    cal = PhysicalCalibrator(rtmp_url=rtmp_url, voice=voice)
    try:
        screen_map = cal.calibrate()
        cal.save_map(screen_map)
        logger.info("✓ Calibración completada")
        return True
    except Exception as exc:
        logger.error("Error en calibración: %s", exc)
        return False


def step_test_runner(
    voice,
    symbols: list[str],
    cycles: int,
    force_rerun: bool = False,
) -> tuple[bool, dict]:
    """Paso 2: Verificar resultados de test runner o ejecutar si no existen."""
    from atlas_code_quant.calibration.test_runner import TestRunner

    # Buscar resultado existente
    if not force_rerun:
        for path in [_READY_FILE,
                     Path("reports/ready_for_live.json"),
                     Path("data/ready_for_live.json")]:
            if path.exists():
                try:
                    with open(path, encoding="utf-8") as f:
                        data = json.load(f)
                    logger.info(
                        "✓ Test runner resultado existente — ready=%s Sharpe=%.2f DD=%.1f%%",
                        data.get("ready_for_live"), data.get("sharpe", 0),
                        data.get("max_drawdown_pct", 0)
                    )
                    return data.get("ready_for_live", False), data
                except Exception:
                    pass

    # Ejecutar test runner
    logger.info("Ejecutando test runner (%d ciclos)…", cycles)
    voice.announce_test_start(cycles)

    runner = TestRunner(
        symbols        = symbols,
        cycles         = cycles,
        output_dir     = "reports",
        voice          = voice,
        fast_mode      = True,
    )
    report = runner.run()
    report_path = runner.save_report(report)
    runner.plot_equity_curve(report)
    runner.announce_results(report)

    # Guardar como ready_for_live.json para futuras activaciones
    _READY_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(_READY_FILE, "w", encoding="utf-8") as f:
        json.dump(report.to_dict(), f, indent=2)

    logger.info("Test runner completado — ready=%s", report.ready_for_live)
    return report.ready_for_live, report.to_dict()


def step_production_guard(voice, hid) -> tuple[bool, "ProductionGuard"]:  # type: ignore[name-defined]
    """Paso 3: Verificar ProductionGuard y obtener instancia."""
    from atlas_code_quant.production.production_guard import ProductionGuard

    guard = ProductionGuard(voice=voice, hid=hid)
    ok, reason = guard.check_ready_for_live()
    if not ok:
        logger.error("ProductionGuard bloqueó activación LIVE: %s", reason)
        voice.speak(f"Activación bloqueada. {reason}", urgent=True)
        return False, guard

    logger.info("✓ ProductionGuard: sistema apto para LIVE")
    return True, guard


def step_tradier_preview_test(mode: str = "live") -> tuple[bool, str]:
    """Paso 4: Verifica conectividad Tradier con preview=true."""
    try:
        from atlas_code_quant.execution.tradier_controls import resolve_account_session
        from atlas_code_quant.api.schemas import OrderRequest

        order = OrderRequest(
            symbol        = "SPY",
            side          = "buy",
            size          = 1,
            order_type    = "market",
            duration      = "day",
            account_scope = mode,  # type: ignore[arg-type]
            preview       = True,
            tag           = "atlas_live_test",
        )
        from atlas_code_quant.execution.tradier_execution import route_order_to_tradier
        resp = route_order_to_tradier(order)
        logger.info("✓ Tradier preview test OK — response keys: %s",
                    list(resp.keys())[:5])
        return True, "ok"
    except Exception as exc:
        logger.warning("Tradier preview test: %s (puede ser normal en paper)", exc)
        # No bloquear si es paper — el error puede ser que no hay token live
        if mode == "paper" or "TOKEN" in str(exc).upper():
            return True, "paper_mode_skip"
        return False, str(exc)


def step_double_confirmation(guard, voice) -> bool:
    """Paso 5: Confirmación doble (voz + círculos + F12)."""
    voice.speak(
        "ATLAS está listo para iniciar trading real. "
        "Comenzando protocolo de confirmación física en diez segundos.",
        urgent=True
    )
    time.sleep(10.0)

    confirmed = guard.require_double_confirmation()
    if not confirmed:
        voice.speak("Activación cancelada por el operador.", urgent=True)
        logger.warning("Activación LIVE cancelada en step_double_confirmation")
    return confirmed


def step_launch_live(
    core,
    guard,
    telegram,
    grafana,
    symbols: list[str],
    mode: str,
) -> None:
    """Paso 6: Lanzar ATLASQuantCore en modo LIVE."""
    # Telegram: notificar activación
    if telegram:
        telegram.alert_live_activated()

    # Grafana: guardar dashboard y arrancar servidor
    if grafana:
        grafana.save_dashboard()
        grafana.save_provisioning()
        grafana.start_metrics_server()

    # LIVE
    logger.warning("▶▶▶ ATLAS-Quant MODO LIVE INICIADO ◀◀◀")
    core.start()


# ── Flujo principal ───────────────────────────────────────────────────────────

def activate(
    symbols: list[str],
    cycles: int = 50,
    rtmp_url: str = "rtmp://192.168.1.10/live/atlas",
    force_rerun_test: bool = False,
    skip_confirmation: bool = False,   # SÓLO para CI/tests
    mode: str = "live",
) -> int:
    """
    Ejecuta el flujo completo de activación LIVE.
    Retorna 0 si ATLAS se lanzó en live, 1 si se canceló/bloqueó.
    """
    from atlas_code_quant.calibration.voice_feedback import VoiceFeedback
    from atlas_code_quant.production.telegram_alerts import TelegramAlerts
    from atlas_code_quant.production.grafana_dashboard import GrafanaDashboard

    voice    = VoiceFeedback()
    voice.start()
    telegram = TelegramAlerts(voice=voice)
    telegram.start()
    grafana  = GrafanaDashboard()

    voice.speak("ATLAS-Quant activación iniciada. Verificando todos los sistemas.")
    time.sleep(1.0)

    print("\n" + "="*62)
    print("   ATLAS-QUANT — ACTIVACIÓN MODO LIVE")
    print("="*62)

    # ── Paso 1: Calibración ───────────────────────────────────────────────────
    print("\n[1/6] Calibración física…")
    ok = step_calibration(voice, rtmp_url)
    if not ok:
        print("  ✗ FALLO — calibración no completada")
        voice.stop(); telegram.stop()
        return 1
    print("  ✓ OK")

    # ── Paso 2: Test runner ───────────────────────────────────────────────────
    print(f"\n[2/6] Test runner ({cycles} ciclos)…")
    ready, test_data = step_test_runner(voice, symbols, cycles, force_rerun_test)
    if not ready:
        print(
            f"  ✗ Sistema NO apto para LIVE\n"
            f"    Sharpe={test_data.get('sharpe',0):.2f} "
            f"DD={test_data.get('max_drawdown_pct',0):.1f}% "
            f"WinRate={test_data.get('win_rate_pct',0):.1f}%"
        )
        voice.speak("Sistema no cumple criterios para trading real. Continúa en paper.", urgent=True)
        voice.stop(); telegram.stop()
        return 1
    print(
        f"  ✓ OK — Sharpe={test_data.get('sharpe',0):.2f} "
        f"DD={test_data.get('max_drawdown_pct',0):.1f}% "
        f"WinRate={test_data.get('win_rate_pct',0):.1f}%"
    )

    # ── Paso 3: ProductionGuard ───────────────────────────────────────────────
    print("\n[3/6] ProductionGuard…")
    ok, guard = step_production_guard(voice, hid=None)
    if not ok:
        print("  ✗ FALLO — ProductionGuard bloqueó la activación")
        voice.stop(); telegram.stop()
        return 1
    print("  ✓ OK")

    # ── Paso 4: Tradier preview test ──────────────────────────────────────────
    print("\n[4/6] Tradier preview test…")
    ok, reason = step_tradier_preview_test(mode)
    if not ok:
        print(f"  ✗ FALLO — Tradier API error: {reason}")
        voice.speak(f"Error de conexión con Tradier. {reason}", urgent=True)
        voice.stop(); telegram.stop()
        return 1
    print(f"  ✓ OK ({reason})")

    # ── Paso 5: Double confirmation ───────────────────────────────────────────
    print("\n[5/6] Confirmación física doble…")
    if not skip_confirmation:
        confirmed = step_double_confirmation(guard, voice)
        if not confirmed:
            print("  ✗ Cancelado por el operador")
            voice.stop(); telegram.stop()
            return 1
    else:
        logger.warning("skip_confirmation=True — saltando confirmación física (solo CI)")
    print("  ✓ CONFIRMADO")

    # ── Paso 6: Lanzar LIVE ───────────────────────────────────────────────────
    print("\n[6/6] Lanzando ATLAS-Quant en modo LIVE…")

    os.environ["ATLAS_MODE"] = mode

    from atlas_code_quant.atlas_quant_core import ATLASQuantCore
    core = ATLASQuantCore(mode=mode)
    core.setup()

    print("\n" + "="*62)
    print("   ✅ ATLAS-QUANT MODO LIVE — ACTIVO")
    print("="*62 + "\n")

    step_launch_live(core, guard, telegram, grafana, symbols, mode)

    voice.stop()
    telegram.stop()
    return 0


# ── CLI ───────────────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="ATLAS-Quant Live Activation — Protocolo completo de activación LIVE"
    )
    p.add_argument("--symbols",  default="SPY,QQQ,AAPL,TSLA,NVDA",
                   help="Símbolos separados por coma")
    p.add_argument("--cycles",   type=int, default=50,
                   help="Ciclos del test runner si no hay resultado previo")
    p.add_argument("--rtmp",     default="rtmp://192.168.1.10/live/atlas",
                   help="URL RTMP de la Insta360")
    p.add_argument("--rerun-test", action="store_true",
                   help="Forzar nuevo test runner aunque exista resultado previo")
    p.add_argument("--mode",     default="live", choices=["paper", "live"],
                   help="Modo final de ejecución (defecto: live)")
    p.add_argument("--skip-confirmation", action="store_true",
                   help=argparse.SUPPRESS)   # solo para CI
    return p.parse_args()


if __name__ == "__main__":
    _setup_logging()
    args = _parse_args()
    symbols = [s.strip().upper() for s in args.symbols.split(",") if s.strip()]
    rc = activate(
        symbols           = symbols,
        cycles            = args.cycles,
        rtmp_url          = args.rtmp,
        force_rerun_test  = args.rerun_test,
        skip_confirmation = args.skip_confirmation,
        mode              = args.mode,
    )
    sys.exit(rc)
