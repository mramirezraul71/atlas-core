#!/usr/bin/env python3
"""
ATLAS Unified Launcher
=======================
Script unificado para iniciar TODO el sistema ATLAS:
- Servidor HTTP (Dashboard + API)
- Sistema Cognitivo completo
- Voice Assistant
- Autonomía completa

USO:
    python atlas_launcher.py           # Inicia todo en modo servidor
    python atlas_launcher.py --ui      # Abre también el navegador
"""
import argparse
import logging
import os
import signal
import sys
import threading
import time
import webbrowser

# Asegurar path
ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

# Configuración
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8791
DASHBOARD_URL = f"http://127.0.0.1:{HTTP_PORT}/ui"

# Estado global
_shutdown_event = threading.Event()


def setup_logging() -> None:
    """Configura logging."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
        datefmt="%H:%M:%S",
    )


def print_banner() -> None:
    """Banner de ATLAS."""
    banner = """
    ╔═══════════════════════════════════════════════════════════════╗
    ║                                                               ║
    ║     █████╗ ████████╗██╗      █████╗ ███████╗                  ║
    ║    ██╔══██╗╚══██╔══╝██║     ██╔══██╗██╔════╝                  ║
    ║    ███████║   ██║   ██║     ███████║███████╗                  ║
    ║    ██╔══██║   ██║   ██║     ██╔══██║╚════██║                  ║
    ║    ██║  ██║   ██║   ███████╗██║  ██║███████║                  ║
    ║    ╚═╝  ╚═╝   ╚═╝   ╚══════╝╚═╝  ╚═╝╚══════╝                  ║
    ║                                                               ║
    ║         COGNITIVE BRAIN ARCHITECTURE v3.8.0                   ║
    ║         Unified Launcher - Full System                        ║
    ║                                                               ║
    ╚═══════════════════════════════════════════════════════════════╝
    """
    try:
        print(banner)
    except UnicodeEncodeError:
        print("[ATLAS] COGNITIVE BRAIN ARCHITECTURE v3.8.0 - Unified Launcher")


def init_cognitive_system() -> dict:
    """Inicializa el sistema cognitivo."""
    print("\n[1/4] Inicializando Sistema Cognitivo...")
    try:
        from modules.humanoid import get_cognitive_system

        system = get_cognitive_system()
        modules_count = sum(
            len(v) if isinstance(v, dict) else 1 for v in system.values()
        )
        print(f"      [OK] {modules_count} modulos cognitivos cargados")
        return system
    except Exception as e:
        print(f"      [WARN] Sistema cognitivo parcial: {e}")
        return {}


def init_autonomy() -> bool:
    """Inicializa el sistema de autonomía en background (no bloquea el arranque)."""
    print("\n[2/4] Iniciando Autonomia (background)...")

    def _start_bg():
        try:
            import time as _t

            _t.sleep(8)
            from modules.humanoid.quality.autonomy_daemon import (
                AutonomyConfig, start_autonomy)

            config = AutonomyConfig(
                enable_auto_commit=True,
                enable_auto_repair=True,
                enable_scheduled_maintenance=True,
                enable_incident_response=True,
                notify_on_start=False,
                notify_on_critical=True,
                telegram_on_errors=True,
            )
            result = start_autonomy(config)
            ok_count = sum(
                1 for v in result.values() if isinstance(v, dict) and v.get("ok")
            )
            total = len([v for v in result.values() if isinstance(v, dict)])
            print(f"      [Autonomy] {ok_count}/{total} componentes activos")
        except Exception as e:
            print(f"      [Autonomy] parcial: {e}")

    threading.Thread(target=_start_bg, daemon=True).start()
    print("      [OK] Autonomia programada (arranca tras HTTP)")
    return True


def init_voice() -> bool:
    """Inicializa el asistente de voz."""
    print("\n[3/4] Inicializando Voice Assistant...")
    try:
        voice_path = os.path.join(ROOT, "modules", "humanoid", "voice")
        if os.path.exists(voice_path):
            print("      [OK] Voice Assistant disponible")
            return True
        else:
            print("      [WARN] Voice Assistant no instalado")
            return False
    except Exception as e:
        print(f"      [WARN] Voice: {e}")
        return False


def start_http_server(open_browser: bool = False) -> None:
    """Inicia el servidor HTTP con dashboard."""
    print(f"\n[4/4] Iniciando Servidor HTTP en puerto {HTTP_PORT}...")

    try:
        import uvicorn

        from atlas_adapter.atlas_http_api import app

        print(f"      [OK] Dashboard: {DASHBOARD_URL}")
        print(f"      [OK] API: http://127.0.0.1:{HTTP_PORT}/docs")
        print(
            f"      [OK] Cognitive API: http://127.0.0.1:{HTTP_PORT}/cognitive/status"
        )

        if open_browser:

            def open_browser_delayed():
                time.sleep(2)
                webbrowser.open(DASHBOARD_URL)

            threading.Thread(target=open_browser_delayed, daemon=True).start()

        print("\n" + "=" * 65)
        print("  [ONLINE] ATLAS SISTEMA COMPLETO ACTIVO")
        print("=" * 65)
        print(f"\n  Dashboard: {DASHBOARD_URL}")
        print("  Presiona Ctrl+C para detener.\n")

        # Ejecutar servidor
        uvicorn.run(
            app,
            host=HTTP_HOST,
            port=HTTP_PORT,
            log_level="warning",
            access_log=False,
        )

    except ImportError:
        print("      [ERROR] uvicorn no instalado. Ejecuta: pip install uvicorn")
        sys.exit(1)
    except Exception as e:
        print(f"      [ERROR] Error iniciando servidor: {e}")
        sys.exit(1)


def signal_handler(signum, frame):
    """Maneja señales de terminación."""
    print("\n\n[ATLAS] Recibida señal de terminación...")
    _shutdown_event.set()

    # Detener autonomía
    try:
        from modules.humanoid.quality.autonomy_daemon import stop_autonomy

        stop_autonomy()
    except:
        pass

    print("[ATLAS] Sistema detenido.\n")
    sys.exit(0)


def main():
    """Función principal."""
    global HTTP_PORT, DASHBOARD_URL

    parser = argparse.ArgumentParser(description="ATLAS Unified Launcher")
    parser.add_argument(
        "--ui", action="store_true", help="Abrir navegador automáticamente"
    )
    parser.add_argument("--port", type=int, default=HTTP_PORT, help="Puerto HTTP")
    args = parser.parse_args()

    HTTP_PORT = args.port
    DASHBOARD_URL = f"http://127.0.0.1:{HTTP_PORT}/ui"

    # Configurar señales
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    setup_logging()
    print_banner()

    print("\n" + "=" * 65)
    print("  INICIANDO ATLAS - Sistema Unificado")
    print("=" * 65)

    # 1. Sistema cognitivo
    init_cognitive_system()

    # 2. Autonomía
    init_autonomy()

    # 3. Voice
    init_voice()

    # 4. Servidor HTTP (bloqueante)
    start_http_server(open_browser=args.ui)


if __name__ == "__main__":
    main()
