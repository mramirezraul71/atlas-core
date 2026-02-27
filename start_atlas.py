#!/usr/bin/env python3
"""
ATLAS Startup Script
=====================
Script principal para iniciar ATLAS en modo autónomo completo.

USO:
    python start_atlas.py              # Inicia en modo daemon
    python start_atlas.py --foreground # Inicia en primer plano
    python start_atlas.py --status     # Muestra estado
    python start_atlas.py --stop       # Detiene el daemon

Este script arranca TODO el sistema de autonomía:
1. Dispatcher de POTs
2. Motor de Triggers
3. Health Monitor
4. Watchdog
5. Tareas programadas
6. Conexión con Cerebro
"""
import argparse
import logging
import os
import sys
import time

# Asegurar que el path del proyecto está en sys.path
ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def setup_logging(verbose: bool = False) -> None:
    """Configura el logging."""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def print_banner() -> None:
    """Imprime el banner de ATLAS."""
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
    ║              AUTONOMOUS ROBOTIC SYSTEM                        ║
    ║              Version 2.0 - Full Autonomy                      ║
    ║                                                               ║
    ╚═══════════════════════════════════════════════════════════════╝
    """
    print(banner)


def start_daemon(foreground: bool = False) -> None:
    """Inicia el daemon de autonomía."""
    from modules.humanoid.quality.autonomy_daemon import start_autonomy, AutonomyConfig
    
    print_banner()
    print("\n[ATLAS] Iniciando sistema de autonomía completa...\n")
    
    # Configuración
    config = AutonomyConfig(
        enable_auto_commit=True,
        enable_auto_repair=True,
        enable_scheduled_maintenance=True,
        enable_incident_response=True,
        notify_on_start=True,
        notify_on_critical=True,
        telegram_on_errors=True,
    )
    
    # Iniciar
    result = start_autonomy(config)
    
    # Mostrar resultados
    print("\n" + "=" * 60)
    print("RESULTADO DEL ARRANQUE")
    print("=" * 60)
    
    for component, status in result.items():
        if component == "all_ok":
            continue
        if isinstance(status, dict):
            ok = status.get("ok", False)
            symbol = "✅" if ok else "❌"
            print(f"  {symbol} {component}: {'OK' if ok else 'FAILED'}")
            if not ok and "error" in status:
                print(f"      Error: {status['error']}")
    
    print("=" * 60)
    
    if result.get("all_ok"):
        print("\n🟢 ATLAS AUTONOMÍA ACTIVA - Sistema 100% operacional\n")
    else:
        print("\n🟡 ATLAS AUTONOMÍA PARCIAL - Algunos componentes fallaron\n")
    
    if foreground:
        print("[ATLAS] Ejecutando en primer plano. Presiona Ctrl+C para detener.\n")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[ATLAS] Deteniendo...")
            from modules.humanoid.quality.autonomy_daemon import stop_autonomy
            stop_autonomy()
            print("[ATLAS] Detenido.\n")


def show_status() -> None:
    """Muestra el estado del sistema."""
    try:
        from modules.humanoid.quality.autonomy_daemon import get_autonomy_status, is_autonomy_running
        
        print_banner()
        
        if not is_autonomy_running():
            print("\n🔴 ATLAS Autonomía: NO ACTIVA\n")
            return
        
        status = get_autonomy_status()
        
        print("\n" + "=" * 60)
        print("ESTADO DE ATLAS AUTONOMÍA")
        print("=" * 60)
        
        print(f"\n  Estado: {'🟢 ACTIVO' if status['running'] else '🔴 INACTIVO'}")
        print(f"  Inicio: {status['started_at']}")
        print(f"  Uptime: {status['uptime_seconds']} segundos")
        
        print("\n  Configuración:")
        for key, value in status.get("config", {}).items():
            print(f"    - {key}: {value}")
        
        print("\n  Health Checks:")
        for name, health in status.get("health", {}).items():
            symbol = "✅" if health.get("healthy") else "❌"
            print(f"    {symbol} {name}")
        
        print("\n" + "=" * 60 + "\n")
        
    except Exception as e:
        print(f"\n❌ Error obteniendo estado: {e}\n")


def stop_daemon() -> None:
    """Detiene el daemon."""
    try:
        from modules.humanoid.quality.autonomy_daemon import stop_autonomy, is_autonomy_running
        
        if not is_autonomy_running():
            print("\n[ATLAS] El sistema no está corriendo.\n")
            return
        
        print("\n[ATLAS] Deteniendo sistema de autonomía...")
        result = stop_autonomy()
        
        all_ok = all(r.get("ok", False) for r in result.values() if isinstance(r, dict))
        
        if all_ok:
            print("[ATLAS] Sistema detenido correctamente.\n")
        else:
            print("[ATLAS] Sistema detenido con algunos errores.\n")
            
    except Exception as e:
        print(f"\n❌ Error deteniendo: {e}\n")


def main() -> None:
    """Función principal."""
    parser = argparse.ArgumentParser(
        description="ATLAS Autonomous System Startup",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    
    parser.add_argument(
        "--foreground", "-f",
        action="store_true",
        help="Ejecutar en primer plano (no daemon)",
    )
    parser.add_argument(
        "--status", "-s",
        action="store_true",
        help="Mostrar estado del sistema",
    )
    parser.add_argument(
        "--stop",
        action="store_true",
        help="Detener el daemon",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Modo verbose (más logging)",
    )
    
    args = parser.parse_args()
    
    setup_logging(args.verbose)
    
    if args.status:
        show_status()
    elif args.stop:
        stop_daemon()
    else:
        start_daemon(foreground=args.foreground)


if __name__ == "__main__":
    main()
