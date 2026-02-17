#!/usr/bin/env python
"""Test completo del sistema de comunicación de ATLAS."""
from __future__ import annotations

import sys


def main():
    print("=== PRUEBA COMPLETA DEL SISTEMA DE COMUNICACIÓN ===")
    print()

    # 1. Cargar configuración
    try:
        from modules.humanoid.config.vault import load_vault_env
        load_vault_env(override=False)
        print("1. Bóveda cargada OK")
    except Exception as e:
        print(f"1. Bóveda ERROR: {e}")
    print()

    # 2. Bootstrap del sistema
    try:
        from modules.humanoid.comms.bootstrap import bootstrap_comms
        print("2. Bootstrap de servicios...")
        result = bootstrap_comms(skip_tests=True)
        print(f"   OK: {result.get('ok')}")
        services = list(result.get("services", {}).keys())
        print(f"   Servicios: {services}")
        warnings = result.get("warnings", [])
        if warnings:
            print(f"   Warnings: {warnings[:2]}")
    except Exception as e:
        print(f"2. Bootstrap ERROR: {e}")
    print()

    # 3. Test Telegram
    try:
        print("3. Test Telegram...")
        from modules.humanoid.comms.telegram_bridge import TelegramBridge
        from modules.humanoid.comms.ops_bus import _telegram_chat_id
        
        bridge = TelegramBridge()
        chat_id = _telegram_chat_id()
        r = bridge.send(chat_id, "<b>ATLAS</b> Sistema de comunicación verificado.")
        print(f"   Chat ID: {chat_id}")
        print(f"   Envío OK: {r.get('ok')}")
    except Exception as e:
        print(f"3. Telegram ERROR: {e}")
    print()

    # 4. Test Audio
    try:
        print("4. Test Audio (TTS)...")
        from modules.humanoid.voice.tts import speak
        r = speak("Sistema de comunicación verificado.")
        print(f"   TTS OK: {r.get('ok')}")
    except Exception as e:
        print(f"4. Audio ERROR: {e}")
    print()

    # 5. Test OPS Bus completo
    try:
        print("5. Test OPS Bus (multicanal)...")
        from modules.humanoid.comms.ops_bus import emit
        r = emit("diagnostico", "Prueba multicanal exitosa", level="high")
        print(f"   Emit OK: {r.get('ok')}")
    except Exception as e:
        print(f"5. OPS Bus ERROR: {e}")
    print()

    # 6. Test nuevo CommsHub
    try:
        print("6. Test CommsHub...")
        from modules.humanoid.comms import get_hub
        hub = get_hub()
        health = hub.get_health()
        print(f"   Hub OK: {health.get('ok')}")
        print(f"   Canales activos: {[k for k, v in health.get('channels', {}).items() if v.get('enabled')]}")
    except Exception as e:
        print(f"6. CommsHub ERROR: {e}")
    print()

    print("=== FIN DE PRUEBAS ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
