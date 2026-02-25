#!/usr/bin/env python3
"""
Inicio manual de Home Assistant para Atlas
"""

import os
import subprocess
import sys
from pathlib import Path


def start_homeassistant():
    """Inicia Home Assistant manualmente"""
    config_dir = Path("c:/ATLAS_PUSH/homeassistant_config")

    print("Iniciando Home Assistant...")
    print(f"Directorio de configuracion: {config_dir}")

    try:
        # Iniciar Home Assistant
        cmd = [
            sys.executable,
            "-m",
            "homeassistant",
            "--config",
            str(config_dir),
            "--port",
            "8123",
        ]

        print(f"Comando: {' '.join(cmd)}")
        print("Home Assistant se iniciara en: http://localhost:8123")
        print("Presiona Ctrl+C para detener")

        subprocess.run(cmd)

    except KeyboardInterrupt:
        print("\nHome Assistant detenido")
    except Exception as e:
        print(f"Error iniciando Home Assistant: {e}")


if __name__ == "__main__":
    start_homeassistant()
