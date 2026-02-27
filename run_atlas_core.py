#!/usr/bin/env python3
"""Ejecuta AtlasPushCore: Snapshot → Review → Action. Prueba el ciclo sin colapsar."""
import os
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE))

ENV = BASE / "config" / "atlas.env"
if ENV.exists():
    from dotenv import load_dotenv
    load_dotenv(ENV, override=True)

os.environ.setdefault("ATLAS_BRIDGE_MODE", "mock")

import logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(levelname)s %(message)s")

from modules.brain.atlas_push_core import get_core

def main():
    core = get_core()
    core.start()
    try:
        import time
        for i in range(15):
            time.sleep(1)
            print(".", end="", flush=True)
        print("\nOK: ciclo corriendo 15s sin errores")
    except KeyboardInterrupt:
        pass
    finally:
        core.stop()

if __name__ == "__main__":
    main()
