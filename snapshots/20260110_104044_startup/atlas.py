# -*- coding: utf-8 -*-
import time
import threading
from core.logger import log
from core.scheduler import run_scheduler
from modules.snapshot_engine import snapshot

def main():
    # Arranque
    log("ATLAS iniciado. Core activo.")
    snapshot("startup")

    # Scheduler en background
    t = threading.Thread(target=run_scheduler, daemon=True)
    t.start()

    print("ATLAS en ejecucion. Core activo. (Ctrl+C para salir)")

    # Loop principal
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        log("ATLAS detenido manualmente.")
        print("ATLAS detenido.")

if __name__ == "__main__":
    main()
