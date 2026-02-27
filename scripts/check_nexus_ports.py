#!/usr/bin/env python3
"""Diagnóstico: comprueba si NEXUS (8000) y Robot (8002) responden. Para revisar por qué no se conecta."""
import sys
import urllib.request
import json
from pathlib import Path
from typing import Tuple

# Raíz del repo
ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

def check(url: str, name: str, timeout: int = 3) -> Tuple[bool, str]:
    """GET url. Devuelve (ok, mensaje)."""
    try:
        req = urllib.request.Request(url, method="GET", headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            if r.status != 200:
                return False, f"HTTP {r.status}"
            try:
                data = json.loads(r.read().decode("utf-8"))
                return True, json.dumps(data)[:120] + ("..." if len(json.dumps(data)) > 120 else "")
            except Exception:
                return True, "OK (no JSON)"
    except urllib.error.HTTPError as e:
        return False, f"HTTP {e.code}"
    except Exception as e:
        return False, str(e)[:80]


def main():
    print("=== Diagnóstico conexión NEXUS / Robot ===\n")

    # NEXUS (8000): /health o /status
    url_nexus = "http://127.0.0.1:8000/health"
    ok_nexus, msg_nexus = check(url_nexus, "NEXUS")
    if not ok_nexus and "404" in msg_nexus:
        url_nexus = "http://127.0.0.1:8000/status"
        ok_nexus, msg_nexus = check(url_nexus, "NEXUS")
    if ok_nexus:
        print("NEXUS (8000): CONECTADO")
        print("  ", msg_nexus)
    else:
        print("NEXUS (8000): DESCONECTADO")
        print("  ", msg_nexus)
        print("  Acción: arrancar NEXUS (ej. cd nexus/atlas_nexus && python nexus.py --mode api)")

    print()

    # Robot (8002)
    url_robot = "http://127.0.0.1:8002/api/health"
    ok_robot, msg_robot = check(url_robot, "Robot")
    if not ok_robot:
        url_robot_alt = "http://127.0.0.1:8002/health"
        ok_robot, msg_robot = check(url_robot_alt, "Robot")
    if ok_robot:
        print("Robot (8002): CONECTADO")
        print("  ", msg_robot)
    else:
        print("Robot (8002): DESCONECTADO")
        print("  ", msg_robot)
        print("  Acción: arrancar backend robot (ej. cd nexus/atlas_nexus_robot/backend && python main.py)")

    print()
    if ok_nexus and ok_robot:
        print("Resumen: ambos servicios responden. El dashboard debería mostrar Conectado.")
    elif ok_nexus:
        print("Resumen: solo NEXUS responde. Robot (cámaras) no está en marcha.")
    elif ok_robot:
        print("Resumen: solo Robot responde. NEXUS (8000) no está en marcha.")
    else:
        print("Resumen: ni NEXUS ni Robot responden. Arrancar ambos (ver docs/PROMPT_NEXUS_CLAUDE_MAPEO.md sección 7).")


if __name__ == "__main__":
    main()
