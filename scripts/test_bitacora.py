#!/usr/bin/env python
"""Prueba la conexión a la Bitácora Central."""
import urllib.request
import json
import time
from pathlib import Path

base = "http://127.0.0.1:8791"

# Enviar eventos de prueba
print("=== Enviando eventos a Bitacora ===")
eventos = [
    {"message": "[SISTEMA] Dashboard reiniciado en puerto 8791", "ok": True, "source": "startup"},
    {"message": "[EVOLUCION] Ciclo de Triada ejecutado: PyPI, GitHub, HuggingFace", "ok": True, "source": "evolution"},
    {"message": "[PYPI] httpx actualizado 0.25.0 -> 0.28.1", "ok": True, "source": "evolution"},
    {"message": "[WHATSAPP] Servicio WAHA conectado y autenticado", "ok": True, "source": "comms"},
]

for e in eventos:
    data = json.dumps(e).encode()
    req = urllib.request.Request(base + "/ans/evolution-log", data=data, headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            print(f"  Enviado: {e['message'][:40]}")
    except Exception as ex:
        print(f"  Error: {ex}")

time.sleep(1)

# Verificar GET /ans/bitacora
print("")
print("=== GET /ans/bitacora ===")
try:
    req = urllib.request.Request(base + "/ans/bitacora?limit=10")
    with urllib.request.urlopen(req, timeout=5) as r:
        data = json.loads(r.read().decode())
        entries = data.get("data", [])
        print(f"Total entradas: {len(entries)}")
        for e in entries[:8]:
            src = e.get("source", "?")
            det = e.get("detalle", "")[:50]
            res = e.get("resultado", "")
            print(f"  [{src}] {det} | {res}")
except Exception as e:
    print(f"Error: {e}")
