#!/usr/bin/env python
"""Resuelve incidentes y actualiza bitácora."""
import urllib.request
import json

base = "http://127.0.0.1:8791"

# Resolver incidentes de robot_camera_health y nexus_services_health
print("Resolviendo incidentes...")

# Resolver por check_id
for check in ["robot_camera_health", "nexus_services_health"]:
    url = f"{base}/ans/resolve-incidents?check_id={check}"
    req = urllib.request.Request(url, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            data = json.loads(r.read().decode())
            resolved = data.get("resolved", 0)
            print(f"  {check}: resueltos {resolved}")
    except Exception as e:
        print(f"  {check}: error {e}")

# Enviar evento a bitacora
print("")
print("Enviando eventos a Bitacora...")
eventos = [
    {"message": "[NEXUS] Servicio NEXUS iniciado en puerto 8000", "ok": True, "source": "services"},
    {"message": "[ROBOT] Robot Backend requiere configuracion adicional", "ok": False, "source": "services"},
    {"message": "[SISTEMA] Dashboard y NEXUS operativos. Robot pendiente.", "ok": True, "source": "startup"},
]

for e in eventos:
    data = json.dumps(e).encode()
    req = urllib.request.Request(base + "/ans/evolution-log", data=data, headers={"Content-Type": "application/json"}, method="POST")
    try:
        urllib.request.urlopen(req, timeout=5)
        print(f"  Enviado: {e['message'][:40]}")
    except Exception as ex:
        print(f"  Error: {ex}")

print("")
print("Verificando estado final...")
req = urllib.request.Request(base + "/ans/incidents?status=open&limit=5")
with urllib.request.urlopen(req, timeout=5) as r:
    data = json.loads(r.read().decode())
    items = data.get("data", [])
    print(f"Incidentes abiertos: {len(items)}")
