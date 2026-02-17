#!/usr/bin/env python
"""Verifica incidentes y estado del Workshop."""
import urllib.request
import json

base = "http://127.0.0.1:8791"

# Incidentes abiertos
print("=== Incidentes Abiertos ===")
try:
    req = urllib.request.Request(base + "/ans/incidents?status=open&limit=10")
    with urllib.request.urlopen(req, timeout=5) as r:
        data = json.loads(r.read().decode())
        items = data.get("data", [])
        print(f"Total: {len(items)}")
        for inc in items[:5]:
            cid = inc.get("check_id", "?")
            msg = inc.get("message", "")[:50]
            sev = inc.get("severity", "?")
            print(f"  [{sev}] {cid}: {msg}")
except Exception as e:
    print(f"Error: {e}")

# Estado de Workshop
print("")
print("=== Workshop Status ===")
try:
    req = urllib.request.Request(base + "/ans/workshop/status")
    with urllib.request.urlopen(req, timeout=5) as r:
        data = json.loads(r.read().decode())
        running = data.get("running")
        mode = data.get("mode")
        pending = data.get("pending_count")
        print(f"Running: {running}")
        print(f"Mode: {mode}")
        print(f"Pending: {pending}")
except Exception as e:
    print(f"Error: {e}")

# Ejecutar ciclo ANS
print("")
print("=== Ejecutando ciclo ANS ===")
try:
    req = urllib.request.Request(base + "/ans/run-now", method="POST")
    with urllib.request.urlopen(req, timeout=30) as r:
        data = json.loads(r.read().decode())
        print(f"OK: {data.get('ok')}")
        print(f"Message: {data.get('message', '')[:80]}")
except Exception as e:
    print(f"Error: {e}")
