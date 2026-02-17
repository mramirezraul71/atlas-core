#!/usr/bin/env python
"""Verifica el estado del Dashboard."""
import urllib.request
import json

# Health check del dashboard
print("=== Dashboard Health (8791) ===")
try:
    req = urllib.request.Request("http://127.0.0.1:8791/health")
    with urllib.request.urlopen(req, timeout=10) as r:
        data = json.loads(r.read().decode())
        print(f"OK: {data.get('ok')}")
        print(f"Score: {data.get('score')}")
        checks = data.get("checks", {})
        print(f"API Up: {checks.get('api_up')}")
        print(f"Scheduler: {checks.get('scheduler_alive')}")
        print(f"Active Port: {checks.get('active_port')}")
except Exception as e:
    print(f"Error: {e}")

# Verificar bitacora
print("")
print("=== Bitacora ANS ===")
try:
    req = urllib.request.Request("http://127.0.0.1:8791/ans/bitacora?limit=10")
    with urllib.request.urlopen(req, timeout=5) as r:
        data = json.loads(r.read().decode())
        entries = data.get("entries", [])
        print(f"Total entradas: {data.get('total', 0)}")
        for e in entries[:5]:
            msg = e.get("message", "")[:60]
            print(f"  [{e.get('source', '?')}] {msg}")
except Exception as e:
    print(f"Error: {e}")

# Estado de ANS
print("")
print("=== ANS Status ===")
try:
    req = urllib.request.Request("http://127.0.0.1:8791/ans/status")
    with urllib.request.urlopen(req, timeout=5) as r:
        data = json.loads(r.read().decode())
        print(f"Running: {data.get('running')}")
        print(f"Checks: {data.get('checks_registered')}")
        print(f"Open incidents: {data.get('open_incidents')}")
except Exception as e:
    print(f"Error: {e}")
