import requests
import json

headers = {"X-API-Key": "atlas-quant-local", "Content-Type": "application/json"}
base = "http://127.0.0.1:8795"

# 1. Ver posiciones paper actuales
r = requests.get(f"{base}/api/v2/quant/paper/positions", headers=headers, timeout=10)
print(f"Paper positions: {r.status_code} - {r.text[:300]}")

# 2. Ver estado del scanner
r = requests.get(f"{base}/api/v2/quant/scanner/status", headers=headers, timeout=10)
print(f"\nScanner status: {r.status_code}")
if r.status_code == 200:
    data = r.json()
    print(json.dumps(data, indent=2)[:500])

# 3. Iniciar loop si no está corriendo
r = requests.post(f"{base}/api/v2/quant/operation/loop/start", 
                  headers=headers, timeout=10,
                  json={})
print(f"\nLoop start: {r.status_code} - {r.text[:300]}")

# 4. Ejecutar test-cycle
r = requests.post(f"{base}/api/v2/quant/operation/test-cycle",
                  headers=headers, timeout=30,
                  json={"action": "submit", "dry_run": False})
print(f"\nTest-cycle: {r.status_code}")
if r.status_code == 200:
    data = r.json()
    print(json.dumps(data, indent=2)[:1000])
else:
    print(r.text[:300])
