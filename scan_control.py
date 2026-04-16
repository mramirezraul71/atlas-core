import requests
import json

headers = {"X-API-Key": "atlas-quant-local", "Content-Type": "application/json"}
base = "http://127.0.0.1:8795"

# Ver qué acepta scanner/control
r = requests.get(f"{base}/openapi.json", timeout=10)
api = r.json()
paths = api.get("paths", {})

# Buscar scanner control
for path, methods in paths.items():
    if 'scanner' in path.lower():
        for method, details in methods.items():
            print(f"\n{method.upper()} {path}")
            body = details.get('requestBody', {})
            if body:
                schema = body.get('content', {}).get('application/json', {}).get('schema', {})
                print(f"  Body: {json.dumps(schema, indent=2)[:300]}")

# Intentar llamar scanner control para forzar ciclo inmediato
print("\n=== Intentando trigger de scanner ===")
payloads = [
    {"action": "run_now"},
    {"action": "start"},
    {"trigger": "immediate"},
    {"force_cycle": True},
]
for payload in payloads:
    try:
        r = requests.post(f"{base}/api/v2/quant/scanner/control", 
                         headers=headers, json=payload, timeout=10)
        print(f"POST scanner/control {json.dumps(payload)}: {r.status_code} - {r.text[:200]}")
        if r.status_code == 200:
            break
    except Exception as e:
        print(f"Error: {e}")
