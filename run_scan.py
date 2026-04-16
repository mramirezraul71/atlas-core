import requests
import json
import time

headers = {"X-API-Key": "atlas-quant-local", "Content-Type": "application/json"}
base = "http://127.0.0.1:8795"

# Ejecutar run_once
print("=== Ejecutando scanner run_once ===")
r = requests.post(f"{base}/api/v2/quant/scanner/control",
                 headers=headers, json={"action": "run_once"}, timeout=15)
print(f"run_once: {r.status_code} - {r.text[:400]}")

# Esperar y verificar candidatos
time.sleep(5)
r = requests.get(f"{base}/api/v2/quant/scanner/status", headers=headers, timeout=10)
status = r.json().get('data', {})
print(f"\nStatus post-run: cycle_count={status.get('cycle_count')}, accepted={status.get('summary', {}).get('accepted')}")
print(f"Current step: {status.get('current_step')}")
print(f"Current symbol: {status.get('current_symbol')}")

# Ver si hay candidatos ahora
time.sleep(10)
r = requests.get(f"{base}/api/v2/quant/scanner/report", headers=headers, timeout=15)
report = r.json().get('data', {})
candidates = report.get('candidates', [])
print(f"\nCandidates after run_once: {len(candidates)}")
for c in candidates[:5]:
    print(f"  {c.get('symbol')} | score={c.get('selection_score')} | {c.get('strategy_type')} | direction={c.get('direction')}")
