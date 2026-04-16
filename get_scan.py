import requests
import json

headers = {"X-API-Key": "atlas-quant-local"}
base = "http://127.0.0.1:8795"

# Scanner report
r = requests.get(f"{base}/api/v2/quant/scanner/report", headers=headers, timeout=15)
print(f"Scanner report: {r.status_code}")
if r.status_code == 200:
    data = r.json()
    # Mostrar candidatos top
    report = data.get('data', {})
    candidates = report.get('candidates', [])
    print(f"Total candidates: {len(candidates)}")
    for c in candidates[:10]:
        print(f"  {c.get('symbol')} | score={c.get('selection_score')} | strat={c.get('strategy_type')} | signal={c.get('signal_strength')}")
    
    print(f"\nLast scan: {report.get('last_scan_at')}")
    print(f"Universe size: {report.get('universe_size')}")
    print(f"Status: {report.get('status')}")
else:
    print(r.text[:500])

# Ver si hay candidatos activos para equity
r2 = requests.get(f"{base}/api/v2/quant/positions", headers=headers, timeout=10)
print(f"\nAll positions: {r2.status_code} - {r2.text[:200]}")
