import requests
import json

r = requests.get("http://127.0.0.1:8795/openapi.json", 
                 headers={"X-API-Key": "atlas-quant-local"}, timeout=10)
api = r.json()

# Listar todos los paths
paths = list(api.get("paths", {}).keys())
print(f"Total endpoints: {len(paths)}")
print("\nEndpoints relevantes:")
for p in sorted(paths):
    if any(k in p.lower() for k in ['cycle', 'scan', 'journal', 'position', 'auto', 'trigger', 'run', 'start']):
        # Obtener métodos
        methods = list(api["paths"][p].keys())
        print(f"  {','.join(methods).upper()} {p}")
