import urllib.request, json

# Buscar todos los endpoints disponibles
req = urllib.request.Request(
    "http://127.0.0.1:8795/openapi.json",
    headers={"X-Api-Key": "atlas-quant-local"},
)
try:
    with urllib.request.urlopen(req, timeout=10) as resp:
        schema = json.loads(resp.read())
    paths = list(schema.get("paths", {}).keys())
    # Filtrar los relacionados con loop/cycle/auto
    for p in paths:
        if any(x in p.lower() for x in ["loop", "cycle", "auto", "start", "stop"]):
            print(p)
except Exception as e:
    print("Error:", e)

