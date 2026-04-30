import urllib.request, json

req = urllib.request.Request(
    "http://127.0.0.1:8795/openapi.json",
    headers={"X-Api-Key": "atlas-quant-local"},
)
try:
    with urllib.request.urlopen(req, timeout=10) as resp:
        schema = json.loads(resp.read())
    paths = list(schema.get("paths", {}).keys())
    for p in sorted(paths):
        print(p)
except Exception as e:
    print("Error:", e)

