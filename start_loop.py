import urllib.request, json

req = urllib.request.Request(
    "http://127.0.0.1:8795/api/v2/quant/auto-cycle/start",
    data=json.dumps({"loop_interval_sec": 120, "max_per_cycle": 1}).encode(),
    headers={"Content-Type": "application/json", "X-Api-Key": "atlas-quant-local"},
    method="POST"
)
try:
    with urllib.request.urlopen(req, timeout=10) as resp:
        print("Loop start:", resp.read().decode()[:200])
except Exception as e:
    print("Error:", e)

