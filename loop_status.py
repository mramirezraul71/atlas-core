import urllib.request, json

req = urllib.request.Request(
    "http://127.0.0.1:8795/api/v2/quant/operation/loop/status",
    headers={"X-Api-Key": "atlas-quant-local"},
)
try:
    with urllib.request.urlopen(req, timeout=10) as resp:
        result = json.loads(resp.read())
    data = result.get("data", result)
    print("running:", data.get("running"))
    print("cycle_count:", data.get("cycle_count"))
    print("started_at:", data.get("started_at"))
    print("stopped_at:", data.get("stopped_at"))
    print("error:", data.get("error"))
    print("current_stage:", data.get("current_stage"))
except Exception as e:
    print("Error:", e)

