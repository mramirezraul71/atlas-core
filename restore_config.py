import urllib.request, json

payload = {
    "auton_mode": "paper_autonomous",
    "executor_mode": "paper_api",
    "vision_mode": "off",
    "require_operator_present": False,
    "min_auton_win_rate_pct": 35.0,
    "operational_error_limit": 10
}
req = urllib.request.Request(
    "http://127.0.0.1:8795/api/v2/quant/operation/config",
    data=json.dumps(payload).encode(),
    headers={"Content-Type": "application/json", "X-Api-Key": "atlas-quant-local"},
    method="POST"
)
try:
    with urllib.request.urlopen(req, timeout=10) as resp:
        result = json.loads(resp.read())
    cfg = result.get("data", {}).get("config", {})
    print("auton_mode:", cfg.get("auton_mode"))
    print("vision_mode:", cfg.get("vision_mode"))
    print("ok:", result.get("ok"))
except Exception as e:
    print("Error:", e)

