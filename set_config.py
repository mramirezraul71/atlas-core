import urllib.request, json

# Primero GET para ver campos disponibles
req_get = urllib.request.Request(
    "http://127.0.0.1:8795/api/v2/quant/operation/config",
    headers={"X-Api-Key": "atlas-quant-local"},
)
try:
    with urllib.request.urlopen(req_get, timeout=10) as resp:
        data = json.loads(resp.read())
    print("GET config keys:", list(data.get("data", data).keys() if isinstance(data.get("data",data), dict) else []))
    print("Full:", json.dumps(data, indent=2)[:500])
except Exception as e:
    print("GET Error:", e)

# POST para desactivar var_monitor
payload = {
    "var_monitor_enabled": False,
    "signal_validator_enabled": True,
    "fail_safe_active": False
}
req_post = urllib.request.Request(
    "http://127.0.0.1:8795/api/v2/quant/operation/config",
    data=json.dumps(payload).encode(),
    headers={"Content-Type": "application/json", "X-Api-Key": "atlas-quant-local"},
    method="POST"
)
try:
    with urllib.request.urlopen(req_post, timeout=10) as resp:
        result = json.loads(resp.read())
    print("POST result:", json.dumps(result, indent=2)[:500])
except Exception as e:
    print("POST Error:", e)

