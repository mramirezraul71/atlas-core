
import sys
import urllib.request
import urllib.parse
import json

TOKEN = "UqYAFhBY0sPWSP4OmuAHChHB0lAN"
ACCOUNT = "VA9201365"
BASE_URL = "https://sandbox.tradier.com/v1"

def tradier_get(path):
    req = urllib.request.Request(
        f"{BASE_URL}{path}",
        headers={"Authorization": f"Bearer {TOKEN}", "Accept": "application/json"}
    )
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            return json.loads(r.read())
    except Exception as e:
        return {"error": str(e)}

def tradier_post(path, data):
    payload = urllib.parse.urlencode(data).encode()
    req = urllib.request.Request(
        f"{BASE_URL}{path}",
        data=payload,
        headers={"Authorization": f"Bearer {TOKEN}", "Accept": "application/json"}
    )
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            return json.loads(r.read())
    except Exception as e:
        return {"error": str(e)}

print("=== Posiciones en Tradier sandbox ===")
resp = tradier_get(f"/accounts/{ACCOUNT}/positions")
positions = resp.get("positions", {})
if not positions or positions == "null":
    print("Sin posiciones")
    sys.exit(0)
pos_list = positions.get("position", [])
if isinstance(pos_list, dict):
    pos_list = [pos_list]
print(f"Total: {len(pos_list)}")
for p in pos_list:
    sym = p.get("symbol")
    qty = float(p.get("quantity", 0))
    close_side = "sell" if qty > 0 else "buy"
    abs_qty = int(abs(qty))
    print(f"Cerrando {sym} qty={abs_qty} side={close_side}")
    r = tradier_post(f"/accounts/{ACCOUNT}/orders", {
        "class": "equity", "symbol": sym,
        "side": close_side, "quantity": str(abs_qty),
        "type": "market", "duration": "day", "preview": "false"
    })
    print(f"  -> {r}")
print("DONE")
