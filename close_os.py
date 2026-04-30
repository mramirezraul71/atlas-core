import requests
import json
import time

TOKEN = "UqYAFhBY0sPWSP4OmuAHChHB0lAN"
ACCOUNT = "VA9201365"
BASE = "https://sandbox.tradier.com/v1"
HEADERS = {
    "Authorization": f"Bearer {TOKEN}",
    "Accept": "application/json",
    "Content-Type": "application/x-www-form-urlencoded"
}

print("=== ATLAS OS Position Closer ===")

# 1. Verificar posición actual
r = requests.get(f"{BASE}/accounts/{ACCOUNT}/positions", headers=HEADERS)
print(f"Positions response: {r.status_code} - {r.text}")

# 2. Intentar limit order muy bajo (forzar fill)
for price in ["0.01", "0.001"]:
    try:
        r = requests.post(f"{BASE}/accounts/{ACCOUNT}/orders", headers=HEADERS, data={
            "class": "equity",
            "symbol": "OS",
            "side": "sell",
            "quantity": "30",
            "type": "limit",
            "price": price,
            "duration": "day"
        })
        print(f"Limit ${price} order: {r.status_code} - {r.text}")
        time.sleep(1)
    except Exception as e:
        print(f"Error: {e}")

# 3. Ver si hay endpoint de administración o ajuste de posición
try:
    r = requests.get(f"{BASE}/accounts/{ACCOUNT}/balances", headers=HEADERS)
    print(f"Balances: {r.status_code} - {r.text[:500]}")
except Exception as e:
    print(f"Balances error: {e}")

print("=== Done ===")
