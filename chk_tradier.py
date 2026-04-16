import urllib.request, json

# Verificar órdenes en Tradier sandbox
token = "UqYAFhBY0sPWSP4OmuAHChHB0lAN"
account = "VA9201365"
base = "https://sandbox.tradier.com/v1"

req = urllib.request.Request(
    f"{base}/accounts/{account}/orders?includeTags=true",
    headers={"Authorization": f"Bearer {token}", "Accept": "application/json"}
)
try:
    with urllib.request.urlopen(req, timeout=10) as resp:
        data = json.loads(resp.read())
    orders = data.get("orders", {}).get("order", [])
    if not isinstance(orders, list):
        orders = [orders]
    # Filtrar órdenes de hoy
    from datetime import datetime
    today = datetime.utcnow().strftime("%Y-%m-%d")
    for o in orders[-10:]:
        created = str(o.get("create_date",""))[:10]
        sym = o.get("symbol","")
        side = o.get("side","")
        status = o.get("status","")
        qty = o.get("quantity","")
        price = o.get("avg_fill_price","")
        oid = o.get("id","")
        print(f"[{created}] {sym} {side} qty={qty} price={price} status={status} id={oid}")
except Exception as e:
    print("Error:", e)

