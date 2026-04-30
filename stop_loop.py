import urllib.request, json

req = urllib.request.Request(
    "http://127.0.0.1:8795/api/v2/quant/operation/loop/stop",
    data=b"{}",
    headers={"Content-Type": "application/json", "X-Api-Key": "atlas-quant-local"},
    method="POST"
)
try:
    with urllib.request.urlopen(req, timeout=10) as resp:
        print("Loop stop:", resp.read().decode()[:200])
except Exception as e:
    print("Error:", e)

