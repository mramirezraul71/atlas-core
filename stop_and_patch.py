"""
Stop loop + patch AA en _SANDBOX_RESTRICTED_SYMBOLS
"""
import urllib.request
import urllib.error
import json
import sys
from pathlib import Path

API = "http://127.0.0.1:8795"
HEADERS = {"Content-Type": "application/json", "X-API-Key": "atlas-quant-local"}

def api_post(path, body=None):
    data = json.dumps(body or {}).encode()
    req = urllib.request.Request(f"{API}{path}", data=data, headers=HEADERS, method="POST")
    try:
        r = urllib.request.urlopen(req, timeout=10)
        return json.loads(r.read())
    except Exception as e:
        return {"error": str(e)}

# 1. Detener loop
print("=== STOP LOOP ===")
r = api_post("/operation/loop/stop")
print(json.dumps(r, indent=2))

# 2. Patch main.py: agregar AA
print("\n=== PATCH AA ===")
path = Path(r"C:\ATLAS_PUSH\atlas_code_quant\api\main.py")
if not path.exists():
    print(f"ERROR: No se encontró {path}")
    sys.exit(1)

with open(path, "r", encoding="utf-8") as f:
    content = f.read()

old = '_SANDBOX_RESTRICTED_SYMBOLS: frozenset[str] = frozenset({"OS"})'
new = '_SANDBOX_RESTRICTED_SYMBOLS: frozenset[str] = frozenset({"OS", "AA"})'

if new in content:
    print("YA APLICADO: AA ya en frozenset")
elif old in content:
    content = content.replace(old, new)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"PATCH OK: {new}")
else:
    import re
    m = re.search(r"_SANDBOX_RESTRICTED_SYMBOLS.*?frozenset\([^\n]+\)", content)
    print(f"LÍNEA ACTUAL: {m.group() if m else 'NO ENCONTRADA'}")
    # Intentar reemplazo regex
    new_content = re.sub(
        r"(_SANDBOX_RESTRICTED_SYMBOLS: frozenset\[str\] = frozenset\(\{)([^\}]+)(\}\))",
        r'\g<1>"OS", "AA"\3',
        content
    )
    if new_content != content:
        with open(path, "w", encoding="utf-8") as f:
            f.write(new_content)
        print("PATCH OK via regex")
    else:
        print("ERROR: no se pudo patchear")
        sys.exit(1)

# 3. Verificar resultado
with open(path, "r", encoding="utf-8") as f:
    content = f.read()
m = __import__("re").search(r"_SANDBOX_RESTRICTED_SYMBOLS.*?frozenset\([^\n]+\)", content)
print(f"\nVERIFICACIÓN: {m.group() if m else 'NO ENCONTRADO'}")
print("\nNOTA: Reinicia uvicorn para activar el patch en memoria")
