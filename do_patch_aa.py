"""
do_patch_aa.py — Aplica patch AA en main.py Y detiene el loop.
No requiere git pull. Se ejecuta directamente via MCP run_script.
Ejecutar: python do_patch_aa.py
"""
import re
import sys
import json
import urllib.request
from pathlib import Path

# ── 1. Detener el loop ────────────────────────────────────────────
print("[1/3] Deteniendo auto-cycle loop...")
try:
    body = json.dumps({}).encode()
    req = urllib.request.Request(
        "http://127.0.0.1:8795/operation/loop/stop",
        data=body,
        headers={"Content-Type": "application/json", "X-API-Key": "atlas-quant-local"},
        method="POST"
    )
    r = urllib.request.urlopen(req, timeout=10)
    resp = json.loads(r.read())
    print(f"  Loop stop: {resp.get('ok', resp)}")
except Exception as e:
    print(f"  Loop stop error (puede ya estar detenido): {e}")

# ── 2. Patch main.py ─────────────────────────────────────────────
print("\n[2/3] Patcheando main.py...")
path = Path(r"C:\ATLAS_PUSH\atlas_code_quant\api\main.py")

if not path.exists():
    print(f"  ERROR: {path} no existe")
    sys.exit(1)

content = path.read_text(encoding="utf-8")

old1 = '_SANDBOX_RESTRICTED_SYMBOLS: frozenset[str] = frozenset({"OS"})'
new1 = '_SANDBOX_RESTRICTED_SYMBOLS: frozenset[str] = frozenset({"OS", "AA"})'

if new1 in content:
    print("  YA APLICADO: AA ya en _SANDBOX_RESTRICTED_SYMBOLS")
elif old1 in content:
    content = content.replace(old1, new1)
    path.write_text(content, encoding="utf-8")
    print(f"  PATCH OK — nueva línea: {new1}")
else:
    # Búsqueda flexible
    m = re.search(r"_SANDBOX_RESTRICTED_SYMBOLS.*?frozenset\([^\n]*\)", content)
    current = m.group() if m else "NO ENCONTRADO"
    print(f"  Línea actual (variante): {current}")
    # Reemplazo regex
    new_content = re.sub(
        r'(_SANDBOX_RESTRICTED_SYMBOLS\s*:\s*frozenset\[str\]\s*=\s*frozenset\(\{)([^\}]+)(\}\))',
        r'\g<1>"OS", "AA"\3',
        content
    )
    if new_content != content:
        path.write_text(new_content, encoding="utf-8")
        print("  PATCH OK via regex")
    else:
        print("  ERROR: no se pudo patchear main.py")
        sys.exit(1)

# ── 3. Verificar patch ───────────────────────────────────────────
print("\n[3/3] Verificando patch...")
content = path.read_text(encoding="utf-8")
m = re.search(r"_SANDBOX_RESTRICTED_SYMBOLS.*?frozenset\([^\n]+\)", content)
if m:
    print(f"  OK: {m.group()}")
else:
    print("  ERROR: no encontrado en archivo")

print("\n[DONE] Patch aplicado en disco.")
print("NOTA: Reinicia uvicorn :8795 para activar AA en memoria.")
print("      OS ya está activo (hardcodeado desde el inicio).")
