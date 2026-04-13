"""
Patch: Agregar AA a _SANDBOX_RESTRICTED_SYMBOLS en main.py
Ejecutar una vez desde C:\ATLAS_PUSH después de git pull
"""
import sys
from pathlib import Path

path = Path(r"C:\ATLAS_PUSH\atlas_code_quant\api\main.py")
if not path.exists():
    print(f"ERROR: No se encontró {path}")
    sys.exit(1)

with open(path, "r", encoding="utf-8") as f:
    content = f.read()

old = '_SANDBOX_RESTRICTED_SYMBOLS: frozenset[str] = frozenset({"OS"})'
new = '_SANDBOX_RESTRICTED_SYMBOLS: frozenset[str] = frozenset({"OS", "AA"})'

if new in content:
    print("YA APLICADO: AA ya está en _SANDBOX_RESTRICTED_SYMBOLS")
elif old in content:
    content = content.replace(old, new)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    print("PATCH OK: AA agregado a _SANDBOX_RESTRICTED_SYMBOLS")
    print(f"Línea nueva: {new}")
else:
    # Buscar variantes
    import re
    m = re.search(r"_SANDBOX_RESTRICTED_SYMBOLS.*?frozenset\([^\)]+\)", content)
    if m:
        print(f"LÍNEA ENCONTRADA (variante): {m.group()}")
        print("Aplicando patch con regex...")
        new_content = re.sub(
            r'_SANDBOX_RESTRICTED_SYMBOLS: frozenset\[str\] = frozenset\(\{[^}]+\}\)',
            new,
            content
        )
        if new_content != content:
            with open(path, "w", encoding="utf-8") as f:
                f.write(new_content)
            print("PATCH OK via regex")
        else:
            print("ERROR: regex no modificó el contenido")
    else:
        print("ERROR: No se encontró _SANDBOX_RESTRICTED_SYMBOLS en main.py")
        sys.exit(1)
