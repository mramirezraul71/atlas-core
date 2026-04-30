import os, re

path = r"C:\ATLAS_PUSH\atlas_code_quant\api\main.py"
with open(path, 'r', encoding='utf-8') as f:
    content = f.read()

# ── PATCH 1: exit_pass — skip OS en urgent_exits ────────────────────────────
# Insertar después de:  sym = str(pos.get("symbol") or "").strip().upper()
#                       if not sym:
#                           continue
# Agregar:              if sym in _SANDBOX_RESTRICTED_SYMBOLS:
#                           continue

OLD_EXIT = '''                            sym = str(pos.get("symbol") or "").strip().upper()
                            if not sym:
                                continue
                            # strategy_type determina la dirección original de la posición'''

NEW_EXIT = '''                            sym = str(pos.get("symbol") or "").strip().upper()
                            if not sym:
                                continue
                            # Symbols restricted in Tradier sandbox — skip exit and entry
                            if sym in _SANDBOX_RESTRICTED_SYMBOLS:
                                logger.debug("[auto-cycle] skip sandbox-restricted symbol=%s", sym)
                                continue
                            # strategy_type determina la dirección original de la posición'''

# ── PATCH 2: entry_pass — filtrar sorted_cands ──────────────────────────────
OLD_ENTRY = '''                sorted_cands = sorted(
                    candidates,
                    key=lambda c: float(c.get("selection_score") or 0),
                    reverse=True,
                )'''

NEW_ENTRY = '''                sorted_cands = sorted(
                    [c for c in candidates if str(c.get("symbol") or "").strip().upper() not in _SANDBOX_RESTRICTED_SYMBOLS],
                    key=lambda c: float(c.get("selection_score") or 0),
                    reverse=True,
                )'''

# ── PATCH 3: Definir _SANDBOX_RESTRICTED_SYMBOLS cerca del top del módulo ───
# Insertar después de "_AUTO_CYCLE_STATE: dict = {"
OLD_CONST = '_AUTO_CYCLE_STATE: dict = {'
NEW_CONST = '''# Symbols that Tradier sandbox cannot trade (asset class restricted).
# These are excluded from both exit_pass and entry_pass in the auto-cycle.
_SANDBOX_RESTRICTED_SYMBOLS: frozenset[str] = frozenset(
    s.strip().upper()
    for s in os.getenv("ATLAS_SANDBOX_RESTRICTED_SYMBOLS", "OS").split(",")
    if s.strip()
)

_AUTO_CYCLE_STATE: dict = {'''

# Aplicar patches
if OLD_EXIT not in content:
    print("ERROR: OLD_EXIT not found — puede que la indentación difiera")
else:
    content = content.replace(OLD_EXIT, NEW_EXIT, 1)
    print("PATCH 1 (exit_pass): OK")

if OLD_ENTRY not in content:
    print("ERROR: OLD_ENTRY not found")
else:
    content = content.replace(OLD_ENTRY, NEW_ENTRY, 1)
    print("PATCH 2 (entry_pass): OK")

if OLD_CONST not in content:
    print("ERROR: OLD_CONST not found")
elif '_SANDBOX_RESTRICTED_SYMBOLS' in content.split(OLD_CONST)[0]:
    print("PATCH 3: ya aplicado, skip")
else:
    content = content.replace(OLD_CONST, NEW_CONST, 1)
    print("PATCH 3 (constant): OK")

with open(path, 'w', encoding='utf-8') as f:
    f.write(content)

print("Archivo guardado.")

# Verificar que los patches están
lines = content.split('\n')
for i,l in enumerate(lines):
    if '_SANDBOX_RESTRICTED_SYMBOLS' in l:
        print(f"  L{i+1}: {l.strip()[:80]}")
