
path = r'C:\ATLAS_PUSH\atlas_code_quant\api\main.py'
with open(path, 'r', encoding='utf-8', errors='ignore') as f:
    content = f.read()

old = '''# ── Auto-cycle loop (scanner → operation_center bridge) ──────────────────────
# Symbols that Tradier sandbox cannot trade (asset class restricted).
# These are excluded from both exit_pass and entry_pass in the auto-cycle.
_SANDBOX_RESTRICTED_SYMBOLS: frozenset[str] = frozenset(
    s.strip().upper()
    for s in os.getenv("ATLAS_SANDBOX_RESTRICTED_SYMBOLS", "OS").split(",")
    if s.strip()
)'''

new = '''# ── Auto-cycle loop (scanner → operation_center bridge) ──────────────────────
# Symbols that Tradier sandbox cannot trade (asset class restricted).
# These are excluded from both exit_pass and entry_pass in the auto-cycle.
_SANDBOX_RESTRICTED_SYMBOLS: frozenset[str] = frozenset({"OS"})'''

if old in content:
    content = content.replace(old, new)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(content)
    print('Patch fixed: os.getenv replaced with frozenset({"OS"})')
else:
    # Check what's actually there
    idx = content.find('_SANDBOX_RESTRICTED_SYMBOLS')
    print('Current patch area:')
    print(content[idx-50:idx+300])
