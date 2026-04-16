import re, sys, json, urllib.request, sqlite3
from pathlib import Path

# 1. Stop loop
try:
    req = urllib.request.Request('http://127.0.0.1:8795/operation/loop/stop', data=b'{}', headers={'Content-Type':'application/json','X-API-Key':'atlas-quant-local'}, method='POST')
    r = json.loads(urllib.request.urlopen(req, timeout=10).read())
    print('Loop stop:', r.get('ok', r))
except Exception as e:
    print('Loop stop error:', e)

# 2. Patch main.py
p = Path(r'C:\ATLAS_PUSH\atlas_code_quant\api\main.py')
txt = p.read_text(encoding='utf-8')
OLD = '_SANDBOX_RESTRICTED_SYMBOLS: frozenset[str] = frozenset({"OS"})'
NEW = '_SANDBOX_RESTRICTED_SYMBOLS: frozenset[str] = frozenset({"OS", "AA"})'
if NEW in txt:
    print('YA APLICADO')
elif OLD in txt:
    p.write_text(txt.replace(OLD, NEW), encoding='utf-8')
    print('PATCH OK:', NEW)
else:
    m = re.search(r'_SANDBOX_RESTRICTED_SYMBOLS.*', txt)
    print('LINEA ACTUAL:', m.group() if m else 'NO ENCONTRADO')

# 3. Purgar phantoms OS+AA
db = r'C:\ATLAS_PUSH\atlas_code_quant\data\journal\trading_journal.sqlite3'
conn = sqlite3.connect(db)
cur = conn.cursor()
cur.execute("DELETE FROM trading_journal WHERE symbol IN ('OS','AA') AND broker_order_ids_json IN ('{}','[]','')")
print('Phantoms eliminados:', cur.rowcount)
conn.commit()
conn.close()

# 4. Verificar
txt2 = p.read_text(encoding='utf-8')
m2 = re.search(r'_SANDBOX_RESTRICTED_SYMBOLS.*', txt2)
print('VERIFICACION:', m2.group() if m2 else 'NO ENCONTRADO')
print('DONE')
