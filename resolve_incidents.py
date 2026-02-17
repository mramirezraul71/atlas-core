"""Resolve all open incidents in memory."""
import sys
sys.path.insert(0, '.')
from modules.humanoid.ans.incident import get_incidents, resolve_all_open

open_inc = get_incidents(status='open')
print(f'Incidentes abiertos en memoria: {len(open_inc)}')
for i in open_inc[:5]:
    sev = i.get('severity', '')
    cid = i.get('check_id', '')[:30]
    msg = i.get('message', '')[:30]
    print(f'  [{sev:4}] {cid:30} {msg}')

if open_inc:
    n = resolve_all_open()
    print(f'Resueltos: {n}')
else:
    print('No hay incidentes abiertos')
