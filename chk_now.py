
import requests
BASE = 'http://127.0.0.1:8795'
H = {'x-api-key': 'atlas-quant-local'}
r = requests.get(BASE + '/operation/loop/status', headers=H, timeout=5)
import json
data = r.json()
loop = data.get('data', {})
print('loop running:', loop.get('running'))
print('loop cycles:', loop.get('cycle_count'))
print('loop stopped_at:', loop.get('stopped_at'))
