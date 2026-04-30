
import requests
BASE = 'http://127.0.0.1:8795'
H = {'x-api-key': 'atlas-quant-local'}
r = requests.get(BASE + '/operation/loop/status', headers=H, timeout=5)
d = r.json().get('data', {})
print('Loop running:', d.get('running'), '| cycles:', d.get('cycle_count'), '| stopped_at:', d.get('stopped_at'))
h = requests.get(BASE + '/health', headers=H, timeout=5)
print('Uptime:', h.json().get('uptime_sec'), 'sec')
