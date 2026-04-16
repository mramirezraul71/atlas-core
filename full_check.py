
import requests, json
BASE = 'http://127.0.0.1:8795'
H = {'x-api-key': 'atlas-quant-local'}
h = requests.get(BASE + '/health', headers=H, timeout=5)
print(json.dumps(h.json(), indent=2))

# Loop status
l = requests.get(BASE + '/operation/loop/status', headers=H, timeout=5)
d = l.json().get('data', {})
print('Loop running:', d.get('running'), '| cycles:', d.get('cycle_count'))
print('Started:', d.get('started_at'), '| Stopped:', d.get('stopped_at'))

# OS in journal
import sqlite3
conn = sqlite3.connect(r'C:\ATLAS_PUSH\atlas_code_quant\data\journal\trading_journal.sqlite3')
c = conn.cursor()
c.execute("SELECT symbol, status, entry_time FROM trading_journal WHERE symbol='OS' AND status='open'")
rows = c.fetchall()
print('OS open entries:', rows)
conn.close()
