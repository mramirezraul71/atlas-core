import os
import time
import traceback
import sys
sys.path.insert(0, r'c:\ATLAS_PUSH')
sys.path.insert(0, r'c:\ATLAS_PUSH\atlas_code_quant')
from atlas_code_quant.production.grafana_dashboard import GrafanaDashboard

os.environ.setdefault('PYTHONUNBUFFERED', '1')
gd = GrafanaDashboard()
print('START_METRICS', gd.start_metrics_server(9100), flush=True)
while True:
    try:
        snap = gd.sync_from_canonical(account_scope='paper')
        print('SYNC_OK', bool(snap), flush=True)
    except Exception:
        traceback.print_exc()
    time.sleep(2)
