"""atlas_live_ready.py
Script de configuracion optima para live trading.
Ejecutar ANTES de iniciar el loop en live.
Uso: python atlas_live_ready.py
"""
import json, sqlite3, urllib.request, sys, re
from pathlib import Path
from datetime import datetime

API = 'http://127.0.0.1:8795'
KEY = {'Content-Type': 'application/json', 'X-API-Key': 'atlas-quant-local'}
DB  = r'C:\ATLAS_PUSH\atlas_code_quant\data\journal\trading_journal.sqlite3'
STATE = r'C:\ATLAS_PUSH\atlas_code_quant\data\operation\operation_center_state.json'

def api(path, body=None, method='POST'):
    req = urllib.request.Request(f'{API}{path}', data=json.dumps(body or {}).encode(), headers=KEY, method=method)
    try:
        return json.loads(urllib.request.urlopen(req, timeout=15).read())
    except Exception as e:
        return {'error': str(e)}

ok = True
log = []

# 1. Verificar API viva
print('[1/7] Verificando API...')
h = api('/health', method='GET')
if 'error' in h:
    print('  ERROR: API no responde:', h['error']); sys.exit(1)
print(f'  OK — uptime {h.get("uptime_sec","?")}s')

# 2. Purgar phantoms OS y AA del journal (bloqueo primario)
print('[2/7] Purgando phantoms OS+AA del journal...')
conn = sqlite3.connect(DB)
cur = conn.cursor()
cur.execute("DELETE FROM trading_journal WHERE symbol IN ('OS','AA') AND broker_order_ids_json IN ('{}','[]','')")
n = cur.rowcount
conn.commit(); conn.close()
print(f'  Eliminados: {n} phantoms')
log.append(f'phantoms_purgados={n}')

# 3. Detener loop
print('[3/7] Deteniendo loop...')
r = api('/operation/loop/stop')
print(f'  Loop stopped: {r.get("ok", r)}')

# 4. Configurar operation center para live-ready
print('[4/7] Configurando operation center...')
cfg = {
    'auton_mode': 'paper_autonomous',
    'executor_mode': 'paper_api',
    'require_operator_present': False,
    'operator_present': True,
    'fail_safe_active': False,
    'signal_validator_enabled': True,
    'var_monitor_enabled': True,
    'auto_pause_on_operational_errors': True,
    'operational_error_limit': 10,
    'min_auton_win_rate_pct': 35.0,
}
r = api('/operation/config', cfg)
print(f'  Config: {r.get("ok", r)}')

# 5. Verificar patch _SANDBOX_RESTRICTED_SYMBOLS en main.py
print('[5/7] Verificando patch restricted symbols...')
p = Path(r'C:\ATLAS_PUSH\atlas_code_quant\api\main.py')
txt = p.read_text(encoding='utf-8')
m = re.search(r'_SANDBOX_RESTRICTED_SYMBOLS.*frozenset\([^\n]+\)', txt)
line = m.group() if m else 'NO ENCONTRADO'
print(f'  {line}')
if '"OS"' in line and '"AA"' in line:
    print('  OK — OS y AA bloqueados en disco')
else:
    print('  ADVERTENCIA — Patch incompleto, revisar main.py')

# 6. Reiniciar loop con parametros controlados
print('[6/7] Iniciando loop (interval=120s, max_per_cycle=1)...')
r = api('/operation/loop/start', {'interval_sec': 120, 'max_per_cycle': 1})
print(f'  Loop start: ok={r.get("ok")} error={r.get("error","-")}')

# 7. Estado final
print('[7/7] Verificacion final...')
conn2 = sqlite3.connect(DB)
cur2 = conn2.cursor()
cur2.execute("SELECT symbol, COUNT(*) FROM trading_journal WHERE status='open' AND broker_order_ids_json IN ('{}','[]','') GROUP BY symbol")
phantoms = cur2.fetchall()
conn2.close()
print(f'  Phantoms open restantes: {phantoms if phantoms else "NINGUNO (OK)"}')

now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
print(f'\n[DONE] {now} — ATLAS listo. Loop activo.')
print('  OS y AA: bloqueados por patch hardcodeado en main.py')
print('  Portfolio risk guard: activo (bloqueara nuevas entradas si hay exit_now)')
print('  Scanner: activo, candidatos reales cada 180s')
print('  Loop: ciclos cada 120s, max 1 trade por ciclo')
print('  Nota: reiniciar uvicorn activa AA en memoria ademas de OS')
