
import subprocess, time, sys, os

print('Matando PID 24420...')
r = subprocess.run(['taskkill', '/F', '/PID', '24420'], capture_output=True, text=True)
print('taskkill stdout:', r.stdout)
print('taskkill stderr:', r.stderr)
time.sleep(4)

print('Relanzando uvicorn con codigo parcheado...')
env = {**os.environ}
env['QUANT_EXIT_GOVERNANCE_ENABLED'] = 'false'
env['QUANT_XGBOOST_ENABLED'] = 'false'

import pathlib
pathlib.Path(r'C:\ATLAS_PUSH\logs').mkdir(exist_ok=True)

proc = subprocess.Popen(
    [
        r'C:\ATLAS_PUSH\venv\Scripts\python.exe', '-m', 'uvicorn',
        'api.main:app',
        '--host', '0.0.0.0',
        '--port', '8795',
        '--log-level', 'warning',
    ],
    cwd=r'C:\ATLAS_PUSH\atlas_code_quant',
    env=env,
    stdout=open(r'C:\ATLAS_PUSH\logs\uv_out.log', 'w'),
    stderr=open(r'C:\ATLAS_PUSH\logs\uv_err.log', 'w'),
    creationflags=0x00000008,
)
print(f'Nuevo PID: {proc.pid}')
