
import subprocess, os, sys

env = os.environ.copy()
try:
    with open(r'C:\dev\credenciales.txt', 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if '=' in line and not line.startswith('#') and not line.startswith('['):
                k, v = line.split('=', 1)
                env[k.strip()] = v.strip()
except: pass
env['ATLAS_SANDBOX_RESTRICTED_SYMBOLS'] = 'OS'
env['QUANT_EXIT_GOVERNANCE_ENABLED'] = 'false'

PYTHON = r'C:\ATLAS_PUSH\venv\Scripts\python.exe'
CWD = r'C:\ATLAS_PUSH'

# Run synchronously (short timeout) to capture the error
r = subprocess.run(
    [PYTHON, '-m', 'uvicorn', 'atlas_code_quant.api.main:app',
     '--host', '0.0.0.0', '--port', '8795', '--log-level', 'debug'],
    cwd=CWD, env=env,
    capture_output=True, text=True, errors='ignore',
    timeout=15  # Will fail or succeed in 15s
)
print('RC:', r.returncode)
print('STDOUT:', r.stdout[:1000])
print('STDERR:', r.stderr[:2000])
