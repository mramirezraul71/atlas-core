
import subprocess, os, time

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
LOG = r'C:\ATLAS_PUSH\api_launch_test.log'

# Run via shell with redirect to capture error
cmd = (
    f'"{PYTHON}" -m uvicorn atlas_code_quant.api.main:app '
    f'--host 0.0.0.0 --port 8795 --log-level info '
    f'> "{LOG}" 2>&1'
)

# Use Popen with shell=True so we don't need capture_output
p = subprocess.Popen(cmd, shell=True, env=env, cwd=CWD)
print(f'Started PID={p.pid}')
time.sleep(10)

# Check if still running
poll = p.poll()
print(f'Process poll: {poll}')  # None=still running, number=exit code

# Read log
try:
    with open(LOG, 'r', encoding='utf-8', errors='ignore') as f:
        print('Log output:')
        print(f.read()[:2000])
except Exception as e:
    print('Log error:', e)

# Port check
r2 = subprocess.run('netstat -ano | findstr ":8795"', shell=True,
                    capture_output=True, text=True, errors='ignore', timeout=5)
print('Port:', repr(r2.stdout[:300]))
