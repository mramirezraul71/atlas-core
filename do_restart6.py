
import subprocess, time, os, requests

print('=== ATLAS RESTART v6 ===')

# Step 1: Credentials
cred_path = r'C:\dev\credenciales.txt'
env = os.environ.copy()
try:
    with open(cred_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if '=' in line and not line.startswith('#'):
                k, v = line.split('=', 1)
                env[k.strip()] = v.strip()
    print('Creds OK, EXIT_GOVERNANCE=' + env.get('QUANT_EXIT_GOVERNANCE_ENABLED', '?'))
except Exception as e:
    print('Cred error:', e)

PYTHON = r'C:\ATLAS_PUSH\venv\Scripts\python.exe'
LAUNCHER = r'C:\ATLAS_PUSH\immediate_start.py'
CWD = r'C:\ATLAS_PUSH'

# Step 2: Try graceful shutdown via API
try:
    resp = requests.post('http://127.0.0.1:8795/shutdown', 
                        headers={'X-ATLAS-Key': 'atlas-quant-local'},
                        timeout=5)
    print('Shutdown response:', resp.status_code, resp.text[:200])
    time.sleep(6)
except Exception as e:
    print('Shutdown API error (ok if not available):', e)

# Check port now
r = subprocess.run('netstat -ano | findstr ":8795"', shell=True, 
                   capture_output=True, text=True, errors='ignore', timeout=5)
print('Port after shutdown:', repr(r.stdout[:200]))

# If still up, try taskkill with /T (tree kill)
for line in r.stdout.splitlines():
    if ':8795' in line and 'LISTENING' in line:
        parts = line.split()
        pid = parts[-1] if parts else None
        if pid and pid.isdigit():
            kr = subprocess.run(['taskkill', '/F', '/T', '/PID', pid], 
                                capture_output=True, text=True, errors='ignore')
            print('Taskkill /T:', kr.stdout.strip(), kr.stderr[:100].strip())
            time.sleep(3)
        break

# Step 3: Launch without any stdout redirect (let MCP handle it)
try:
    p = subprocess.Popen(
        [PYTHON, LAUNCHER],
        cwd=CWD,
        env=env,
    )
    print('ATLAS launched PID=' + str(p.pid))
    with open(r'C:\ATLAS_PUSH\atlas_restart.log', 'w') as f:
        f.write('Restart at ' + time.ctime() + ', new PID=' + str(p.pid) + '\n')
except Exception as e:
    print('Launch error:', e)

print('Done')
