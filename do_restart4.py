import subprocess, time, os

# Step 1: Read credentials
cred_path = r'C:\dev\credenciales.txt'
env = os.environ.copy()
try:
    with open(cred_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if '=' in line and not line.startswith('#'):
                k, v = line.split('=', 1)
                env[k.strip()] = v.strip()
    print('Credenciales OK, EXIT_GOVERNANCE=' + env.get('QUANT_EXIT_GOVERNANCE_ENABLED','?'))
except Exception as e:
    print('Cred error:', e)

# Step 2: Kill port 8795
try:
    r = subprocess.run('netstat -ano | findstr ":8795"', shell=True, capture_output=True, text=True, errors='ignore', timeout=10)
    print('Netstat:', repr(r.stdout[:300]))
    pid = None
    for line in r.stdout.splitlines():
        if ':8795' in line and 'LISTENING' in line:
            parts = line.split()
            pid = parts[-1] if parts else None
            break
    if pid and pid.isdigit():
        kr = subprocess.run('taskkill /F /PID ' + pid, shell=True, capture_output=True, text=True, errors='ignore')
        print('Kill:', kr.stdout.strip())
        time.sleep(4)
    else:
        print('Port 8795 not LISTENING or already down')
except Exception as e:
    print('Kill error:', e)

# Step 3: Launch via start /B
try:
    cmd = r'start /B C:\ATLAS_PUSH\venv\Scripts\python.exe C:\ATLAS_PUSH\immediate_start.py'
    r2 = subprocess.run(cmd, shell=True, env=env, capture_output=True, text=True, errors='ignore', timeout=8)
    print('Launch rc:', r2.returncode)
    print('Launch stdout:', r2.stdout[:200])
    print('Launch stderr:', r2.stderr[:200])
except subprocess.TimeoutExpired:
    print('Launch timeout — process started OK (expected)')
except Exception as e:
    print('Launch error:', e)

# Log
try:
    with open(r'C:\ATLAS_PUSH\atlas_restart.log', 'w') as f:
        f.write('Restart at ' + time.ctime() + '\n')
except:
    pass

print('Done')
