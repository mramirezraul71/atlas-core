
import subprocess, time, os, sys

print('=== ATLAS RESTART v5 ===')

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

# Step 2: Kill PID 39932 directly
pid_to_kill = '39932'
try:
    kr = subprocess.run(['taskkill', '/F', '/PID', pid_to_kill], 
                        capture_output=True, text=True, errors='ignore')
    print('Kill PID 39932:', kr.stdout.strip(), kr.stderr.strip())
    time.sleep(4)
except Exception as e:
    print('Kill error:', e)

# Verify port is free
try:
    r = subprocess.run('netstat -ano | findstr ":8795"', shell=True, 
                       capture_output=True, text=True, errors='ignore', timeout=5)
    print('Port check after kill:', repr(r.stdout[:300]))
except:
    pass

# Step 3: Launch ATLAS — plain Popen, no special flags
PYTHON = r'C:\ATLAS_PUSH\venv\Scripts\python.exe'
LAUNCHER = r'C:\ATLAS_PUSH\immediate_start.py'
CWD = r'C:\ATLAS_PUSH'

try:
    p = subprocess.Popen(
        [PYTHON, LAUNCHER],
        cwd=CWD,
        env=env,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    print('ATLAS launched PID=' + str(p.pid))
    with open(r'C:\ATLAS_PUSH\atlas_restart.log', 'w') as f:
        f.write('Restart at ' + time.ctime() + ', new PID=' + str(p.pid) + '\n')
except Exception as e:
    print('Launch error:', e)
    # Fallback: try with shell
    try:
        r2 = subprocess.Popen(
            PYTHON + ' ' + LAUNCHER,
            cwd=CWD,
            env=env,
            shell=True,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        print('Shell launch PID=' + str(r2.pid))
    except Exception as e2:
        print('Shell launch error:', e2)

print('Done')
