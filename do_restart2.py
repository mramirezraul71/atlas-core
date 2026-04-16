import subprocess, time, os, sys

# Step 1: Read credentials
cred_path = r'C:\dev\credenciales.txt'
env = os.environ.copy()
try:
    with open(cred_path, 'r') as f:
        for line in f:
            line = line.strip()
            if '=' in line and not line.startswith('#'):
                k, v = line.split('=', 1)
                env[k.strip()] = v.strip()
    print('Credenciales cargadas OK')
except Exception as e:
    print(f'Error leyendo credenciales: {e}')

# Step 2: Kill process on port 8795
try:
    r = subprocess.run(['netstat','-ano'],capture_output=True,text=True)
    pid = None
    for line in r.stdout.splitlines():
        if ':8795' in line and 'LISTENING' in line:
            parts = line.split()
            pid = parts[-1]
            break
    if pid:
        subprocess.run(['taskkill','/F','/PID',pid], capture_output=True)
        print(f'Killed PID {pid} on :8795')
        time.sleep(3)
    else:
        print('No process found on :8795')
except Exception as e:
    print(f'Error killing: {e}')

# Step 3: Launch ATLAS
log = open(r'C:\ATLAS_PUSH\atlas_restart.log', 'w')
try:
    p = subprocess.Popen(
        [r'C:\ATLAS_PUSH\venv\Scripts\python.exe', r'C:\ATLAS_PUSH\immediate_start.py'],
        cwd=r'C:\ATLAS_PUSH',
        env=env,
        stdout=log, stderr=log,
        creationflags=0x00000008  # DETACHED_PROCESS
    )
    print(f'ATLAS launched PID={p.pid}')
except Exception as e:
    print(f'Launch error: {e}')
finally:
    log.close()

print('Restart script complete')
