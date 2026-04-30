import subprocess, time, os, sys

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
    print('Credenciales cargadas OK')
    print('EXIT_GOVERNANCE:', env.get('QUANT_EXIT_GOVERNANCE_ENABLED', 'NO ENCONTRADO'))
except Exception as e:
    print(f'Error leyendo credenciales: {e}')

# Step 2: Kill process on port 8795 using wmic
try:
    # Find PID via wmic
    r = subprocess.run(
        ['wmic','process','where','(CommandLine like "%%8795%%" or CommandLine like "%%immediate_start%%")','get','ProcessId,CommandLine'],
        capture_output=True, text=True, errors='ignore'
    )
    print('WMIC output:', r.stdout[:500])
    # Also try netstat with shell
    r2 = subprocess.run('netstat -ano | findstr :8795', shell=True, capture_output=True, text=True, errors='ignore')
    print('Netstat:', r2.stdout[:300])
    pid = None
    for line in r2.stdout.splitlines():
        if ':8795' in line and 'LISTENING' in line:
            parts = line.split()
            pid = parts[-1]
            break
    if pid and pid.isdigit():
        kill = subprocess.run(['taskkill','/F','/PID',pid], capture_output=True, text=True, errors='ignore')
        print(f'Killed PID {pid}: {kill.stdout.strip()}')
        time.sleep(4)
    else:
        print('No LISTENING PID found on :8795 — trying by name')
        kill2 = subprocess.run(['taskkill','/F','/IM','python.exe','/FI','WINDOWTITLE eq immediate_start*'], capture_output=True, text=True, errors='ignore')
        print(f'Kill by name: {kill2.stdout.strip()}')
        time.sleep(2)
except Exception as e:
    print(f'Error killing: {e}')

# Step 3: Launch ATLAS detached (no stdout redirect to avoid WinError 87)
try:
    DETACHED = 0x00000008
    CREATE_NEW_PROCESS_GROUP = 0x00000200
    p = subprocess.Popen(
        [r'C:\ATLAS_PUSH\venv\Scripts\python.exe', r'C:\ATLAS_PUSH\immediate_start.py'],
        cwd=r'C:\ATLAS_PUSH',
        env=env,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        creationflags=DETACHED | CREATE_NEW_PROCESS_GROUP
    )
    print(f'ATLAS launched PID={p.pid}')
except Exception as e:
    print(f'Launch error: {e}')

# Write simple log
try:
    with open(r'C:\ATLAS_PUSH\atlas_restart.log', 'w') as f:
        f.write(f'Restart ejecutado a {time.ctime()}\n')
except Exception as e:
    print(f'Log error: {e}')

print('Restart script complete')
