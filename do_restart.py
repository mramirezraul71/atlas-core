import subprocess, os, time, requests, sys

# Identificar PID del proceso ATLAS principal (el más grande en RAM en puerto 8795)
def get_atlas_pid():
    import socket
    # El proceso en el puerto 8795 es el API principal
    result = subprocess.run(
        ['netstat', '-ano'],
        capture_output=True, text=True
    )
    for line in result.stdout.split('\n'):
        if ':8795 ' in line and 'LISTENING' in line:
            parts = line.strip().split()
            if parts:
                return int(parts[-1])
    return None

pid = get_atlas_pid()
print(f"PID ATLAS en :8795 = {pid}")

# Matar proceso
if pid:
    subprocess.run(['taskkill', '/F', '/PID', str(pid)], capture_output=True)
    time.sleep(2)
    # Verificar muerto
    result = subprocess.run(['tasklist', '/FI', f'PID eq {pid}', '/NH', '/FO', 'CSV'],
                           capture_output=True, text=True)
    alive = str(pid) in result.stdout
    print(f"Proceso {pid} muerto: {not alive}")
else:
    print("No se encontró PID en :8795")

time.sleep(2)

# Cargar credenciales
env = dict(os.environ)
try:
    with open(r"C:\dev\credenciales.txt", 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if '=' in line and not line.startswith('#') and not line.startswith(' '):
                k, _, v = line.partition('=')
                if k.strip() and not any(c in k for c in [' ', '\t']):
                    env[k.strip()] = v.strip()
    print(f"Credenciales cargadas. EXIT_GOVERNANCE={env.get('QUANT_EXIT_GOVERNANCE_ENABLED','?')}")
    print(f"SANDBOX_RESTRICTED={env.get('ATLAS_SANDBOX_RESTRICTED_SYMBOLS','OS (default)')}")
except Exception as e:
    print(f"Credenciales error: {e}")

# Lanzar ATLAS
PYTHON = r"C:\ATLAS_PUSH\venv\Scripts\python.exe"
LAUNCHER = r"C:\ATLAS_PUSH\immediate_start.py"
CWD = r"C:\ATLAS_PUSH"
LOG = r"C:\ATLAS_PUSH\atlas_restart.log"

print(f"Lanzando: {PYTHON} {LAUNCHER}")
with open(LOG, 'w') as log:
    proc = subprocess.Popen(
        [PYTHON, LAUNCHER],
        cwd=CWD, env=env,
        stdout=log, stderr=log,
        creationflags=0x00000008
    )
print(f"PID nuevo: {proc.pid}")

# Esperar arranque
print("Esperando que la API levante en :8795...")
for i in range(1, 16):
    time.sleep(6)
    try:
        r = requests.get("http://127.0.0.1:8795/health", timeout=4,
                        headers={"X-API-Key": "atlas-quant-local"})
        if r.status_code == 200:
            data = r.json()
            print(f"[{i}] API UP: uptime={data.get('uptime_sec'):.1f}s, open_positions={data.get('open_positions')}")
            break
    except Exception as e:
        print(f"[{i}] Esperando... ({e.__class__.__name__})")

print("Restart completo.")
