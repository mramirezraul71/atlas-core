"""
ATLAS Controlled Restart Script
Mata el proceso principal y lo relanza con las nuevas credenciales
"""
import subprocess
import os
import time
import signal
import sys
import requests

ATLAS_PID = 39932
ATLAS_URL = "http://127.0.0.1:8795"
API_KEY = "atlas-quant-local"
PYTHON = r"C:\ATLAS_PUSH\venv\Scripts\python.exe"
LAUNCHER = r"C:\ATLAS_PUSH\atlas_code_quant\tools\service_launcher.py"
CWD = r"C:\ATLAS_PUSH"
LOG = r"C:\ATLAS_PUSH\restart_log.txt"

def check_process_alive(pid):
    try:
        result = subprocess.run(['tasklist', '/FI', f'PID eq {pid}', '/NH', '/FO', 'CSV'],
                               capture_output=True, text=True)
        return str(pid) in result.stdout
    except:
        return False

def wait_for_api(timeout=90):
    start = time.time()
    while time.time() - start < timeout:
        try:
            r = requests.get(f"{ATLAS_URL}/health", timeout=3)
            if r.status_code == 200:
                return True
        except:
            pass
        time.sleep(3)
    return False

print(f"=== ATLAS Restart ===")
print(f"PID actual: {ATLAS_PID}, alive: {check_process_alive(ATLAS_PID)}")

# 1. Matar proceso principal
if check_process_alive(ATLAS_PID):
    print(f"Matando PID {ATLAS_PID}...")
    subprocess.run(['taskkill', '/F', '/PID', str(ATLAS_PID)], capture_output=True)
    time.sleep(3)
    if check_process_alive(ATLAS_PID):
        print("ADVERTENCIA: Proceso sigue vivo, forzando...")
        subprocess.run(['taskkill', '/F', '/T', '/PID', str(ATLAS_PID)], capture_output=True)
        time.sleep(3)
    print(f"Proceso {ATLAS_PID} terminado: {not check_process_alive(ATLAS_PID)}")
else:
    print(f"PID {ATLAS_PID} ya no existe")

# 2. Esperar un momento
time.sleep(2)

# 3. Verificar que el launcher existe
if not os.path.exists(LAUNCHER):
    # Intentar con immediate_start.py
    LAUNCHER = r"C:\ATLAS_PUSH\immediate_start.py"
    print(f"Usando launcher alternativo: {LAUNCHER}")

print(f"Lanzando ATLAS desde {LAUNCHER}...")

# 4. Cargar credenciales
cred_path = r"C:\dev\credenciales.txt"
env = dict(os.environ)
try:
    with open(cred_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if '=' in line and not line.startswith('#'):
                k, _, v = line.partition('=')
                if k.strip():
                    env[k.strip()] = v.strip()
    print("Credenciales cargadas")
    print(f"EXIT_GOVERNANCE: {env.get('QUANT_EXIT_GOVERNANCE_ENABLED', 'NOT SET')}")
except Exception as e:
    print(f"Error cargando credenciales: {e}")

# 5. Lanzar en background
try:
    with open(LOG, 'w') as log:
        proc = subprocess.Popen(
            [PYTHON, LAUNCHER],
            cwd=CWD,
            env=env,
            stdout=log,
            stderr=log,
            creationflags=0x00000008  # DETACHED_PROCESS
        )
    print(f"ATLAS relanzado. PID nuevo: {proc.pid}")
except Exception as e:
    print(f"Error lanzando ATLAS: {e}")
    sys.exit(1)

# 6. Esperar a que arranque
print("Esperando API...")
time.sleep(10)

# 7. Verificar
for attempt in range(1, 6):
    try:
        r = requests.get(f"{ATLAS_URL}/health", timeout=5)
        print(f"[{attempt}] API responde: {r.status_code}")
        if r.status_code in [200, 422]:
            print("ATLAS UP!")
            break
    except Exception as e:
        print(f"[{attempt}] Esperando... {e}")
    time.sleep(8)

print("=== Restart completado ===")
