import subprocess
import sys
import os

script_path = r"C:\ATLAS_PUSH\os_blocker_v2.py"
python_exe = r"C:\ATLAS_PUSH\venv\Scripts\python.exe"
log_path = r"C:\ATLAS_PUSH\os_blocker_v2.log"

# Iniciar en background como proceso independiente
with open(log_path, 'w') as log:
    proc = subprocess.Popen(
        [python_exe, script_path],
        stdout=log,
        stderr=log,
        creationflags=0x00000008  # DETACHED_PROCESS en Windows
    )
    print(f"OS Blocker iniciado en background. PID: {proc.pid}")
    print(f"Log: {log_path}")
