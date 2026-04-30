import subprocess
import os

python_exe = r"C:\ATLAS_PUSH\venv\Scripts\python.exe"
script = r"C:\ATLAS_PUSH\trade_monitor.py"
log = r"C:\ATLAS_PUSH\trade_monitor.log"

with open(log, 'w') as f:
    proc = subprocess.Popen(
        [python_exe, script],
        stdout=f, stderr=f,
        creationflags=0x00000008
    )
print(f"Trade monitor iniciado. PID: {proc.pid}")
print(f"Log: {log}")
