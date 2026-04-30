
import subprocess, sys, time, os

ROOT = r"C:\ATLAS_PUSH"

# Encontrar todos los python que corren atlas_mcp_server
print("=== Buscando procesos python con atlas_mcp ===")
try:
    r = subprocess.run(
        'wmic process where "name=\'python.exe\'" get processid,commandline /format:list',
        shell=True, capture_output=True, text=True, timeout=15
    )
    lines = r.stdout.splitlines()
    current_pid = os.getpid()
    to_kill = []
    i = 0
    cmd = ""
    pid = ""
    for line in lines:
        if line.startswith("CommandLine="):
            cmd = line
        elif line.startswith("ProcessId="):
            pid = line.split("=")[1].strip()
            if "atlas_mcp_server" in cmd and pid.isdigit() and int(pid) != current_pid:
                print(f"  Found: PID={pid} CMD={cmd[12:80]}")
                to_kill.append(pid)
            cmd = ""
            pid = ""
    
    for pid in to_kill:
        r2 = subprocess.run(['taskkill','/F','/PID',pid], capture_output=True, text=True)
        print(f"  Killed PID {pid}: {r2.stdout.strip()}")
    
    if not to_kill:
        print("  No atlas_mcp_server processes found via wmic")
        
except Exception as e:
    print(f"wmic error: {e}")

time.sleep(2)

# Relanzar
mcp = os.path.join(ROOT, 'atlas_mcp_server.py')
py = sys.executable
log = open(os.path.join(ROOT,'mcp_restart.log'), 'a')
proc = subprocess.Popen([py, mcp], cwd=ROOT, stdout=log, stderr=log)
time.sleep(5)

import socket
try:
    s = socket.create_connection(('127.0.0.1', 8799), timeout=4)
    s.close()
    print(f"MCP ONLINE — PID={proc.pid}")
except Exception as e:
    print(f"Socket: {e} — PID={proc.pid} (puede seguir iniciando)")

print("DONE")
