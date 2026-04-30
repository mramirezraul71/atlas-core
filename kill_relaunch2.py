import subprocess, time

# Matar PID 30360 directamente con taskkill
print("[1] Matando PID 30360...")
r = subprocess.run(["taskkill", "/F", "/PID", "30360"],
    stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
print("kill:", r.stdout.decode(errors="replace").strip())
time.sleep(4)

# Lanzar uvicorn sin capturar stdout/stderr para no bloquear
print("[2] Lanzando uvicorn nuevo con exit_governance=false...")
env_setup = """
import os, subprocess
os.environ["QUANT_EXIT_GOVERNANCE_ENABLED"] = "false"
os.environ["TRADIER_PAPER_TOKEN"] = "UqYAFhBY0sPWSP4OmuAHChHB0lAN"
import sys
p = subprocess.Popen(
    [sys.executable, "-m", "uvicorn",
     "atlas_code_quant.api.main:app",
     "--host", "0.0.0.0", "--port", "8795", "--log-level", "info"],
    cwd=r"C:\ATLAS_PUSH",
    stdout=open(r"C:\ATLAS_PUSH\api_stdout2.log", "w"),
    stderr=open(r"C:\ATLAS_PUSH\api_stderr2.log", "w"),
    env={**os.environ},
    creationflags=0x00000008
)
print("Launched PID:", p.pid)
"""
# Escribir sub-launcher
with open(r"C:\ATLAS_PUSH\sub_launch.py", "w") as f:
    f.write(env_setup)

# Ejecutar sub-launcher con Popen (no bloquea)
p2 = subprocess.Popen(
    [r"C:\ATLAS_PUSH\venv\Scripts\python.exe",
     r"C:\ATLAS_PUSH\sub_launch.py"],
    stdout=subprocess.PIPE, stderr=subprocess.PIPE
)
time.sleep(5)
out, err = p2.communicate(timeout=10)
print("sub_launch stdout:", out.decode(errors="replace").strip())
print("sub_launch stderr:", err.decode(errors="replace").strip())
print("Done.")
