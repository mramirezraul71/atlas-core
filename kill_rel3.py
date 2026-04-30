import subprocess, time

# Matar PID actual 41716
print("[1] Matando PID 41716...")
r = subprocess.run(["taskkill", "/F", "/PID", "41716"],
    stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
print("kill:", r.stdout.decode(errors="replace").strip())
time.sleep(4)

# Lanzar nuevo uvicorn - SIN exit_governance ya que el VaR ahora se resolverá con el patch
print("[2] Lanzando uvicorn con patch journal_sync activo...")
p = subprocess.Popen(
    ["C:\\ATLAS_PUSH\\venv\\Scripts\\python.exe",
     "-m", "uvicorn",
     "atlas_code_quant.api.main:app",
     "--host", "0.0.0.0", "--port", "8795", "--log-level", "info"],
    cwd="C:\\ATLAS_PUSH",
    stdout=open("C:\\ATLAS_PUSH\\api_out3.log", "w"),
    stderr=open("C:\\ATLAS_PUSH\\api_err3.log", "w"),
    creationflags=0x00000008
)
print("Launched PID:", p.pid)
time.sleep(2)
print("Done.")

