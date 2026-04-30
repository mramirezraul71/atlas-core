import subprocess, time

# Matar PID 30360 directamente
print("[1] Matando PID 30360 (uvicorn viejo)...")
r = subprocess.run(
    ["taskkill", "/F", "/PID", "30360"],
    timeout=10, stdout=subprocess.PIPE, stderr=subprocess.PIPE
)
print("taskkill:", r.stdout.decode(errors="replace").strip(), r.stderr.decode(errors="replace").strip())
time.sleep(3)

# Lanzar uvicorn con Start-Process sin capturar output (evita timeout)
print("[2] Lanzando uvicorn nuevo...")
ps = (
    r:QUANT_EXIT_GOVERNANCE_ENABLED="false";
    r:TRADIER_PAPER_TOKEN="UqYAFhBY0sPWSP4OmuAHChHB0lAN";
    rStart-Process -FilePath "C:\ATLAS_PUSH\venv\Scripts\python.exe" 
    r-ArgumentList @("-m","uvicorn","atlas_code_quant.api.main:app","--host","0.0.0.0","--port","8795","--log-level","info") 
    r-WorkingDirectory "C:\ATLAS_PUSH" 
    r-RedirectStandardOutput "C:\ATLAS_PUSH\api_stdout.log" 
    r-RedirectStandardError "C:\ATLAS_PUSH\api_stderr.log" -NoNewWindow
)

# Usar cmd /c start para no bloquear
r2 = subprocess.Popen(
    ["powershell.exe", "-NonInteractive", "-Command",
     "$env:QUANT_EXIT_GOVERNANCE_ENABLED=\"false\"; $env:TRADIER_PAPER_TOKEN=\"UqYAFhBY0sPWSP4OmuAHChHB0lAN\"; Start-Process -FilePath \"C:\\ATLAS_PUSH\\venv\\Scripts\\python.exe\" -ArgumentList @(\"-m\",\"uvicorn\",\"atlas_code_quant.api.main:app\",\"--host\",\"0.0.0.0\",\"--port\",\"8795\",\"--log-level\",\"info\") -WorkingDirectory \"C:\\ATLAS_PUSH\" -RedirectStandardOutput \"C:\\ATLAS_PUSH\\api_stdout2.log\" -RedirectStandardError \"C:\\ATLAS_PUSH\\api_stderr2.log\" -NoNewWindow"]
)
print("Lanzado PID PS:", r2.pid)
time.sleep(2)
print("Done - uvicorn deberia estar arrancando")

