import subprocess, time, sys

# Paso 1: Matar proceso en 8795
print("[1] Matando uvicorn en :8795...")
kill_cmd = """
$conn = Get-NetTCPConnection -LocalPort 8795 -ErrorAction SilentlyContinue | Select-Object -First 1
if ($conn) {
    $pid = $conn.OwningProcess
    Write-Output "Killing PID $pid"
    Stop-Process -Id $pid -Force -ErrorAction SilentlyContinue
    Start-Sleep -Seconds 3
} else {
    Write-Output "No process on :8795"
}
"""

r = subprocess.run(
    ["powershell.exe", "-NonInteractive", "-Command", kill_cmd.strip()],
    timeout=15, stdout=subprocess.PIPE, stderr=subprocess.PIPE
)
print("Kill stdout:", r.stdout.decode(errors="replace").strip())
print("Kill stderr:", r.stderr.decode(errors="replace").strip())

# Paso 2: Lanzar uvicorn nuevo con QUANT_EXIT_GOVERNANCE_ENABLED=false
print("[2] Lanzando uvicorn con exit_governance=false + AA restringida...")
launch_cmd = r"""
$env:QUANT_EXIT_GOVERNANCE_ENABLED = "false"
$env:TRADIER_PAPER_TOKEN = "UqYAFhBY0sPWSP4OmuAHChHB0lAN"
$p = Start-Process -FilePath "C:\ATLAS_PUSH\venv\Scripts\python.exe" `
    -ArgumentList @("-m","uvicorn","atlas_code_quant.api.main:app","--host","0.0.0.0","--port","8795","--log-level","info") `
    -WorkingDirectory "C:\ATLAS_PUSH" `
    -RedirectStandardOutput "C:\ATLAS_PUSH\api_stdout.log" `
    -RedirectStandardError "C:\ATLAS_PUSH\api_stderr.log" `
    -NoNewWindow -PassThru
Write-Output "Launched PID: $($p.Id)"
"""

r2 = subprocess.run(
    ["powershell.exe", "-NonInteractive", "-Command", launch_cmd.strip()],
    timeout=20, stdout=subprocess.PIPE, stderr=subprocess.PIPE
)
print("Launch stdout:", r2.stdout.decode(errors="replace").strip())
print("Launch stderr:", r2.stderr.decode(errors="replace").strip())
print("Done.")

