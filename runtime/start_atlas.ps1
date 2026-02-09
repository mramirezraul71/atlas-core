$ErrorActionPreference="Stop"

# === CONFIG ===
$BASE="C:\ATLAS"
$BRIDGE_DIR="$BASE\bridge"
$PORT=8787

# Token del Bridge (el mismo que usas en Authorization: Bearer ...)
# RECOMENDADO: cámbialo por uno fuerte después.
$env:ATLAS_TOKEN="raul_atlas_token_123"

# === START: ATLAS BRIDGE ===
Start-Process -WindowStyle Minimized -FilePath "powershell.exe" -ArgumentList @(
  "-NoProfile","-ExecutionPolicy","Bypass",
  "-Command", "cd `"$BRIDGE_DIR`"; python -m uvicorn server:app --host 127.0.0.1 --port $PORT"
)

Start-Sleep -Seconds 2

# === START: CLOUDFLARED QUICK TUNNEL ===
Start-Process -WindowStyle Minimized -FilePath "powershell.exe" -ArgumentList @(
  "-NoProfile","-ExecutionPolicy","Bypass",
  "-Command", "cloudflared tunnel --url http://127.0.0.1:$PORT"
)

