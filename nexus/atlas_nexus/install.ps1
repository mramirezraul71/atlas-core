# ATLAS NEXUS - Windows Installation
# Run: PowerShell -ExecutionPolicy Bypass -File install.ps1

Write-Host "🚀 ATLAS NEXUS - Installation Starting..." -ForegroundColor Cyan

# Check Python
if (Get-Command python -ErrorAction SilentlyContinue) {
    Write-Host "✓ Python found" -ForegroundColor Green
} else {
    Write-Host "✗ Python not found. Install from python.org" -ForegroundColor Red
    exit 1
}

# Create venv
if (-Not (Test-Path "venv")) {
    Write-Host "Creating virtual environment..." -ForegroundColor Yellow
    python -m venv venv
}

# Activate and install
Write-Host "Installing dependencies..." -ForegroundColor Yellow
& "venv\Scripts\Activate.ps1"
python -m pip install -U pip wheel setuptools --quiet
pip install -r requirements.txt --quiet

# Create dirs
"logs", "memory", "snapshots", "plugins" | ForEach-Object {
    if (-Not (Test-Path $_)) { New-Item -ItemType Directory -Path $_ | Out-Null }
}

# Config
if (-Not (Test-Path "config\.env")) {
    if (Test-Path ".env.example") {
        Copy-Item ".env.example" "config\.env"
        Write-Host "✓ Config created: config\.env" -ForegroundColor Green
    }
}

Write-Host "`n✅ Installation Complete!" -ForegroundColor Green
Write-Host "Next: Edit config\.env, then run: python nexus.py" -ForegroundColor Yellow
