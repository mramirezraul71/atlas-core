param(
    [string]$RepoPath = "C:\ATLAS_PUSH\_external\rauli-panaderia",
    [switch]$NoStart,
    [switch]$AllowUnsupportedNode
)

$ErrorActionPreference = "Stop"

function Write-Info([string]$msg) {
    Write-Host "[INFO] $msg" -ForegroundColor Cyan
}

function Write-WarnMsg([string]$msg) {
    Write-Host "[WARN] $msg" -ForegroundColor Yellow
}

function Write-ErrMsg([string]$msg) {
    Write-Host "[ERROR] $msg" -ForegroundColor Red
}

function Test-Port([int]$Port) {
    try {
        $c = Get-NetTCPConnection -State Listen -LocalPort $Port -ErrorAction SilentlyContinue
        return ($null -ne $c)
    } catch {
        return $false
    }
}

if (-not (Test-Path $RepoPath)) {
    Write-ErrMsg "Repo no encontrado: $RepoPath"
    Write-Host "Clona primero: git clone https://github.com/mramirezraul71/rauli-panaderia.git $RepoPath"
    exit 1
}

$backendDir = Join-Path $RepoPath "backend"
$frontendDir = Join-Path $RepoPath "frontend"

if (-not (Test-Path (Join-Path $backendDir "package.json"))) {
    Write-ErrMsg "No existe backend/package.json en $backendDir"
    exit 1
}
if (-not (Test-Path (Join-Path $frontendDir "package.json"))) {
    Write-ErrMsg "No existe frontend/package.json en $frontendDir"
    exit 1
}

$nodeVerRaw = ""
try {
    $nodeVerRaw = (& node -v).Trim()
} catch {
    Write-ErrMsg "Node.js no esta disponible en PATH."
    exit 1
}

if (-not $nodeVerRaw.StartsWith("v")) {
    Write-ErrMsg "Version de Node invalida: $nodeVerRaw"
    exit 1
}

$nodeMajor = 0
try {
    $nodeMajor = [int](($nodeVerRaw.TrimStart("v")).Split(".")[0])
} catch {
    Write-ErrMsg "No se pudo parsear Node version: $nodeVerRaw"
    exit 1
}

Write-Info "Node detectado: $nodeVerRaw"
if (($nodeMajor -ne 20) -and ($nodeMajor -ne 22)) {
    $msg = "Este repo recomienda Node 20/22. Detectado $nodeVerRaw."
    if ($AllowUnsupportedNode) {
        Write-WarnMsg "$msg Continuando por bandera -AllowUnsupportedNode."
    } else {
        Write-ErrMsg $msg
        Write-Host "Usa nvm-windows, ejemplo: nvm install 22.11.0 ; nvm use 22.11.0"
        exit 1
    }
}

if (-not (Test-Path (Join-Path $backendDir "node_modules"))) {
    Write-Info "Instalando dependencias backend..."
    Push-Location $backendDir
    try {
        npm install
    } finally {
        Pop-Location
    }
} else {
    Write-Info "Dependencias backend: OK"
}

if (-not (Test-Path (Join-Path $frontendDir "node_modules"))) {
    Write-Info "Instalando dependencias frontend..."
    Push-Location $frontendDir
    try {
        npm install
    } finally {
        Pop-Location
    }
} else {
    Write-Info "Dependencias frontend: OK"
}

if ($NoStart) {
    Write-Info "NoStart activo. Validacion completada sin levantar servicios."
    Write-Host "Dashboard esperado: http://localhost:5173/dashboard"
    exit 0
}

if (Test-Port 3000) {
    Write-WarnMsg "Puerto 3000 ya esta en uso. Backend puede fallar o ya estar corriendo."
}
if (Test-Port 5173) {
    Write-WarnMsg "Puerto 5173 ya esta en uso. Frontend puede fallar o ya estar corriendo."
}

Write-Info "Levantando backend (npm start)..."
Start-Process powershell.exe -ArgumentList @(
    "-NoExit",
    "-Command",
    "Set-Location '$backendDir'; npm start"
) -WorkingDirectory $backendDir | Out-Null

Write-Info "Levantando frontend (npm run dev)..."
Start-Process powershell.exe -ArgumentList @(
    "-NoExit",
    "-Command",
    "Set-Location '$frontendDir'; npm run dev"
) -WorkingDirectory $frontendDir | Out-Null

Start-Sleep -Seconds 5
$b = Test-Port 3000
$f = Test-Port 5173

Write-Info ("Backend 3000 escuchando: " + $b)
Write-Info ("Frontend 5173 escuchando: " + $f)
Write-Host "Abre en navegador: http://localhost:5173/dashboard"
