# Idempotent: check OpenSSH client and config (no tunnel start).
$sshPath = $env:SSH_EXE_PATH
if (-not $sshPath) { $sshPath = "C:\Windows\System32\OpenSSH\ssh.exe" }
if (-not (Test-Path $sshPath)) {
    Write-Host "SSH not found at $sshPath. Install OpenSSH Client (Windows Settings > Apps > Optional features)."
    exit 0
}
$user = $env:SSH_USER
$host = $env:SSH_HOST
if (-not $user -or -not $host) {
    Write-Host "Set SSH_USER and SSH_HOST (e.g. atlas@hq.example.com). Use SSH keys (ssh-keygen, copy to HQ)."
    exit 0
}
Write-Host "SSH OK. Reverse tunnel: ssh -N -R ${env:SSH_REMOTE_PORT}:127.0.0.1:${env:SSH_LOCAL_PORT} ${user}@${host}"
exit 0
