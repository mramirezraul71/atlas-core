param(
    [string]$To = "gpinchev@internetmasters.com",
    [string]$CredFile = "C:\dev\credenciales.txt",
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"

function Get-CredValue {
    param([string]$Key)
    if (-not (Test-Path $CredFile)) { return "" }
    $line = Get-Content $CredFile | Where-Object { $_ -match "^$([regex]::Escape($Key))=" } | Select-Object -First 1
    if (-not $line) { return "" }
    return ($line -replace "^$([regex]::Escape($Key))=", "").Trim()
}

$smtpHost = Get-CredValue "EMAIL_SMTP_HOST"
$smtpPort = Get-CredValue "EMAIL_SMTP_PORT"
$smtpUser = Get-CredValue "EMAIL_SMTP_USER"
$smtpPass = Get-CredValue "EMAIL_SMTP_PASS"
$fromAddr = Get-CredValue "EMAIL_FROM"

if (-not $smtpHost -or -not $smtpPort -or -not $smtpUser -or -not $smtpPass -or -not $fromAddr) {
    throw "SMTP config incomplete in $CredFile"
}

$subject = "Domain MIDOMINIO.COM confirmation - nameserver update"
$body = @(
    "Hola Gabriel,",
    "",
    "Confirmo que el dominio correcto es MIDOMINIO.COM.",
    "Repito: el dominio correcto es MIDOMINIO.COM (midominio.com).",
    "",
    "Solicito cambiar los nameservers de MIDOMINIO.COM a:",
    "- elsa.ns.cloudflare.com",
    "- margo.ns.cloudflare.com",
    "",
    "Autoriza: Raul Martinez Ramirez (mramirezraul71@gmail.com)"
) -join "`r`n"

if ($DryRun.IsPresent) {
    Write-Output "DRY_RUN to=$To from=$fromAddr host=$smtpHost port=$smtpPort"
    Write-Output "SUBJECT: $subject"
    Write-Output "BODY_START"
    Write-Output $body
    Write-Output "BODY_END"
    exit 0
}

$mail = New-Object System.Net.Mail.MailMessage
$mail.From = $fromAddr
$mail.To.Add($To)
$mail.Subject = $subject
$mail.Body = $body
$mail.IsBodyHtml = $false

$portNum = [int]$smtpPort
$sslAttempts = @()
if ($portNum -eq 465 -or $portNum -eq 587) {
    $sslAttempts = @($true, $false)
} else {
    $sslAttempts = @($false, $true)
}

$sent = $false
$lastError = ""
foreach ($useSsl in $sslAttempts) {
    $client = $null
    try {
        $client = New-Object System.Net.Mail.SmtpClient($smtpHost, $portNum)
        $client.EnableSsl = $useSsl
        $client.Credentials = New-Object System.Net.NetworkCredential($smtpUser, $smtpPass)
        $client.Send($mail)
        Write-Output "MAIL_SENT to=$To ssl=$useSsl"
        $sent = $true
        break
    } catch {
        $lastError = $_.Exception.Message
        Write-Output "MAIL_SEND_ATTEMPT_FAILED ssl=$useSsl msg=$lastError"
    } finally {
        if ($client) { $client.Dispose() }
    }
}

$mail.Dispose()

if (-not $sent) {
    throw "Could not send email. Last error: $lastError"
}

