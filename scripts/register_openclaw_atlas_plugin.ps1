param(
  [string]$Workspace = "C:\ATLAS_PUSH",
  [string]$ConfigPath = "$HOME\.openclaw\openclaw.json",
  [string]$PythonPath = "",
  [string]$PluginPath = "",
  [string]$SkillConfig = ""
)

$ErrorActionPreference = "Stop"

function Normalize-AtlasPath {
  param([string]$PathValue)
  return $PathValue.Replace("\", "/")
}

function ConvertTo-AtlasHashtable {
  param([object]$InputObject)

  if ($null -eq $InputObject) {
    return $null
  }

  if ($InputObject -is [System.Collections.IDictionary]) {
    $result = @{}
    foreach ($key in $InputObject.Keys) {
      $result[$key] = ConvertTo-AtlasHashtable $InputObject[$key]
    }
    return $result
  }

  if ($InputObject -is [System.Collections.IEnumerable] -and $InputObject -isnot [string]) {
    $items = @()
    foreach ($item in $InputObject) {
      $items += ,(ConvertTo-AtlasHashtable $item)
    }
    return $items
  }

  if ($InputObject -is [psobject]) {
    $props = $InputObject.PSObject.Properties
    if ($props.Count -gt 0) {
      $result = @{}
      foreach ($prop in $props) {
        $result[$prop.Name] = ConvertTo-AtlasHashtable $prop.Value
      }
      return $result
    }
  }

  return $InputObject
}

if ([string]::IsNullOrWhiteSpace($PythonPath)) {
  $PythonPath = Join-Path $Workspace ".venv_openclaw_atlas\Scripts\python.exe"
}
if ([string]::IsNullOrWhiteSpace($PluginPath)) {
  $PluginPath = Join-Path $Workspace "tools\openclaw_plugins\atlas-robot-skill"
}
if ([string]::IsNullOrWhiteSpace($SkillConfig)) {
  $SkillConfig = Join-Path $Workspace "modules\humanoid\openclaw\config.json"
}

foreach ($requiredPath in @($Workspace, $PythonPath, $PluginPath, $SkillConfig)) {
  if (!(Test-Path $requiredPath)) {
    throw "Ruta requerida no encontrada: $requiredPath"
  }
}

$configDir = Split-Path -Parent $ConfigPath
if (!(Test-Path $configDir)) {
  New-Item -ItemType Directory -Path $configDir -Force | Out-Null
}

$config = @{}
if (Test-Path $ConfigPath) {
  $raw = Get-Content $ConfigPath -Raw
  if ($raw.Trim()) {
    $config = ConvertTo-AtlasHashtable (ConvertFrom-Json $raw)
  }
}

if (-not $config.ContainsKey("agents")) { $config["agents"] = @{} }
if (-not $config["agents"].ContainsKey("defaults")) { $config["agents"]["defaults"] = @{} }
$config["agents"]["defaults"]["workspace"] = (Normalize-AtlasPath $Workspace)

if (-not $config.ContainsKey("plugins")) { $config["plugins"] = @{} }
if (-not $config["plugins"].ContainsKey("allow")) { $config["plugins"]["allow"] = @() }
if (-not $config["plugins"].ContainsKey("load")) { $config["plugins"]["load"] = @{} }
if (-not $config["plugins"]["load"].ContainsKey("paths")) { $config["plugins"]["load"]["paths"] = @() }
if (-not $config["plugins"].ContainsKey("entries")) { $config["plugins"]["entries"] = @{} }

$legacyPath = Normalize-AtlasPath (Join-Path $HOME ".openclaw\extensions\atlas-robot-skill")
$repoPluginPath = Normalize-AtlasPath $PluginPath
$pluginPaths = @($config["plugins"]["load"]["paths"]) |
  Where-Object { $_ -and $_ -ne $legacyPath -and $_ -ne $repoPluginPath }
$pluginPaths += $repoPluginPath
$config["plugins"]["load"]["paths"] = @($pluginPaths | Select-Object -Unique)

$pluginAllow = @($config["plugins"]["allow"])
if ($pluginAllow -notcontains "atlas-robot-skill") {
  $pluginAllow += "atlas-robot-skill"
}
$config["plugins"]["allow"] = @($pluginAllow | Select-Object -Unique)

$config["plugins"]["entries"]["atlas-robot-skill"] = @{
  enabled = $true
  config = @{
    pythonPath = (Normalize-AtlasPath $PythonPath)
    skillConfig = (Normalize-AtlasPath $SkillConfig)
    workspace = (Normalize-AtlasPath $Workspace)
  }
}

if (-not $config.ContainsKey("tools")) { $config["tools"] = @{} }
if (-not $config["tools"].ContainsKey("allow")) { $config["tools"]["allow"] = @() }
$toolAllow = @($config["tools"]["allow"])
if ($toolAllow -notcontains "atlas-robot-skill") {
  $toolAllow += "atlas-robot-skill"
}
$config["tools"]["allow"] = @($toolAllow | Select-Object -Unique)

$json = $config | ConvertTo-Json -Depth 12
Set-Content -Path $ConfigPath -Value $json -Encoding UTF8

Write-Host "OpenClaw config actualizada: $ConfigPath" -ForegroundColor Green
Write-Host "Plugin repo-local: $repoPluginPath" -ForegroundColor Cyan

& openclaw config validate
if ($LASTEXITCODE -ne 0) {
  throw "openclaw config validate fallo"
}

& openclaw plugins list
