param(
  [string]$BaseUrl = "http://127.0.0.1:8791",
  [ValidateSet("capabilities","models","ai_status","goal","step","cursor_run","cursor_status","terminal","navigate_open","navigate_extract","architect","builder","multifuncion")]
  [string]$Action = "capabilities",
  [string]$Goal = "Analiza este modulo y crea plan de refactor",
  [string]$Model = "ollama:deepseek-r1:14b",
  [string]$Mode = "auto",
  [int]$Depth = 3,
  [int]$StepIndex = 0,
  [string]$Command = "python -V",
  [string]$Url = "http://127.0.0.1:8791/health"
)

$ErrorActionPreference = "Stop"

function Invoke-AtlasApi {
  param(
    [ValidateSet("GET","POST")]
    [string]$Method,
    [string]$Path,
    [object]$Body = $null
  )

  $uri = "$BaseUrl$Path"
  if ($Method -eq "GET") {
    return Invoke-RestMethod -Method Get -Uri $uri -TimeoutSec 90
  }

  $json = if ($null -ne $Body) { $Body | ConvertTo-Json -Depth 10 } else { "{}" }
  return Invoke-RestMethod -Method Post -Uri $uri -ContentType "application/json" -Body $json -TimeoutSec 120
}

switch ($Action) {
  "capabilities" {
    Invoke-AtlasApi -Method GET -Path "/api/workspace/capabilities"
    break
  }
  "models" {
    Invoke-AtlasApi -Method GET -Path "/agent/models"
    break
  }
  "ai_status" {
    Invoke-AtlasApi -Method GET -Path "/ai/status"
    break
  }
  "goal" {
    $body = @{
      goal = $Goal
      mode = $Mode
      depth = $Depth
      model = $Model
      sync_config = $true
    }
    Invoke-AtlasApi -Method POST -Path "/agent/goal" -Body $body
    break
  }
  "architect" {
    $body = @{
      goal = $Goal
      mode = "auto"
      depth = 3
      model = "ollama:deepseek-r1:14b"
      sync_config = $true
    }
    Invoke-AtlasApi -Method POST -Path "/agent/goal" -Body $body
    break
  }
  "builder" {
    $body = @{
      goal = $Goal
      mode = "auto"
      depth = 2
      model = "ollama:deepseek-coder:6.7b"
      sync_config = $true
    }
    Invoke-AtlasApi -Method POST -Path "/agent/goal" -Body $body
    break
  }
  "multifuncion" {
    $body = @{
      goal = $Goal
      mode = "auto"
      depth = 2
      model = "ollama:llama3.1:latest"
      sync_config = $true
    }
    Invoke-AtlasApi -Method POST -Path "/agent/goal" -Body $body
    break
  }
  "step" {
    $body = @{ step_index = $StepIndex; approve = $true }
    Invoke-AtlasApi -Method POST -Path "/agent/step/execute" -Body $body
    break
  }
  "cursor_run" {
    $body = @{ goal = $Goal; mode = $Mode; depth = $Depth; profile = "owner" }
    Invoke-AtlasApi -Method POST -Path "/cursor/run" -Body $body
    break
  }
  "cursor_status" {
    Invoke-AtlasApi -Method GET -Path "/cursor/status"
    break
  }
  "terminal" {
    $body = @{ command = $Command; timeout_sec = 120; actor = "workspace_ui"; role = "owner" }
    Invoke-AtlasApi -Method POST -Path "/api/workspace/terminal/execute" -Body $body
    break
  }
  "navigate_open" {
    $body = @{ action = "open_url"; payload = @{ url = $Url; show_browser = $false } }
    Invoke-AtlasApi -Method POST -Path "/api/workspace/navigate" -Body $body
    break
  }
  "navigate_extract" {
    $body = @{ action = "extract_text"; payload = @{} }
    Invoke-AtlasApi -Method POST -Path "/api/workspace/navigate" -Body $body
    break
  }
}
