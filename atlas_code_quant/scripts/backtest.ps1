$ErrorActionPreference = "Stop"

$repo = Split-Path -Parent $PSScriptRoot
Set-Location $repo

if (-not (Test-Path "reports")) { New-Item -ItemType Directory -Path "reports" | Out-Null }
if (-not (Test-Path "data\\backtest\\backtest_results")) { New-Item -ItemType Directory -Path "data\\backtest\\backtest_results" -Force | Out-Null }

$startDate = if ($env:BT_START_DATE) { $env:BT_START_DATE } else { "2026-04-01" }
$endDate = if ($env:BT_END_DATE) { $env:BT_END_DATE } else { "2026-04-15" }
$symbols = if ($env:BT_SYMBOLS) { $env:BT_SYMBOLS } else { "AAPL,MSFT,GOOGL,TSLA,NVDA" }

Write-Host "Running Atlas backtester..." -ForegroundColor Green
python scripts\run_backtest.py `
  --start-date $startDate `
  --end-date $endDate `
  --symbols $symbols `
  --timeframe 1h `
  --json-out "reports\backtest_summary_$endDate.json"

if ($LASTEXITCODE -ne 0) {
  Write-Host "Backtest failed with code $LASTEXITCODE" -ForegroundColor Red
  exit $LASTEXITCODE
}

Write-Host "Backtest completed." -ForegroundColor Green
