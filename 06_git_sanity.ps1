param(
  [Parameter(Mandatory=$true)][string]$RepoPath
)

Write-Host "== Git sanity ==" -ForegroundColor Cyan
Set-Location $RepoPath

if (!(Test-Path (Join-Path $RepoPath ".git"))) {
  Write-Warning "No veo .git en $RepoPath"
  exit 0
}

git status
git remote -v
git branch -vv

Write-Host "`nSugerencia release:" -ForegroundColor Yellow
Write-Host "  git tag v1.0.0"
Write-Host "  git push origin v1.0.0"
