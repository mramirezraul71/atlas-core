from __future__ import annotations

from pathlib import Path

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse, HTMLResponse

router = APIRouter(tags=["Radar Dashboard"])

_BASE_DIR = Path(__file__).resolve().parent
_ASSETS_DIR = _BASE_DIR / "assets"
_HTML_PATH = _ASSETS_DIR / "dashboard.html"


@router.get("/radar/dashboard", response_class=HTMLResponse)
async def radar_dashboard() -> HTMLResponse:
    if not _HTML_PATH.exists():
        raise HTTPException(status_code=404, detail="dashboard_not_found")
    return HTMLResponse(_HTML_PATH.read_text(encoding="utf-8"))


@router.get("/radar/dashboard/assets/{asset_name}")
async def radar_dashboard_assets(asset_name: str) -> FileResponse:
    safe = asset_name.strip().replace("\\", "/")
    if "/" in safe or ".." in safe:
        raise HTTPException(status_code=400, detail="invalid_asset_path")
    path = _ASSETS_DIR / safe
    if not path.exists() or not path.is_file():
        raise HTTPException(status_code=404, detail="asset_not_found")
    return FileResponse(path)
