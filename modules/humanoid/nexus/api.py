"""API REST para estado NEXUS (robot). PUSH consume NEXUS vía nexus_client."""
from fastapi import APIRouter

router = APIRouter(prefix="/nexus", tags=["NEXUS"])


@router.get("/status")
def nexus_status():
    """Estado de ATLAS NEXUS (robot, directivas, visión). Si no conecta, ok=False."""
    from modules.nexus_client import get_nexus_status
    return get_nexus_status()


@router.get("/robot-url")
def nexus_robot_url():
    """URL del Robot (cámaras, visión) para iframe."""
    import os
    url = (os.getenv("NEXUS_ROBOT_URL") or "http://127.0.0.1:5174").rstrip("/")
    return {"ok": True, "url": url}


@router.get("/chat-url")
def nexus_chat_url():
    """URL del Chat IA (atlas-dashboard-clean) para iframe."""
    import os
    url = (os.getenv("NEXUS_CHAT_URL") or "http://127.0.0.1:5173").rstrip("/")
    return {"ok": True, "url": url}


@router.get("/app-url")
def nexus_app_url():
    """URL de Atlas App (Goals, Directivas) para iframe."""
    import os
    url = (os.getenv("NEXUS_APP_URL") or "http://127.0.0.1:3000").rstrip("/")
    return {"ok": True, "url": url}
