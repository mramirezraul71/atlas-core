"""Proxy para Robot (cámaras, visión): /cuerpo/* -> NEXUS_ROBOT_URL (5174).
Permite embed del dashboard Robot sin perder funcionalidad de NEXUS."""
import os
import httpx
from fastapi import Request
from fastapi.responses import Response

ROBOT_BASE = (os.getenv("NEXUS_ROBOT_URL") or "http://127.0.0.1:8002").rstrip("/")
TIMEOUT = float(os.getenv("NEXUS_TIMEOUT", "30"))


async def proxy_to_cuerpo(request: Request, path: str) -> Response:
    """Reenvía a Robot frontend (cámaras, visión, dashboard)."""
    url = f"{ROBOT_BASE}/{path}" if path else ROBOT_BASE + "/"
    headers = {k: v for k, v in request.headers.items() if k.lower() not in ("host", "connection")}
    try:
        async with httpx.AsyncClient(timeout=TIMEOUT, follow_redirects=True) as client:
            if request.method == "GET":
                r = await client.get(url, headers=headers, params=request.query_params)
            elif request.method == "POST":
                body = await request.body()
                r = await client.post(url, headers=headers, content=body)
            else:
                r = await client.get(url, headers=headers)
            exclude = {"transfer-encoding", "connection", "content-encoding"}
            out_headers = {k: v for k, v in r.headers.items() if k.lower() not in exclude}
            content = r.content
            ct = (r.headers.get("content-type") or "").lower()
            if "text/html" in ct and r.status_code == 200 and b"<head" in content[:2048]:
                try:
                    body = content.decode("utf-8", errors="replace")
                    base_tag = '<base href="/cuerpo/">'
                    if "<base " not in body.lower():
                        body = body.replace("<head>", "<head>" + base_tag, 1)
                        body = body.replace("<head ", "<head " + base_tag, 1)
                    content = body.encode("utf-8")
                except Exception:
                    pass
            return Response(content=content, status_code=r.status_code, headers=out_headers)
    except httpx.RequestError as e:
        # 503 cuando Robot/cámara no está disponible (más claro que 502 para el cliente)
        err_msg = str(e).replace('"', "'")
        if "stream" in path or "camera" in path:
            # Respuesta mínima para <img> para no mostrar icono roto
            html = (
                '<html><body style="margin:0;background:#1a1a2e;color:#94a3b8;font-family:sans-serif;'
                'display:flex;align-items:center;justify-content:center;min-height:100vh;">'
                f'<p>Cámara no disponible (Robot no responde: {err_msg[:80]})</p></body></html>'
            )
            return Response(
                content=html.encode("utf-8"),
                status_code=503,
                media_type="text/html; charset=utf-8",
                headers={"Cache-Control": "no-store"},
            )
        return Response(
            content=f'{{"ok":false,"error":"Robot no responde: {err_msg}"}}',
            status_code=503,
            media_type="application/json",
        )
