"""Proxy inverso: /robot/* -> NEXUS. Acceso unificado desde el puerto de PUSH."""
import os
import httpx
from fastapi import Request
from fastapi.responses import Response

NEXUS_BASE = (os.getenv("NEXUS_BASE_URL") or "").rstrip("/")
NEXUS_ENABLED = os.getenv("NEXUS_ENABLED", "false").strip().lower() in ("1", "true", "yes", "y", "on")
TIMEOUT = float(os.getenv("NEXUS_TIMEOUT", "30"))


async def proxy_to_nexus(request: Request, path: str) -> Response:
    """Reenvía la petición a NEXUS y devuelve la respuesta."""
    if not NEXUS_ENABLED or not NEXUS_BASE:
        return Response(
            content='{"ok":false,"error":"NEXUS no configurado"}',
            status_code=503,
            media_type="application/json",
        )
    url = f"{NEXUS_BASE}/{path}" if path else NEXUS_BASE + "/"
    headers = {k: v for k, v in request.headers.items() if k.lower() not in ("host", "connection")}
    try:
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            if request.method == "GET":
                r = await client.get(url, headers=headers, params=request.query_params)
            elif request.method == "POST":
                body = await request.body()
                r = await client.post(url, headers=headers, content=body)
            elif request.method == "PUT":
                body = await request.body()
                r = await client.put(url, headers=headers, content=body)
            elif request.method == "DELETE":
                r = await client.delete(url, headers=headers)
            elif request.method == "PATCH":
                body = await request.body()
                r = await client.patch(url, headers=headers, content=body)
            else:
                return Response(content="Method not allowed", status_code=405)
            # Filtrar headers de respuesta que no deben reenviarse
            exclude = {"transfer-encoding", "connection", "content-encoding"}
            out_headers = {k: v for k, v in r.headers.items() if k.lower() not in exclude}
            content = r.content
            ct = (r.headers.get("content-type") or "").lower()
            # Inyectar barra de vuelta al dashboard principal (cámaras, voz) en HTML
            if "text/html" in ct and r.status_code == 200 and b"<html" in content[:2048]:
                try:
                    body = content.decode("utf-8", errors="replace")
                    banner = (
                        '<div style="position:sticky;top:0;z-index:9999;background:#1a1a2e;padding:.5rem 1rem;'
                        'border-bottom:1px solid #333;font-size:.9rem;box-shadow:0 2px 8px rgba(0,0,0,.3);">'
                        '<a href="#" onclick="window.location.href=window.location.origin+\'/ui\';return false;" '
                        'style="color:#a78bfa;text-decoration:none;font-weight:600;">← Volver al panel principal</a></div>'
                    )
                    if "<body" in body:
                        idx = body.find(">", body.find("<body")) + 1
                        body = body[:idx] + banner + body[idx:]
                    else:
                        body = banner + body
                    content = body.encode("utf-8")
                except Exception:
                    pass
            return Response(
                content=content,
                status_code=r.status_code,
                headers=out_headers,
                media_type=r.headers.get("content-type"),
            )
    except httpx.RequestError as e:
        return Response(
            content=f'{{"ok":false,"error":"NEXUS no responde: {e!s}"}}',
            status_code=502,
            media_type="application/json",
        )
