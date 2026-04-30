"""Proxy para Robot (cámaras, visión): /cuerpo/* -> Robot UI y Robot API.

Problema histórico:
- `NEXUS_ROBOT_URL` suele apuntar al frontend.
- La API real de cámaras/visión vive en `NEXUS_ROBOT_API_URL` o `ROBOT_BASE_URL`.
Si se usa una sola base para todo, rutas tipo `/cuerpo/api/...` terminan yendo al
frontend y fallan.

Solución:
- Enrutado por prefijo: UI para HTML/assets; API para `/api/*`, `/docs`,
  `/openapi.json`, `/health`, `/status`, `/ws`, etc.
"""

import httpx
from atlas_adapter.bootstrap.service_urls import (
    get_camera_proxy_timeout,
    get_nexus_timeout,
    get_robot_api_base,
    get_robot_ui_base,
)
from fastapi import Request
from fastapi.responses import Response

_API_PREFIXES = (
    "api/",
    "docs",
    "redoc",
    "openapi",
    "openapi.json",
    "ws",
    "health",
    "status",
)


def _pick_base(path: str) -> str:
    clean_path = (path or "").lstrip("/")
    for prefix in _API_PREFIXES:
        if clean_path == prefix or clean_path.startswith(prefix):
            return get_robot_api_base()
    return get_robot_ui_base()


async def proxy_to_cuerpo(request: Request, path: str) -> Response:
    """Reenvía a Robot frontend (cámaras, visión, dashboard)."""
    base = _pick_base(path)
    url = f"{base}/{path}" if path else base + "/"
    headers = {
        k: v
        for k, v in request.headers.items()
        if k.lower() not in ("host", "connection")
    }
    clean_path = (path or "").lstrip("/")
    timeout = get_nexus_timeout()
    if clean_path.startswith("api/camera/"):
        timeout = get_camera_proxy_timeout()
    try:
        async with httpx.AsyncClient(timeout=timeout, follow_redirects=True) as client:
            if request.method == "GET":
                response = await client.get(
                    url,
                    headers=headers,
                    params=request.query_params,
                )
            elif request.method == "POST":
                body = await request.body()
                response = await client.post(url, headers=headers, content=body)
            else:
                response = await client.get(url, headers=headers)
            exclude = {"transfer-encoding", "connection", "content-encoding"}
            out_headers = {
                k: v
                for k, v in response.headers.items()
                if k.lower() not in exclude
            }
            content = response.content
            content_type = (response.headers.get("content-type") or "").lower()
            if (
                "text/html" in content_type
                and response.status_code == 200
                and b"<head" in content[:2048]
            ):
                try:
                    body = content.decode("utf-8", errors="replace")
                    route_prefix = "/robot/" if request.url.path.startswith("/robot") else "/cuerpo/"
                    base_tag = f'<base href="{route_prefix}">'
                    if "<base " not in body.lower():
                        body = body.replace("<head>", "<head>" + base_tag, 1)
                        body = body.replace("<head ", "<head " + base_tag, 1)
                    content = body.encode("utf-8")
                except Exception:
                    pass
            return Response(
                content=content,
                status_code=response.status_code,
                headers=out_headers,
            )
    except httpx.RequestError as exc:
        err_msg = str(exc).replace('"', "'")
        if clean_path.startswith("api/"):
            return Response(
                content=f'{{"ok":false,"error":"Robot no responde: {err_msg}","path":"{clean_path}"}}',
                status_code=503,
                media_type="application/json",
            )
        if "stream" in clean_path or "camera" in clean_path:
            html = (
                '<html><body style="margin:0;background:#1a1a2e;color:#94a3b8;font-family:sans-serif;'
                'display:flex;align-items:center;justify-content:center;min-height:100vh;">'
                f"<p>Cámara no disponible (Robot no responde: {err_msg[:80]})</p></body></html>"
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
