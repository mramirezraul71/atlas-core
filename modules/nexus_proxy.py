"""Proxy inverso: /robot/* -> NEXUS. Acceso unificado desde el puerto de PUSH."""

import httpx
from atlas_adapter.bootstrap.service_urls import (
    get_nexus_base,
    get_nexus_enabled,
    get_nexus_timeout,
)
from fastapi import Request
from fastapi.responses import Response


async def proxy_to_nexus(request: Request, path: str) -> Response:
    """Reenvía la petición a NEXUS y devuelve la respuesta."""
    nexus_enabled = get_nexus_enabled()
    nexus_base = get_nexus_base()
    if not nexus_enabled or not nexus_base:
        return Response(
            content='{"ok":false,"error":"NEXUS no configurado"}',
            status_code=503,
            media_type="application/json",
        )

    url = f"{nexus_base}/{path}" if path else nexus_base + "/"
    headers = {
        k: v
        for k, v in request.headers.items()
        if k.lower() not in ("host", "connection")
    }
    try:
        async with httpx.AsyncClient(timeout=get_nexus_timeout()) as client:
            if request.method == "GET":
                response = await client.get(
                    url,
                    headers=headers,
                    params=request.query_params,
                )
            elif request.method == "POST":
                body = await request.body()
                response = await client.post(url, headers=headers, content=body)
            elif request.method == "PUT":
                body = await request.body()
                response = await client.put(url, headers=headers, content=body)
            elif request.method == "DELETE":
                response = await client.delete(url, headers=headers)
            elif request.method == "PATCH":
                body = await request.body()
                response = await client.patch(url, headers=headers, content=body)
            else:
                return Response(content="Method not allowed", status_code=405)

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
                and b"<html" in content[:2048]
            ):
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
                status_code=response.status_code,
                headers=out_headers,
                media_type=response.headers.get("content-type"),
            )
    except httpx.RequestError as exc:
        return Response(
            content=f'{{"ok":false,"error":"NEXUS no responde: {exc!s}"}}',
            status_code=502,
            media_type="application/json",
        )
