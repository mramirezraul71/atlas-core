"""Proxy /api/* -> Robot backend (8002). Para que el iframe /cuerpo/ funcione."""
import os
import httpx
from fastapi import Request
from fastapi.responses import Response

ROBOT_API = (os.getenv("NEXUS_ROBOT_API_URL") or "http://127.0.0.1:8002").rstrip("/")
TIMEOUT = 30.0


async def proxy_robot_api(request: Request, path: str) -> Response:
    """Reenvía /api/* al backend Robot (8002)."""
    url = f"{ROBOT_API}/api/{path}" if path else f"{ROBOT_API}/api/"
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
            else:
                r = await client.get(url, headers=headers)
            exclude = {"transfer-encoding", "connection", "content-encoding"}
            h = {k: v for k, v in r.headers.items() if k.lower() not in exclude}
            return Response(content=r.content, status_code=r.status_code, headers=h)
    except httpx.RequestError as e:
        return Response(
            content=f'{{"ok":false,"error":"Robot API no responde: {e!s}"}}',
            status_code=502,
            media_type="application/json",
        )
