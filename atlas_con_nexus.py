#!/usr/bin/env python3
"""
Servidor Atlas con Proxy a NEXUS
Conecta el dashboard de NEXUS a través del puerto 8000
"""

import os
import time

import requests
from fastapi import FastAPI
from fastapi.responses import JSONResponse, Response

app = FastAPI(title="Atlas con Proxy a NEXUS")


@app.get("/health")
def health():
    return {"status": "healthy", "service": "ATLAS con Proxy", "ok": True}


@app.get("/api/monitor/snapshot")
def monitor_snapshot():
    shell_history = [
        {
            "ts": time.strftime("%H:%M:%S"),
            "action": "exec_command",
            "payload_json": '{"command": "dir"}',
            "ok": True,
            "ms": 150,
        }
    ]

    return {
        "ok": True,
        "data": {
            "shell_history": shell_history,
            "processes": [],
            "health": {"cpu_pct": 25, "ram_pct": 45},
            "audit": [],
            "tasks": [],
            "bitacora": [],
        },
        "ms": 50,
    }


@app.get("/nexus/{path:path}")
async def proxy_to_nexus(path: str):
    try:
        nexus_url = (
            f"http://localhost:8002/{path}" if path else "http://localhost:8002/"
        )
        response = requests.get(nexus_url, timeout=10)
        return Response(
            content=response.content,
            status_code=response.status_code,
            headers=dict(response.headers),
        )
    except Exception as e:
        return JSONResponse(
            content={"ok": False, "error": f"Error proxy: {str(e)}"}, status_code=503
        )


@app.get("/")
async def root():
    try:
        response = requests.get("http://localhost:8002/", timeout=5)
        return Response(
            content=response.content,
            status_code=response.status_code,
            headers=dict(response.headers),
        )
    except:
        return JSONResponse(
            {
                "message": "Atlas con Proxy a NEXUS",
                "nexus_status": "No disponible",
                "endpoints": ["/health", "/api/monitor/snapshot", "/nexus/*"],
            }
        )


if __name__ == "__main__":
    import uvicorn

    print("🚀 Iniciando Atlas con Proxy a NEXUS...")
    print("📡 NEXUS disponible en: http://localhost:8002")
    print("🔗 Proxy disponible en: http://localhost:8000")
    uvicorn.run(app, host="0.0.0.0", port=8000)
