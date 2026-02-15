from __future__ import annotations

import time
from typing import Any, Dict, List, Optional


def scan_network(protocol: str = "rtsp") -> Dict[str, Any]:
    """
    Escanea la red local buscando nodos RTSP/ONVIF.
    Wrapper sobre `modules.humanoid.vision.ubiq.discover_local_cameras`.
    """
    proto = (protocol or "rtsp").strip().lower()
    try:
        from modules.humanoid.vision.ubiq.discovery import discover_local_cameras

        r = discover_local_cameras(protocol=proto)
        return {"ok": bool(r.get("ok", True)), "protocol": proto, "result": r}
    except Exception as e:
        return {"ok": False, "protocol": proto, "error": str(e)[:200]}


def stream_proxy(source_ip: str, *, variant: str = "mobile") -> Dict[str, Any]:
    """
    Inicia proxy/stream HLS desde una cámara registrada.
    Se basa en registry: busca cam_id por IP y llama start_stream(cam_id, variant).
    """
    ip = (source_ip or "").strip()
    if not ip:
        return {"ok": False, "error": "missing source_ip"}
    try:
        from modules.humanoid.vision.ubiq.registry import list_cameras
        from modules.humanoid.vision.ubiq.streaming import start_stream

        cams = list_cameras(limit=200) or []
        cam = next((c for c in cams if (c.get("ip") or "").strip() == ip), None)
        if not cam:
            return {"ok": False, "error": "camera_not_found_for_ip", "ip": ip}
        cam_id = cam.get("id")
        r = start_stream(str(cam_id), variant=variant)
        return {"ok": bool(r.get("ok")), "cam_id": cam_id, "variant": variant, "result": r}
    except Exception as e:
        return {"ok": False, "error": str(e)[:200], "ip": ip}


def perimeter_check(*, snapshot_limit: int = 8, emit_ops: bool = True) -> Dict[str, Any]:
    """
    Ronda de vigilancia: recorre cámaras registradas y toma un snapshot por cada una (hasta snapshot_limit).
    """
    try:
        from modules.humanoid.vision.ubiq.registry import list_cameras
        from modules.humanoid.vision.ubiq.streaming import take_snapshot
        from modules.humanoid.comms.ops_bus import emit as ops_emit

        cams = list_cameras(limit=max(1, int(snapshot_limit or 8)))
        out: List[Dict[str, Any]] = []
        for cam in cams[: max(1, int(snapshot_limit or 8))]:
            cid = cam.get("id")
            snap = take_snapshot(str(cid), timeout_s=3.0)
            out.append({"cam_id": cid, "ip": cam.get("ip"), "ok": bool(snap.get("ok")), "error": snap.get("error")})
            if emit_ops and snap.get("ok"):
                try:
                    ops_emit("vision", f"Perímetro: snapshot OK ({cid})", level="low", data={"cam_id": cid})
                except Exception:
                    pass
            time.sleep(0.1)
        ok = all(x.get("ok") for x in out) if out else True
        return {"ok": ok, "checked": len(out), "results": out}
    except Exception as e:
        return {"ok": False, "error": str(e)[:200]}

