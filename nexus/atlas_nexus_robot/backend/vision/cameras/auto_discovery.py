from __future__ import annotations

import asyncio
import multiprocessing
from typing import Any, Dict, List

from .detector import detect_cameras
from .network import discover_network_cameras
from .protocols.mdns import scan_mdns_cameras
from .unified_registry import register_camera


def _discover_onvif_devices() -> List[Dict[str, Any]]:
    try:
        from modules.humanoid.vision.ubiq.onvif_wsdiscovery import discover_onvif_devices

        devices = discover_onvif_devices(timeout_s=2.0, max_results=50)
    except Exception:
        devices = []
    out: List[Dict[str, Any]] = []
    for d in devices:
        ip = d.get("ip")
        xaddr = d.get("xaddr", "")
        if not ip:
            continue
        url = xaddr or f"http://{ip}/onvif/device_service"
        out.append(
            {
                "ip": ip,
                "url": url,
                "protocol": "onvif",
                "model": f"ONVIF {ip}",
                "source": "onvif",
                "type": "network",
            }
        )
    return out


def _usb_worker(q) -> None:
    try:
        q.put({"ok": True, "data": detect_cameras()})
    except Exception as e:
        q.put({"ok": False, "error": str(e), "data": []})


def _network_worker(q) -> None:
    try:
        q.put({"ok": True, "data": discover_network_cameras()})
    except Exception as e:
        q.put({"ok": False, "error": str(e), "data": []})


def _onvif_worker(q) -> None:
    try:
        q.put({"ok": True, "data": _discover_onvif_devices()})
    except Exception as e:
        q.put({"ok": False, "error": str(e), "data": []})


def _run_with_timeout(worker, timeout_s: float) -> Dict[str, Any]:
    ctx = multiprocessing.get_context("spawn")
    q = ctx.Queue()
    p = ctx.Process(target=worker, args=(q,))
    p.daemon = True
    p.start()
    p.join(float(timeout_s))
    if p.is_alive():
        try:
            p.terminate()
        except Exception:
            pass
        try:
            p.join(1.0)
        except Exception:
            pass
        return {"ok": False, "error": "timeout", "data": []}
    try:
        return q.get_nowait()
    except Exception:
        return {"ok": False, "error": "no_result", "data": []}


async def discover_all_cameras(auto_register: bool = True) -> Dict[str, Any]:
    loop = asyncio.get_running_loop()
    usb_task = loop.run_in_executor(None, _run_with_timeout, _usb_worker, 30.0)
    net_task = loop.run_in_executor(None, _run_with_timeout, _network_worker, 30.0)
    onvif_task = loop.run_in_executor(None, _run_with_timeout, _onvif_worker, 10.0)
    mdns_task = asyncio.create_task(scan_mdns_cameras(timeout_s=4.0))

    usb_result = await usb_task
    net_result = await net_task
    onvif_result = await onvif_task
    mdns = await mdns_task

    usb = usb_result.get("data", []) if isinstance(usb_result, dict) else []
    net = net_result.get("data", []) if isinstance(net_result, dict) else []
    onvif = onvif_result.get("data", []) if isinstance(onvif_result, dict) else []

    registered_count = 0
    errors: List[str] = []
    if isinstance(usb_result, dict) and not usb_result.get("ok", False):
        errors.append(f"usb_discovery: {usb_result.get('error')}")
    if isinstance(net_result, dict) and not net_result.get("ok", False):
        errors.append(f"network_discovery: {net_result.get('error')}")
    if isinstance(onvif_result, dict) and not onvif_result.get("ok", False):
        errors.append(f"onvif_discovery: {onvif_result.get('error')}")

    if auto_register:
        for cam in usb:
            try:
                idx = cam.get("index", 0)
                register_camera(
                    type="usb",
                    url=f"cv2://{idx}",
                    label=cam.get("model", f"USB {idx}"),
                    priority=1,
                    connection_method="direct",
                    metadata=cam,
                )
                registered_count += 1
            except Exception as e:
                errors.append(f"usb: {e}")

        for cam in (net + onvif + mdns):
            try:
                register_camera(
                    type=cam.get("type", "network"),
                    url=cam.get("url", ""),
                    label=cam.get("model", cam.get("ip", "Network Camera")),
                    priority=2,
                    connection_method=cam.get("source", "network"),
                    metadata=cam,
                )
                registered_count += 1
            except Exception as e:
                errors.append(f"network: {e}")

    return {
        "ok": True,
        "usb_cameras": usb,
        "network_cameras": net,
        "onvif_cameras": onvif,
        "mdns_cameras": mdns,
        "usb_count": len(usb),
        "network_count": len(net),
        "onvif_count": len(onvif),
        "mdns_count": len(mdns),
        "registered_count": registered_count,
        "errors": errors,
    }
