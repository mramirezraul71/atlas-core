from __future__ import annotations

import asyncio
from typing import Any, Dict, List


async def scan_mdns_cameras(timeout_s: float = 5.0) -> List[Dict[str, Any]]:
    try:
        from zeroconf import ServiceBrowser, ServiceListener, Zeroconf
    except Exception:
        return []

    found: List[Dict[str, Any]] = []
    seen = set()

    class _Listener(ServiceListener):
        def add_service(self, zc, service_type, name):
            try:
                info = zc.get_service_info(service_type, name, timeout=1500)
                if not info:
                    return
                addrs = info.parsed_addresses()
                if not addrs:
                    return
                ip = addrs[0]
                port = int(info.port or 0)
                if not port:
                    return
                lowered = f"{name} {service_type}".lower()
                looks_camera = any(x in lowered for x in ("camera", "cam", "ipcam", "rtsp", "mjpeg", "onvif", "webcam"))
                if not looks_camera:
                    return
                proto = "rtsp" if "_rtsp" in service_type.lower() else "mjpeg"
                url = f"{'rtsp' if proto == 'rtsp' else 'http'}://{ip}:{port}/"
                key = (ip, port, proto)
                if key in seen:
                    return
                seen.add(key)
                found.append(
                    {
                        "ip": ip,
                        "port": port,
                        "url": url,
                        "protocol": proto,
                        "model": name,
                        "source": "mdns",
                        "type": "network",
                    }
                )
            except Exception:
                return

        def update_service(self, zc, service_type, name):
            return

        def remove_service(self, zc, service_type, name):
            return

    zc = Zeroconf()
    listener = _Listener()
    browsers = []
    for svc in ("_rtsp._tcp.local.", "_http._tcp.local.", "_onvif._tcp.local."):
        try:
            browsers.append(ServiceBrowser(zc, svc, listener))
        except Exception:
            continue
    await asyncio.sleep(max(0.5, float(timeout_s)))
    for b in browsers:
        try:
            b.cancel()
        except Exception:
            pass
    try:
        zc.close()
    except Exception:
        pass
    return found
