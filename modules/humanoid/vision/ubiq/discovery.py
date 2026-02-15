from __future__ import annotations

import hashlib
import os
import socket
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Any, Dict, List, Optional, Tuple

from .onvif_wsdiscovery import discover_onvif_devices
from .registry import upsert_camera


CAMERA_PORTS = [554, 80, 8080, 8081, 8000, 443]
CONNECT_TIMEOUT = float(os.getenv("VISION_DISCOVERY_TIMEOUT", "1.5"))
MAX_WORKERS = int(os.getenv("VISION_DISCOVERY_WORKERS", "60"))


# Paths RTSP típicos de dashcams y cámaras duales (frontal/trasera)
DASHCAM_RTSP_PATHS: List[Tuple[str, str]] = [
    ("stream1", "Dashcam frontal"),
    ("stream2", "Dashcam trasera"),
    ("livestream/11", "70mai principal"),
    ("livestream/12", "70mai secundario"),
    ("cam/realmonitor?channel=1&subtype=1", "Canal 1 frontal"),
    ("cam/realmonitor?channel=2&subtype=1", "Canal 2 trasera"),
    ("live/ch00_0", "Hikvision principal"),
    ("live/ch01_0", "Hikvision secundario"),
]


def _ip_to_base(ip: str, prefix: int) -> str:
    parts = ip.split(".")
    if len(parts) != 4:
        return ip
    mask_bits = int(prefix) if isinstance(prefix, (int, float)) else 24
    if mask_bits >= 24:
        return ".".join(parts[:3])
    if mask_bits >= 16:
        return ".".join(parts[:2])
    return parts[0]


def _ip_range(base: str, prefix: int) -> List[str]:
    ips: List[str] = []
    parts = base.split(".")
    if len(parts) == 3:
        for i in range(1, 255):
            ips.append(f"{base}.{i}")
    elif len(parts) == 4:
        ips.append(base)
    else:
        for a in range(256):
            for b in range(256):
                ips.append(f"{base}.{a}.{b}")
                if len(ips) >= 254:
                    return ips[:254]
    return ips[:254]


def get_local_networks() -> List[Tuple[str, int]]:
    """
    Retorna lista de bases de red para escaneo: (base, prefix).
    """
    results: List[Tuple[str, int]] = []
    try:
        if sys.platform == "win32":
            ps = subprocess.run(
                [
                    "powershell",
                    "-NoProfile",
                    "-Command",
                    "Get-NetIPAddress -AddressFamily IPv4 | "
                    "Where-Object { $_.IPAddress -notlike '127.*' } | "
                    "Select-Object IPAddress, PrefixLength | ConvertTo-Json -Compress",
                ],
                capture_output=True,
                text=True,
                timeout=6,
            )
            if ps.returncode == 0 and ps.stdout:
                import json

                data = json.loads(ps.stdout)
                if isinstance(data, dict):
                    data = [data]
                for item in data:
                    ip = (item.get("IPAddress") or "").strip()
                    prefix = int(item.get("PrefixLength", 24) or 24)
                    if ip and not ip.startswith("127."):
                        base = _ip_to_base(ip, prefix)
                        results.append((base, prefix))
    except Exception:
        pass

    if not results:
        # fallback simple
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            base = ".".join(ip.split(".")[:-1])
            results.append((base, 24))
        except Exception:
            results.append(("192.168.1", 24))
    return results[:3]


def check_port_open(host: str, port: int) -> bool:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(CONNECT_TIMEOUT)
        r = sock.connect_ex((host, int(port)))
        sock.close()
        return r == 0
    except Exception:
        return False


def _probe_rtsp_path(ip: str, port: int, path: str = "/") -> bool:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        sock.connect((ip, int(port)))
        req_path = path if path.startswith("/") else f"/{path}"
        sock.send(
            f"OPTIONS rtsp://{ip}:{int(port)}{req_path} RTSP/1.0\r\nCSeq: 1\r\n\r\n".encode("utf-8")
        )
        data = sock.recv(512).decode("utf-8", errors="ignore")
        sock.close()
        return "RTSP" in data.upper()
    except Exception:
        return False


def _probe_dashcam_streams(ip: str, port: int) -> List[Dict[str, Any]]:
    found: List[Dict[str, Any]] = []
    for path, label in DASHCAM_RTSP_PATHS:
        path_norm = path if path.startswith("/") else f"/{path}"
        if _probe_rtsp_path(ip, port, path_norm):
            url = f"rtsp://{ip}:{port}{path_norm}"
            safe_path = path.replace("/", "_").replace("?", "_").replace("&", "_").replace("=", "_")[:25]
            cam_id = f"auto_{ip.replace('.', '_')}_{port}_{safe_path}"
            found.append(
                {
                    "id": cam_id,
                    "ip": ip,
                    "port": int(port),
                    "protocol": "rtsp",
                    "url": url,
                    "model": f"{label} ({ip})",
                    "source": "network",
                    "type": "auto",
                }
            )
    return found


def _mk_id_from_url(url: str) -> str:
    h = hashlib.md5((url or "").encode("utf-8", errors="ignore")).hexdigest()[:12]
    return f"net_{h}"


def discover_local_cameras(
    scan_ports: bool = True,
    onvif: bool = True,
) -> Dict[str, Any]:
    """
    Descubrimiento:
    - RTSP/MJPEG: escaneo de puertos en subredes locales (Windows).
    - ONVIF: WS-Discovery.
    Guarda/actualiza en SQLite y retorna resumen.
    """
    found: List[Dict[str, Any]] = []
    new_count = 0
    seen_ids = set()

    # 1) ONVIF (rápido, multicast)
    if onvif:
        try:
            devs = discover_onvif_devices(timeout_s=2.0, max_results=60)
            for d in devs:
                ip = d.get("ip", "")
                xaddr = d.get("xaddr", "")
                url = xaddr or (f"http://{ip}/onvif/device_service" if ip else "")
                cid = f"onvif_{ip.replace('.', '_')}" if ip else _mk_id_from_url(url)
                cam = {
                    "id": cid,
                    "ip": ip,
                    "port": 80,
                    "protocol": "onvif",
                    "url": url,
                    "model": f"ONVIF device ({ip})",
                    "source": "network",
                    "type": "network",
                    "onvif_xaddr": xaddr,
                    "meta": {"endpoint": d.get("endpoint", ""), "discovery": "ws-discovery"},
                }
                if cam["id"] in seen_ids:
                    continue
                seen_ids.add(cam["id"])
                found.append(cam)
        except Exception:
            pass

    # 2) RTSP/MJPEG scan (más pesado)
    candidates: List[Tuple[str, int]] = []
    if scan_ports:
        networks = get_local_networks()
        ip_port_list: List[Tuple[str, int]] = []
        for base, prefix in networks:
            for ip in _ip_range(base, prefix):
                for port in CAMERA_PORTS:
                    ip_port_list.append((ip, port))

        def scan_one(item: Tuple[str, int]) -> Optional[Tuple[str, int]]:
            ip, port = item
            if check_port_open(ip, port):
                return (ip, int(port))
            return None

        with ThreadPoolExecutor(max_workers=max(10, int(MAX_WORKERS))) as ex:
            futs = [ex.submit(scan_one, it) for it in ip_port_list]
            for f in as_completed(futs):
                try:
                    r = f.result()
                    if r:
                        candidates.append(r)
                except Exception:
                    continue

        for ip, port in candidates:
            if int(port) == 554:
                dashcams = _probe_dashcam_streams(ip, port)
                if dashcams:
                    for d in dashcams:
                        if d["id"] not in seen_ids:
                            seen_ids.add(d["id"])
                            found.append(d)
                else:
                    if _probe_rtsp_path(ip, port, "/"):
                        url = f"rtsp://{ip}:{port}/"
                        cid = f"rtsp_{ip.replace('.', '_')}_{port}"
                        if cid not in seen_ids:
                            seen_ids.add(cid)
                            found.append(
                                {
                                    "id": cid,
                                    "ip": ip,
                                    "port": int(port),
                                    "protocol": "rtsp",
                                    "url": url,
                                    "model": f"RTSP camera ({ip})",
                                    "source": "network",
                                    "type": "network",
                                }
                            )
            else:
                # HTTP MJPEG o RTSP en puerto no estándar: registramos como candidato (sin probe profunda)
                proto = "http"
                url = f"http://{ip}:{port}/"
                cid = f"lan_{ip.replace('.', '_')}_{port}"
                if cid not in seen_ids:
                    seen_ids.add(cid)
                    found.append(
                        {
                            "id": cid,
                            "ip": ip,
                            "port": int(port),
                            "protocol": proto,
                            "url": url,
                            "model": f"IP candidate ({ip}:{port})",
                            "source": "network",
                            "type": "network",
                        }
                    )

    # Persist + ANS logging para nuevos
    for cam in found:
        try:
            is_new = upsert_camera(cam)
            if is_new:
                new_count += 1
                ip = (cam.get("ip") or cam.get("url") or "").split("/")[0]
                try:
                    from modules.humanoid.ans.evolution_bitacora import append_evolution_log

                    append_evolution_log(
                        f"[VISION] Nuevo nodo de video asimilado: {cam.get('ip') or cam.get('url')}",
                        ok=True,
                        source="vision",
                    )
                except Exception:
                    pass
        except Exception:
            continue

    return {
        "ok": True,
        "found": found,
        "found_count": len(found),
        "new_count": new_count,
        "candidates_count": len(candidates),
        "ts": time.time(),
    }

