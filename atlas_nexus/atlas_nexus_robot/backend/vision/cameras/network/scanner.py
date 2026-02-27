"""
Escaneo de red local - obtiene rango IP y escanea puertos de cámara.
Usa la red detectada por Windows (interfaces de red).
"""

import socket
import subprocess
import sys
import logging
from typing import List, Tuple
from concurrent.futures import ThreadPoolExecutor, as_completed

logger = logging.getLogger(__name__)

CAMERA_PORTS = [554, 80, 8080, 8081, 8000, 443]
CONNECT_TIMEOUT = 2


def get_local_network() -> List[Tuple[str, str]]:
    """
    Obtiene redes locales (ip, máscara) desde interfaces de Windows.
    Retorna lista de (ip_base, cidr o máscara) para escanear.
    """
    results = []
    try:
        if sys.platform == "win32":
            ps = subprocess.run(
                ["powershell", "-NoProfile", "-Command",
                 "Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -notlike '127.*' } | "
                 "Select-Object IPAddress, PrefixLength | ConvertTo-Json -Compress"],
                capture_output=True, text=True, timeout=5
            )
            if ps.returncode == 0 and ps.stdout:
                data = __import__("json").loads(ps.stdout)
                if isinstance(data, dict):
                    data = [data]
                for item in data:
                    ip = (item.get("IPAddress") or "").strip()
                    prefix = item.get("PrefixLength", 24)
                    if ip and not ip.startswith("127."):
                        base = _ip_to_base(ip, prefix)
                        results.append((base, prefix))
        else:
            import netifaces
            for iface in netifaces.interfaces():
                addrs = netifaces.ifaddresses(iface)
                if netifaces.AF_INET in addrs:
                    for a in addrs[netifaces.AF_INET]:
                        ip = a.get("addr", "")
                        if ip and not ip.startswith("127."):
                            mask = a.get("netmask", "255.255.255.0")
                            base = _ip_network_base(ip, mask)
                            results.append((base, mask))
    except Exception as e:
        logger.debug("get_local_network fallback: %s", e)
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


def _ip_to_base(ip: str, prefix: int) -> str:
    """Convierte IP y prefix a base de red."""
    parts = ip.split(".")
    if len(parts) != 4:
        return ip
    mask_bits = int(prefix) if isinstance(prefix, (int, float)) else 24
    if mask_bits >= 24:
        return ".".join(parts[:3])
    if mask_bits >= 16:
        return ".".join(parts[:2])
    return parts[0]


def _ip_network_base(ip: str, mask: str) -> str:
    """Base de red a partir de IP y máscara."""
    try:
        ip_parts = [int(x) for x in ip.split(".")]
        mask_parts = [int(x) for x in mask.split(".")]
        base = ".".join(str(ip_parts[i] & mask_parts[i]) for i in range(4))
        if mask == "255.255.255.0":
            return ".".join(base.split(".")[:3])
        return base
    except Exception:
        return ".".join(ip.split(".")[:3])


def _ip_range(base: str, prefix: int) -> List[str]:
    """Genera lista de IPs a escanear."""
    ips = []
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


def check_port_open(host: str, port: int) -> bool:
    """Comprueba si un puerto está abierto."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(CONNECT_TIMEOUT)
        r = sock.connect_ex((host, port))
        sock.close()
        return r == 0
    except Exception:
        return False


def scan_network_for_camera_ports() -> List[Tuple[str, int]]:
    """
    Escanea la red local buscando puertos de cámara (554, 80, 8080, etc.).
    Retorna lista de (ip, port) que responden.
    """
    networks = get_local_network()
    candidates = []
    seen = set()

    def scan_ip(ip_port):
        ip, port = ip_port
        key = (ip, port)
        if key in seen:
            return None
        if check_port_open(ip, port):
            seen.add(key)
            return (ip, port)
        return None

    ip_port_list = []
    for base, prefix in networks:
        for ip in _ip_range(base, prefix):
            for port in CAMERA_PORTS:
                ip_port_list.append((ip, port))

    with ThreadPoolExecutor(max_workers=50) as ex:
        futures = {ex.submit(scan_ip, item): item for item in ip_port_list}
        for f in as_completed(futures):
            try:
                r = f.result()
                if r:
                    candidates.append(r)
            except Exception:
                pass

    return candidates
