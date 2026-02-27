from __future__ import annotations

import socket
import time
import uuid
from typing import Dict, List, Optional


_MCAST_ADDR = ("239.255.255.250", 3702)


def _probe_message(message_id: str) -> bytes:
    # WS-Discovery Probe (SOAP). Compatible con la mayoría de cámaras ONVIF.
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<e:Envelope xmlns:e="http://www.w3.org/2003/05/soap-envelope"
            xmlns:w="http://schemas.xmlsoap.org/ws/2004/08/addressing"
            xmlns:d="http://schemas.xmlsoap.org/ws/2005/04/discovery"
            xmlns:dn="http://www.onvif.org/ver10/network/wsdl">
  <e:Header>
    <w:MessageID>uuid:{message_id}</w:MessageID>
    <w:To>urn:schemas-xmlsoap-org:ws:2005:04:discovery</w:To>
    <w:Action>http://schemas.xmlsoap.org/ws/2005/04/discovery/Probe</w:Action>
  </e:Header>
  <e:Body>
    <d:Probe>
      <d:Types>dn:NetworkVideoTransmitter</d:Types>
    </d:Probe>
  </e:Body>
</e:Envelope>
""".encode("utf-8")


def _extract_between(text: str, a: str, b: str) -> Optional[str]:
    try:
        i = text.find(a)
        if i < 0:
            return None
        i += len(a)
        j = text.find(b, i)
        if j < 0:
            return None
        return text[i:j].strip()
    except Exception:
        return None


def discover_onvif_devices(timeout_s: float = 2.0, max_results: int = 50) -> List[Dict[str, str]]:
    """
    Descubre dispositivos ONVIF en la LAN usando WS-Discovery (UDP multicast).
    Retorna lista de dicts: {ip, xaddr, endpoint, raw?}
    """
    message_id = str(uuid.uuid4())
    payload = _probe_message(message_id)
    out: List[Dict[str, str]] = []
    seen = set()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(0.25)
        # TTL 2: LAN
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        sock.sendto(payload, _MCAST_ADDR)

        end = time.time() + float(timeout_s)
        while time.time() < end and len(out) < int(max_results):
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except Exception:
                break
            ip = str(addr[0])
            raw = data.decode("utf-8", errors="ignore")
            xaddrs = _extract_between(raw, "<d:XAddrs>", "</d:XAddrs>") or _extract_between(raw, "<XAddrs>", "</XAddrs>")
            endpoint = _extract_between(raw, "<w:Address>", "</w:Address>") or _extract_between(raw, "<Address>", "</Address>")
            xaddr = ""
            if xaddrs:
                # puede venir como lista separada por espacios
                xaddr = xaddrs.split()[0].strip()
            key = (ip, xaddr or "", endpoint or "")
            if key in seen:
                continue
            seen.add(key)
            out.append(
                {
                    "ip": ip,
                    "xaddr": xaddr,
                    "endpoint": endpoint or "",
                    "raw_hint": "ws-discovery",
                }
            )
    finally:
        try:
            sock.close()
        except Exception:
            pass
    return out

