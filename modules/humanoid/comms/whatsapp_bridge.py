"""WhatsApp Bridge - Comunicación por WhatsApp multi-proveedor.

Soporta múltiples proveedores:
1. WAHA (WhatsApp HTTP API) - Gratuito, self-hosted, recomendado
2. Twilio - De pago, fácil configuración
3. Meta WhatsApp Business API - Gratuito bajo volumen

Variables de entorno:
- WHATSAPP_ENABLED=true          # Habilita WhatsApp
- WHATSAPP_PROVIDER=waha         # waha | twilio | meta
- WHATSAPP_TO=+34612345678       # Número destino (tu número personal)

Para WAHA (recomendado):
- WAHA_API_URL=http://localhost:3000  # URL del servidor WAHA
- WAHA_SESSION=default                 # Nombre de sesión

Para Twilio:
- TWILIO_ACCOUNT_SID=...
- TWILIO_AUTH_TOKEN=...
- TWILIO_WHATSAPP_FROM=whatsapp:+14155238886

Para Meta:
- META_WHATSAPP_TOKEN=...
- META_PHONE_NUMBER_ID=...
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

logger = logging.getLogger("atlas.comms.whatsapp")


def _get(name: str, default: str = "") -> str:
    """Obtiene variable de entorno, cargando Bóveda si es necesario."""
    try:
        from modules.humanoid.config.vault import load_vault_env
        load_vault_env(override=False)
    except Exception:
        pass
    return (os.getenv(name) or default).strip()


def _bool(name: str, default: bool = False) -> bool:
    """Obtiene variable de entorno como booleano."""
    v = _get(name).lower()
    if not v:
        return default
    return v in ("1", "true", "yes", "on")


@dataclass
class WhatsAppMessage:
    """Mensaje de WhatsApp."""
    to: str
    text: str
    message_id: Optional[str] = None
    timestamp: Optional[str] = None
    status: str = "pending"  # pending, sent, delivered, read, failed


class WAHAProvider:
    """Proveedor WAHA (WhatsApp HTTP API) - Gratuito y self-hosted.
    
    WAHA es un servidor Docker que expone la API de WhatsApp Web.
    Requiere escanear código QR una vez para autenticar.
    
    Instalación (sin autenticación para desarrollo):
        docker run -d -p 3000:3000 --name waha \\
            -e WHATSAPP_API_KEY=atlas123 \\
            devlikeapro/waha
    
    Luego visita http://localhost:3000 para escanear el QR.
    """
    
    def __init__(self):
        self.api_url = _get("WAHA_API_URL", "http://localhost:3010")
        self.session = _get("WAHA_SESSION", "default")
        self.api_key = _get("WAHA_API_KEY", "")
    
    def _headers(self) -> Dict[str, str]:
        """Obtiene los headers para las peticiones."""
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["X-Api-Key"] = self.api_key
        return headers
    
    def is_configured(self) -> bool:
        """Verifica si WAHA está configurado."""
        return bool(self.api_url)
    
    def get_status(self) -> Dict[str, Any]:
        """Obtiene el estado de la sesión WAHA."""
        try:
            url = f"{self.api_url}/api/sessions/{self.session}"
            req = urllib.request.Request(url, method="GET", headers=self._headers())
            with urllib.request.urlopen(req, timeout=5) as r:
                data = json.loads(r.read().decode())
            return {
                "ok": True,
                "status": data.get("status"),
                "name": data.get("name"),
                "authenticated": data.get("status") == "WORKING",
            }
        except urllib.error.HTTPError as e:
            if e.code == 404:
                return {"ok": False, "status": "not_found", "authenticated": False}
            if e.code == 401:
                return {"ok": False, "status": "unauthorized", "authenticated": False, 
                        "hint": "Configura WAHA_API_KEY o reinicia WAHA sin autenticación"}
            return {"ok": False, "error": f"HTTP {e.code}", "authenticated": False}
        except Exception as e:
            return {"ok": False, "error": str(e), "authenticated": False}
    
    def start_session(self) -> Dict[str, Any]:
        """Inicia una sesión WAHA (genera QR si es necesario)."""
        try:
            url = f"{self.api_url}/api/sessions/{self.session}/start"
            req = urllib.request.Request(
                url,
                data=json.dumps({}).encode(),
                method="POST",
                headers=self._headers(),
            )
            with urllib.request.urlopen(req, timeout=10) as r:
                data = json.loads(r.read().decode())
            return {"ok": True, "data": data}
        except urllib.error.HTTPError as e:
            if e.code == 401:
                return {"ok": False, "error": "unauthorized", 
                        "hint": "Configura WAHA_API_KEY"}
            return {"ok": False, "error": f"HTTP {e.code}"}
        except Exception as e:
            return {"ok": False, "error": str(e)}
    
    def get_qr(self) -> Dict[str, Any]:
        """Obtiene el código QR para autenticar."""
        try:
            url = f"{self.api_url}/api/sessions/{self.session}/auth/qr"
            req = urllib.request.Request(url, method="GET", headers=self._headers())
            with urllib.request.urlopen(req, timeout=10) as r:
                data = json.loads(r.read().decode())
            return {
                "ok": True,
                "qr": data.get("value"),
                "format": data.get("format", "image"),
            }
        except urllib.error.HTTPError as e:
            if e.code == 401:
                return {"ok": False, "error": "unauthorized"}
            return {"ok": False, "error": f"HTTP {e.code}"}
        except Exception as e:
            return {"ok": False, "error": str(e)}
    
    def send_message(self, to: str, text: str) -> Dict[str, Any]:
        """Envía un mensaje de texto."""
        # Formatear número (debe ser chatId: número@c.us)
        phone = to.replace("+", "").replace(" ", "").replace("-", "")
        chat_id = f"{phone}@c.us"
        
        try:
            url = f"{self.api_url}/api/sendText"
            payload = {
                "session": self.session,
                "chatId": chat_id,
                "text": text[:4096],
            }
            req = urllib.request.Request(
                url,
                data=json.dumps(payload).encode(),
                method="POST",
                headers=self._headers(),
            )
            with urllib.request.urlopen(req, timeout=15) as r:
                data = json.loads(r.read().decode())
            return {
                "ok": True,
                "message_id": data.get("id"),
                "timestamp": datetime.now(timezone.utc).isoformat(),
            }
        except urllib.error.HTTPError as e:
            body = e.read().decode() if e.fp else ""
            if e.code == 401:
                return {"ok": False, "error": "unauthorized", "hint": "Configura WAHA_API_KEY"}
            return {"ok": False, "error": f"HTTP {e.code}: {body[:200]}"}
        except Exception as e:
            return {"ok": False, "error": str(e)}
    
    def send_image(self, to: str, image_url: str, caption: str = "") -> Dict[str, Any]:
        """Envía una imagen."""
        phone = to.replace("+", "").replace(" ", "").replace("-", "")
        chat_id = f"{phone}@c.us"
        
        try:
            url = f"{self.api_url}/api/sendImage"
            payload = {
                "session": self.session,
                "chatId": chat_id,
                "file": {"url": image_url},
                "caption": caption[:1024],
            }
            req = urllib.request.Request(
                url,
                data=json.dumps(payload).encode(),
                method="POST",
                headers=self._headers(),
            )
            with urllib.request.urlopen(req, timeout=30) as r:
                data = json.loads(r.read().decode())
            return {"ok": True, "message_id": data.get("id")}
        except Exception as e:
            return {"ok": False, "error": str(e)}


class TwilioProvider:
    """Proveedor Twilio para WhatsApp."""
    
    def __init__(self):
        self.account_sid = _get("TWILIO_ACCOUNT_SID")
        self.auth_token = _get("TWILIO_AUTH_TOKEN")
        self.from_number = _get("TWILIO_WHATSAPP_FROM")
    
    def is_configured(self) -> bool:
        return bool(self.account_sid and self.auth_token and self.from_number)
    
    def get_status(self) -> Dict[str, Any]:
        return {
            "ok": self.is_configured(),
            "authenticated": self.is_configured(),
            "status": "configured" if self.is_configured() else "not_configured",
        }
    
    def send_message(self, to: str, text: str) -> Dict[str, Any]:
        if not self.is_configured():
            return {"ok": False, "error": "Twilio not configured"}
        
        try:
            import base64
            
            # Formatear número para Twilio
            phone = to.replace(" ", "").replace("-", "")
            if not phone.startswith("+"):
                phone = f"+{phone}"
            to_whatsapp = f"whatsapp:{phone}"
            
            url = f"https://api.twilio.com/2010-04-01/Accounts/{self.account_sid}/Messages.json"
            data = urllib.parse.urlencode({
                "From": self.from_number,
                "To": to_whatsapp,
                "Body": text[:1500],
            }).encode()
            
            auth = base64.b64encode(
                f"{self.account_sid}:{self.auth_token}".encode()
            ).decode("ascii")
            
            req = urllib.request.Request(
                url,
                data=data,
                method="POST",
                headers={"Authorization": f"Basic {auth}"},
            )
            
            with urllib.request.urlopen(req, timeout=20) as r:
                result = json.loads(r.read().decode())
            
            return {
                "ok": True,
                "message_id": result.get("sid"),
                "timestamp": datetime.now(timezone.utc).isoformat(),
            }
        except Exception as e:
            return {"ok": False, "error": str(e)}


class MetaProvider:
    """Proveedor Meta WhatsApp Business API."""
    
    def __init__(self):
        self.token = _get("META_WHATSAPP_TOKEN")
        self.phone_id = _get("META_PHONE_NUMBER_ID")
        self.api_version = _get("META_API_VERSION", "v18.0")
    
    def is_configured(self) -> bool:
        return bool(self.token and self.phone_id)
    
    def get_status(self) -> Dict[str, Any]:
        return {
            "ok": self.is_configured(),
            "authenticated": self.is_configured(),
            "status": "configured" if self.is_configured() else "not_configured",
        }
    
    def send_message(self, to: str, text: str) -> Dict[str, Any]:
        if not self.is_configured():
            return {"ok": False, "error": "Meta not configured"}
        
        try:
            phone = to.replace("+", "").replace(" ", "").replace("-", "")
            
            url = f"https://graph.facebook.com/{self.api_version}/{self.phone_id}/messages"
            payload = {
                "messaging_product": "whatsapp",
                "to": phone,
                "type": "text",
                "text": {"body": text[:4096]},
            }
            
            req = urllib.request.Request(
                url,
                data=json.dumps(payload).encode(),
                method="POST",
                headers={
                    "Content-Type": "application/json",
                    "Authorization": f"Bearer {self.token}",
                },
            )
            
            with urllib.request.urlopen(req, timeout=15) as r:
                result = json.loads(r.read().decode())
            
            return {
                "ok": True,
                "message_id": result.get("messages", [{}])[0].get("id"),
                "timestamp": datetime.now(timezone.utc).isoformat(),
            }
        except Exception as e:
            return {"ok": False, "error": str(e)}


# === Singleton y funciones de conveniencia ===

_provider_instance: Optional[Any] = None
_provider_lock = threading.Lock()


def _get_provider():
    """Obtiene el proveedor configurado."""
    global _provider_instance
    
    if _provider_instance is not None:
        return _provider_instance
    
    with _provider_lock:
        if _provider_instance is not None:
            return _provider_instance
        
        provider_name = _get("WHATSAPP_PROVIDER", "waha").lower()
        
        if provider_name == "twilio":
            _provider_instance = TwilioProvider()
        elif provider_name == "meta":
            _provider_instance = MetaProvider()
        else:
            _provider_instance = WAHAProvider()
        
        return _provider_instance


def status() -> Dict[str, Any]:
    """Obtiene el estado del servicio WhatsApp."""
    enabled = _bool("WHATSAPP_ENABLED") or _bool("OPS_WHATSAPP_ENABLED")
    provider_name = _get("WHATSAPP_PROVIDER", "waha")
    to_number = _get("WHATSAPP_TO") or _get("TWILIO_WHATSAPP_TO")
    
    provider = _get_provider()
    provider_status = provider.get_status() if provider else {"ok": False}
    
    missing = []
    if not to_number:
        missing.append("WHATSAPP_TO")
    if not provider.is_configured():
        if provider_name == "waha":
            missing.append("WAHA_API_URL (opcional, default: localhost:3000)")
        elif provider_name == "twilio":
            missing.extend(["TWILIO_ACCOUNT_SID", "TWILIO_AUTH_TOKEN", "TWILIO_WHATSAPP_FROM"])
        elif provider_name == "meta":
            missing.extend(["META_WHATSAPP_TOKEN", "META_PHONE_NUMBER_ID"])
    
    return {
        "enabled": enabled,
        "provider": provider_name,
        "provider_status": provider_status,
        "to_number": to_number[:6] + "****" if to_number and len(to_number) > 6 else to_number,
        "missing": missing,
        "ready": enabled and provider_status.get("authenticated", False) and bool(to_number),
    }


def send_text(text: str, to: Optional[str] = None) -> Dict[str, Any]:
    """Envía un mensaje de texto por WhatsApp.
    
    Args:
        text: Mensaje a enviar
        to: Número destino (opcional, usa WHATSAPP_TO si no se especifica)
    
    Returns:
        Dict con resultado del envío
    """
    st = status()
    if not st.get("enabled"):
        return {"ok": False, "error": "whatsapp_disabled", "details": st}
    
    to_number = to or _get("WHATSAPP_TO") or _get("TWILIO_WHATSAPP_TO")
    if not to_number:
        return {"ok": False, "error": "no_destination_number", "details": st}
    
    provider = _get_provider()
    if not provider:
        return {"ok": False, "error": "no_provider"}
    
    if not provider.is_configured():
        return {"ok": False, "error": "provider_not_configured", "details": st}
    
    # Para WAHA, verificar que esté autenticado
    if isinstance(provider, WAHAProvider):
        pst = provider.get_status()
        if not pst.get("authenticated"):
            return {
                "ok": False,
                "error": "waha_not_authenticated",
                "hint": "Visita http://localhost:3000 para escanear el QR",
                "details": pst,
            }
    
    result = provider.send_message(to_number, text)
    
    if result.get("ok"):
        logger.info(f"WhatsApp enviado a {to_number[:6]}****: {text[:50]}...")
    else:
        logger.warning(f"WhatsApp falló: {result.get('error')}")
    
    return result


def send_image(image_url: str, caption: str = "", to: Optional[str] = None) -> Dict[str, Any]:
    """Envía una imagen por WhatsApp (solo WAHA)."""
    provider = _get_provider()
    
    if not isinstance(provider, WAHAProvider):
        return {"ok": False, "error": "send_image only supported with WAHA provider"}
    
    to_number = to or _get("WHATSAPP_TO")
    if not to_number:
        return {"ok": False, "error": "no_destination_number"}
    
    return provider.send_image(to_number, image_url, caption)


def get_qr_code() -> Dict[str, Any]:
    """Obtiene el código QR para autenticar WAHA."""
    provider = _get_provider()
    
    if not isinstance(provider, WAHAProvider):
        return {"ok": False, "error": "QR only available for WAHA provider"}
    
    return provider.get_qr()


def start_session() -> Dict[str, Any]:
    """Inicia la sesión WAHA."""
    provider = _get_provider()
    
    if not isinstance(provider, WAHAProvider):
        return {"ok": False, "error": "Sessions only available for WAHA provider"}
    
    return provider.start_session()


def health_check() -> Dict[str, Any]:
    """Health check del servicio WhatsApp."""
    st = status()
    return {
        "ok": st.get("ready", False),
        "enabled": st.get("enabled", False),
        "provider": st.get("provider"),
        "authenticated": st.get("provider_status", {}).get("authenticated", False),
        "to_configured": bool(_get("WHATSAPP_TO")),
    }


# === Compatibilidad con código existente ===

def is_enabled() -> bool:
    """Verifica si WhatsApp está habilitado."""
    return _bool("WHATSAPP_ENABLED") or _bool("OPS_WHATSAPP_ENABLED")

