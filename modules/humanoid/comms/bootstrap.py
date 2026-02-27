"""Bootstrap: inicialización centralizada de todos los servicios de comunicación.

Este módulo se encarga de:
1. Cargar configuración desde la Bóveda
2. Validar credenciales de todos los canales
3. Iniciar servicios de polling/webhooks
4. Registrar health checks
5. Reportar estado inicial

Uso:
    from modules.humanoid.comms.bootstrap import bootstrap_comms, get_status
    
    # Al iniciar la aplicación:
    result = bootstrap_comms()
    if not result["ok"]:
        print(f"Warnings: {result['warnings']}")
"""
from __future__ import annotations

import logging
import os
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger("atlas.comms.bootstrap")


@dataclass
class ServiceStatus:
    """Estado de un servicio de comunicación."""
    name: str
    enabled: bool = False
    initialized: bool = False
    healthy: bool = False
    error: str = ""
    config_valid: bool = False
    started_at: Optional[str] = None


@dataclass
class BootstrapResult:
    """Resultado del proceso de bootstrap."""
    ok: bool
    services: Dict[str, ServiceStatus] = field(default_factory=dict)
    warnings: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    duration_ms: float = 0.0


# Estado global de servicios
_services: Dict[str, ServiceStatus] = {}
_initialized = False
_lock = threading.Lock()


def _load_vault() -> bool:
    """Carga variables de entorno desde la Bóveda."""
    try:
        from modules.humanoid.config.vault import load_vault_env
        load_vault_env(override=False)
        return True
    except Exception as e:
        logger.warning(f"No se pudo cargar la Bóveda: {e}")
        return False


def _check_telegram_config() -> tuple[bool, str]:
    """Verifica la configuración de Telegram."""
    token = (os.getenv("TELEGRAM_BOT_TOKEN") or os.getenv("TELEGRAM_TOKEN") or "").strip()
    if not token:
        return False, "TELEGRAM_BOT_TOKEN no configurado"
    
    chat_ids = (
        os.getenv("TELEGRAM_ALLOWED_CHAT_IDS") or 
        os.getenv("TELEGRAM_CHAT_ID") or 
        os.getenv("TELEGRAM_ADMIN_CHAT_ID") or 
        ""
    ).strip()
    
    if not chat_ids:
        return False, "TELEGRAM_CHAT_ID no configurado (se intentará auto-descubrir)"
    
    return True, ""


def _check_whatsapp_config() -> tuple[bool, str]:
    """Verifica la configuración de WhatsApp/Twilio."""
    required = ["TWILIO_ACCOUNT_SID", "TWILIO_AUTH_TOKEN", "TWILIO_WHATSAPP_FROM", "TWILIO_WHATSAPP_TO"]
    missing = [var for var in required if not os.getenv(var)]
    
    if missing:
        return False, f"Faltan variables: {', '.join(missing)}"
    
    return True, ""


def _check_webhook_config() -> tuple[bool, str]:
    """Verifica la configuración de webhooks."""
    url = (os.getenv("MAKEPLAY_WEBHOOK_URL") or os.getenv("EXTERNAL_WEBHOOK_URL") or "").strip()
    
    if not url:
        return False, "No hay URL de webhook configurada"
    
    if not url.startswith("http"):
        return False, f"URL de webhook inválida: {url[:50]}"
    
    return True, ""


def _check_audio_config() -> tuple[bool, str]:
    """Verifica la disponibilidad del sistema de audio/TTS."""
    try:
        from modules.humanoid.voice import tts
        if tts.is_available():
            return True, ""
        missing = tts.get_missing_deps()
        return False, f"TTS no disponible. Dependencias faltantes: {missing}"
    except Exception as e:
        return False, f"Error verificando TTS: {e}"


def _init_telegram_poller() -> tuple[bool, str]:
    """Inicializa el polling de Telegram."""
    try:
        enabled = os.getenv("TELEGRAM_POLLING_ENABLED", "true").strip().lower()
        if enabled not in ("1", "true", "yes"):
            return True, "Polling deshabilitado por configuración"
        
        from modules.humanoid.comms.telegram_poller import start_polling
        started = start_polling()
        
        if started:
            return True, ""
        return False, "No se pudo iniciar el polling (token o chat_id faltantes)"
    except Exception as e:
        return False, f"Error iniciando poller: {e}"


def _init_makeplay_scheduler() -> tuple[bool, str]:
    """Inicializa el scheduler de MakePlay."""
    try:
        from modules.humanoid.comms.makeplay_scheduler import ensure_makeplay_jobs
        ensure_makeplay_jobs()
        return True, ""
    except Exception as e:
        return False, f"Error iniciando scheduler MakePlay: {e}"


def _init_comms_hub() -> tuple[bool, str]:
    """Inicializa el hub central de comunicaciones."""
    try:
        from modules.humanoid.comms.hub import get_hub
        hub = get_hub()
        health = hub.get_health()
        
        if health.get("ok"):
            return True, ""
        return True, f"Hub inicializado con warnings: {health.get('status')}"
    except Exception as e:
        return False, f"Error inicializando CommsHub: {e}"


def _test_telegram_connection() -> tuple[bool, str]:
    """Prueba la conexión con Telegram."""
    try:
        from modules.humanoid.comms.telegram_bridge import TelegramBridge
        
        bridge = TelegramBridge()
        health = bridge.health_check()
        
        if health.get("ok"):
            details = health.get("details", {})
            if details.get("token_set"):
                return True, ""
            return False, "Token no configurado"
        return False, health.get("error", "Health check falló")
    except Exception as e:
        return False, f"Error conectando con Telegram: {e}"


def _test_webhook_connection() -> tuple[bool, str]:
    """Verifica que el webhook esté accesible (no envía datos)."""
    try:
        url = (os.getenv("MAKEPLAY_WEBHOOK_URL") or os.getenv("EXTERNAL_WEBHOOK_URL") or "").strip()
        if not url:
            return True, "No hay webhook configurado"
        
        # Solo verificamos la URL, no hacemos requests de prueba
        if url.startswith("http"):
            return True, ""
        return False, "URL inválida"
    except Exception as e:
        return False, f"Error verificando webhook: {e}"


def bootstrap_comms(
    skip_tests: bool = False,
    services_to_init: Optional[List[str]] = None,
) -> Dict[str, Any]:
    """Inicializa todos los servicios de comunicación.
    
    Args:
        skip_tests: Si True, omite las pruebas de conexión
        services_to_init: Lista de servicios específicos a inicializar (None = todos)
    
    Returns:
        Dict con resultado del bootstrap
    """
    global _initialized, _services
    
    start = time.perf_counter()
    warnings: List[str] = []
    errors: List[str] = []
    
    with _lock:
        # Cargar configuración
        vault_loaded = _load_vault()
        if not vault_loaded:
            warnings.append("Bóveda no cargada - usando variables de entorno directas")
        
        # Definir servicios a inicializar
        all_services = ["hub", "telegram", "telegram_poller", "whatsapp", "webhook", "audio", "makeplay"]
        services = services_to_init or all_services
        
        # 1. Inicializar CommsHub (siempre primero)
        if "hub" in services:
            status = ServiceStatus(name="hub", enabled=True)
            ok, err = _init_comms_hub()
            status.initialized = ok
            status.healthy = ok
            status.error = err
            status.config_valid = True
            if ok:
                status.started_at = datetime.now(timezone.utc).isoformat()
            else:
                errors.append(f"hub: {err}")
            _services["hub"] = status
        
        # 2. Verificar y configurar Telegram
        if "telegram" in services:
            status = ServiceStatus(name="telegram")
            status.enabled = os.getenv("OPS_TELEGRAM_ENABLED", "true").strip().lower() in ("1", "true", "yes")
            
            config_ok, config_err = _check_telegram_config()
            status.config_valid = config_ok
            
            if not config_ok:
                warnings.append(f"telegram: {config_err}")
            
            if not skip_tests and status.enabled:
                test_ok, test_err = _test_telegram_connection()
                status.healthy = test_ok
                if not test_ok:
                    status.error = test_err
                    warnings.append(f"telegram test: {test_err}")
            else:
                status.healthy = config_ok
            
            status.initialized = status.enabled
            if status.initialized:
                status.started_at = datetime.now(timezone.utc).isoformat()
            
            _services["telegram"] = status
        
        # 3. Inicializar Telegram Poller
        if "telegram_poller" in services:
            status = ServiceStatus(name="telegram_poller")
            status.enabled = os.getenv("TELEGRAM_POLLING_ENABLED", "true").strip().lower() in ("1", "true", "yes")
            
            if status.enabled:
                ok, err = _init_telegram_poller()
                status.initialized = ok
                status.healthy = ok
                status.config_valid = ok
                status.error = err
                if ok:
                    status.started_at = datetime.now(timezone.utc).isoformat()
                elif err:
                    warnings.append(f"telegram_poller: {err}")
            
            _services["telegram_poller"] = status
        
        # 4. Verificar WhatsApp
        if "whatsapp" in services:
            status = ServiceStatus(name="whatsapp")
            status.enabled = os.getenv("OPS_WHATSAPP_ENABLED", "false").strip().lower() in ("1", "true", "yes")
            
            if status.enabled:
                config_ok, config_err = _check_whatsapp_config()
                status.config_valid = config_ok
                status.healthy = config_ok
                status.initialized = config_ok
                
                if not config_ok:
                    status.error = config_err
                    warnings.append(f"whatsapp: {config_err}")
                else:
                    status.started_at = datetime.now(timezone.utc).isoformat()
            
            _services["whatsapp"] = status
        
        # 5. Verificar Webhook
        if "webhook" in services:
            status = ServiceStatus(name="webhook")
            status.enabled = os.getenv("MAKEPLAY_ENABLED", "true").strip().lower() in ("1", "true", "yes")
            
            if status.enabled:
                config_ok, config_err = _check_webhook_config()
                status.config_valid = config_ok
                
                if not skip_tests:
                    test_ok, test_err = _test_webhook_connection()
                    status.healthy = test_ok and config_ok
                    if not test_ok:
                        status.error = test_err
                else:
                    status.healthy = config_ok
                
                status.initialized = config_ok
                if config_ok:
                    status.started_at = datetime.now(timezone.utc).isoformat()
                else:
                    warnings.append(f"webhook: {config_err}")
            
            _services["webhook"] = status
        
        # 6. Verificar Audio/TTS
        if "audio" in services:
            status = ServiceStatus(name="audio")
            status.enabled = os.getenv("OPS_AUDIO_ENABLED", "true").strip().lower() in ("1", "true", "yes")
            
            if status.enabled:
                ok, err = _check_audio_config()
                status.config_valid = ok
                status.healthy = ok
                status.initialized = ok
                status.error = err
                
                if ok:
                    status.started_at = datetime.now(timezone.utc).isoformat()
                else:
                    warnings.append(f"audio: {err}")
            
            _services["audio"] = status
        
        # 7. Inicializar MakePlay Scheduler
        if "makeplay" in services:
            status = ServiceStatus(name="makeplay")
            webhook_url = (os.getenv("MAKEPLAY_WEBHOOK_URL") or os.getenv("EXTERNAL_WEBHOOK_URL") or "").strip()
            status.enabled = bool(webhook_url) and os.getenv("MAKEPLAY_ENABLED", "true").strip().lower() in ("1", "true", "yes")
            
            if status.enabled:
                ok, err = _init_makeplay_scheduler()
                status.initialized = ok
                status.healthy = ok
                status.config_valid = ok
                status.error = err
                
                if ok:
                    status.started_at = datetime.now(timezone.utc).isoformat()
                else:
                    warnings.append(f"makeplay: {err}")
            
            _services["makeplay"] = status
        
        _initialized = True
    
    duration_ms = (time.perf_counter() - start) * 1000
    
    # Determinar estado general
    critical_services = ["hub"]
    critical_ok = all(
        _services.get(s, ServiceStatus(name=s)).healthy
        for s in critical_services
        if s in _services
    )
    
    result = {
        "ok": critical_ok and len(errors) == 0,
        "initialized": True,
        "services": {name: _service_to_dict(s) for name, s in _services.items()},
        "warnings": warnings,
        "errors": errors,
        "duration_ms": duration_ms,
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }
    
    # Log resultado
    if result["ok"]:
        logger.info(f"Bootstrap completado en {duration_ms:.0f}ms - {len(_services)} servicios")
    else:
        logger.warning(f"Bootstrap con problemas: {errors + warnings}")
    
    return result


def _service_to_dict(status: ServiceStatus) -> Dict[str, Any]:
    """Convierte ServiceStatus a dict."""
    return {
        "name": status.name,
        "enabled": status.enabled,
        "initialized": status.initialized,
        "healthy": status.healthy,
        "config_valid": status.config_valid,
        "error": status.error or None,
        "started_at": status.started_at,
    }


def get_status() -> Dict[str, Any]:
    """Obtiene el estado actual de todos los servicios."""
    with _lock:
        return {
            "initialized": _initialized,
            "services": {name: _service_to_dict(s) for name, s in _services.items()},
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }


def get_service_status(service_name: str) -> Optional[Dict[str, Any]]:
    """Obtiene el estado de un servicio específico."""
    with _lock:
        status = _services.get(service_name)
        if status:
            return _service_to_dict(status)
    return None


def is_service_healthy(service_name: str) -> bool:
    """Verifica si un servicio está saludable."""
    with _lock:
        status = _services.get(service_name)
        return status.healthy if status else False


def restart_service(service_name: str) -> Dict[str, Any]:
    """Reinicia un servicio específico.
    
    Returns:
        Dict con resultado del reinicio
    """
    service_initializers: Dict[str, Callable[[], tuple[bool, str]]] = {
        "hub": _init_comms_hub,
        "telegram_poller": _init_telegram_poller,
        "makeplay": _init_makeplay_scheduler,
    }
    
    if service_name not in service_initializers:
        return {
            "ok": False,
            "error": f"Servicio '{service_name}' no soporta reinicio o no existe",
        }
    
    with _lock:
        try:
            ok, err = service_initializers[service_name]()
            
            if service_name in _services:
                _services[service_name].initialized = ok
                _services[service_name].healthy = ok
                _services[service_name].error = err
                if ok:
                    _services[service_name].started_at = datetime.now(timezone.utc).isoformat()
            
            return {
                "ok": ok,
                "service": service_name,
                "error": err if not ok else None,
                "timestamp": datetime.now(timezone.utc).isoformat(),
            }
        except Exception as e:
            return {
                "ok": False,
                "service": service_name,
                "error": str(e),
            }


def health_check() -> Dict[str, Any]:
    """Ejecuta health check de todos los servicios.
    
    Returns:
        Dict con resultado del health check
    """
    results: Dict[str, Dict[str, Any]] = {}
    
    # Hub
    try:
        from modules.humanoid.comms.hub import get_hub
        hub = get_hub()
        results["hub"] = hub.get_health()
    except Exception as e:
        results["hub"] = {"ok": False, "error": str(e)}
    
    # Telegram
    try:
        from modules.humanoid.comms.telegram_bridge import TelegramBridge
        results["telegram"] = TelegramBridge().health_check()
    except Exception as e:
        results["telegram"] = {"ok": False, "error": str(e)}
    
    # WhatsApp
    try:
        from modules.humanoid.comms.whatsapp_bridge import status as wa_status
        results["whatsapp"] = wa_status()
    except Exception as e:
        results["whatsapp"] = {"ok": False, "error": str(e)}
    
    # Webhook
    try:
        from modules.humanoid.comms.webhook_bridge import get_last_error
        last_error = get_last_error()
        results["webhook"] = {
            "ok": last_error is None,
            "last_error": last_error,
        }
    except Exception as e:
        results["webhook"] = {"ok": False, "error": str(e)}
    
    # Audio
    try:
        from modules.humanoid.voice import tts
        results["audio"] = {
            "ok": tts.is_available(),
            "missing_deps": tts.get_missing_deps(),
        }
    except Exception as e:
        results["audio"] = {"ok": False, "error": str(e)}
    
    # Calcular estado general
    all_ok = all(r.get("ok", False) for r in results.values() if r)
    critical_ok = results.get("hub", {}).get("ok", False)
    
    return {
        "ok": critical_ok,
        "all_healthy": all_ok,
        "services": results,
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }
