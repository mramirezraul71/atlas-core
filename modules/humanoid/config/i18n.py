"""Internacionalización: español (es) e inglés (en)."""
from __future__ import annotations

import os
from typing import Any, Dict, Optional

SUPPORTED_LOCALES = ("es", "en")
_DEFAULT_LOCALE = "es"

# Locale por defecto (env UI_LOCALE o es)
_current: str = os.getenv("UI_LOCALE", _DEFAULT_LOCALE).strip().lower() or _DEFAULT_LOCALE
if _current not in SUPPORTED_LOCALES:
    _current = _DEFAULT_LOCALE

# Cadenas por idioma. Clave: identificador; valor: texto.
_STRINGS: Dict[str, Dict[str, str]] = {
    "es": {
        "app.title": "ATLAS Dashboard",
        "app.refresh": "Actualizar",
        "app.last_update": "Última actualización",
        "card.status": "Estado",
        "card.version": "Versión",
        "card.health": "Salud",
        "card.metrics": "Métricas",
        "card.scheduler": "Programador",
        "card.update": "Actualización",
        "card.deploy": "Despliegue",
        "card.approvals": "Aprobaciones",
        "robot.title": "Interactuar con ATLAS",
        "robot.placeholder": "Escribe un comando (ej. /status) o un objetivo para la IA…",
        "robot.send": "Enviar",
        "robot.sending": "Enviando…",
        "robot.task_type": "Envergadura de la tarea",
        "robot.task_quick": "Rápida (comando)",
        "robot.task_quick_help": "Comando directo: /status, /doctor, /modules. Sin IA.",
        "robot.task_medium": "Media (IA plan)",
        "robot.task_medium_help": "La IA genera un plan; no ejecuta. Ideal para revisar pasos.",
        "robot.task_full": "Compleja (IA plan + ejecución)",
        "robot.task_full_help": "Plan y ejecución con aprobación. Mayor uso de recursos.",
        "robot.response": "Respuesta",
        "robot.error": "Error",
        "robot.no_input": "Escribe un comando o objetivo.",
        "approvals.none": "Ninguno",
        "jobs.count": "{n} trabajo(s)",
        "update.check": "Comprobar",
        "deploy.mode": "modo: {mode} · puerto: {port} · salud: {score} · {canary}",
        "deploy.canary_on": "Canary ON",
        "deploy.canary_off": "Canary OFF",
    },
    "en": {
        "app.title": "ATLAS Dashboard",
        "app.refresh": "Refresh",
        "app.last_update": "Last update",
        "card.status": "Status",
        "card.version": "Version",
        "card.health": "Health",
        "card.metrics": "Metrics",
        "card.scheduler": "Scheduler",
        "card.update": "Update",
        "card.deploy": "Deploy",
        "card.approvals": "Approvals",
        "robot.title": "Interact with ATLAS",
        "robot.placeholder": "Type a command (e.g. /status) or a goal for AI…",
        "robot.send": "Send",
        "robot.sending": "Sending…",
        "robot.task_type": "Task scope",
        "robot.task_quick": "Quick (command)",
        "robot.task_quick_help": "Direct command: /status, /doctor. No AI.",
        "robot.task_medium": "Medium (AI plan)",
        "robot.task_medium_help": "AI generates a plan; does not execute. Review steps.",
        "robot.task_full": "Full (AI plan + execution)",
        "robot.task_full_help": "Plan and execution with approval. Higher resource use.",
        "robot.response": "Response",
        "robot.error": "Error",
        "robot.no_input": "Enter a command or goal.",
        "approvals.none": "None",
        "jobs.count": "{n} job(s)",
        "update.check": "Check",
        "deploy.mode": "mode: {mode} · port: {port} · health: {score} · {canary}",
        "deploy.canary_on": "Canary ON",
        "deploy.canary_off": "Canary OFF",
    },
}


def get_locale() -> str:
    """Locale activo (es o en)."""
    return _current


def set_locale(locale: str) -> None:
    """Fija locale; si no es válido, se mantiene el actual."""
    global _current
    if locale.strip().lower() in SUPPORTED_LOCALES:
        _current = locale.strip().lower()


def get_text(key: str, locale: Optional[str] = None, **fmt: Any) -> str:
    """Devuelve la cadena para key en el locale dado (o el actual). fmt para sustituir {name}."""
    loc = (locale or _current).strip().lower()
    if loc not in SUPPORTED_LOCALES:
        loc = _DEFAULT_LOCALE
    s = (_STRINGS.get(loc) or _STRINGS[_DEFAULT_LOCALE]).get(key, key)
    if fmt:
        for k, v in fmt.items():
            s = s.replace("{" + k + "}", str(v))
    return s


def get_all_strings(locale: Optional[str] = None) -> Dict[str, str]:
    """Devuelve todas las cadenas del locale (para API /ui/lang)."""
    loc = (locale or _current).strip().lower()
    if loc not in SUPPORTED_LOCALES:
        loc = _DEFAULT_LOCALE
    return dict(_STRINGS.get(loc, _STRINGS[_DEFAULT_LOCALE]))
