#!/usr/bin/env python3
"""
ATLAS — Access Code Notifier
==============================
Vigila el access-store.json de RAULI-VISION.
Cuando Rauli aprueba un usuario con teléfono registrado,
envía automáticamente el código de acceso por WhatsApp.

Uso (ejecutar en background):
    python scripts/access_code_notifier.py

El sentinel de ATLAS lo arranca automáticamente al iniciar.
"""

import json
import logging
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import requests

# ─── Paths ────────────────────────────────────────────────────────────────────
BASE_DIR    = Path(__file__).resolve().parent.parent
LOG_DIR     = BASE_DIR / "logs"
LOG_PATH    = LOG_DIR / "access_notifier.log"
ACCESS_STORE = BASE_DIR / "_external" / "RAULI-VISION" / "espejo" / "data" / "access-store.json"
STATE_PATH  = BASE_DIR / "state" / "access_notifier_state.json"
ATLAS_URL   = os.environ.get("ATLAS_URL", "http://127.0.0.1:8791")
POLL_SECS   = int(os.environ.get("ACCESS_NOTIFIER_POLL", "15"))

LOG_DIR.mkdir(exist_ok=True)
(BASE_DIR / "state").mkdir(exist_ok=True)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [ACCESS-NOTIFIER] %(levelname)s %(message)s",
    handlers=[
        logging.FileHandler(LOG_PATH, encoding="utf-8"),
        logging.StreamHandler(sys.stdout),
    ],
)
log = logging.getLogger("access_notifier")

# ─── Plantilla del mensaje ─────────────────────────────────────────────────────
MSG_TEMPLATE = (
    "Hola {name}! 👋\n\n"
    "Tu acceso a *RAULI-VISION* ha sido aprobado.\n\n"
    "Tu código personal es:\n"
    "*{code}*\n\n"
    "Úsalo en la pantalla de acceso para entrar.\n"
    "¡Bienvenido/a al sistema! 🎉"
)


def _load_state() -> dict:
    if STATE_PATH.exists():
        try:
            return json.loads(STATE_PATH.read_text(encoding="utf-8"))
        except Exception:
            pass
    return {"notified": []}


def _save_state(state: dict) -> None:
    STATE_PATH.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")


def _load_store() -> dict:
    try:
        return json.loads(ACCESS_STORE.read_text(encoding="utf-8"))
    except Exception as e:
        log.warning(f"No se pudo leer access-store: {e}")
        return {}


def _send_whatsapp(phone: str, message: str) -> bool:
    """Envía mensaje vía /api/comms/whatsapp/send en ATLAS."""
    try:
        resp = requests.post(
            f"{ATLAS_URL}/api/comms/whatsapp/send",
            json={"text": message, "to": phone},
            timeout=15,
        )
        data = resp.json()
        if data.get("ok"):
            return True
        log.warning(f"WhatsApp no enviado: {data.get('error', data)}")
        return False
    except Exception as e:
        log.error(f"Error enviando WhatsApp a {phone}: {e}")
        return False


def _mark_code_sent(user_id: str) -> None:
    """Marca code_sent=true en el access-store para que el admin lo vea."""
    try:
        store = json.loads(ACCESS_STORE.read_text(encoding="utf-8"))
        users = store.get("users", {})
        if user_id in users:
            users[user_id]["code_sent"] = True
            store["updated_at"] = datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
            ACCESS_STORE.write_text(
                json.dumps(store, ensure_ascii=False, indent=2),
                encoding="utf-8"
            )
    except Exception as e:
        log.warning(f"No se pudo marcar code_sent para {user_id}: {e}")


def check_and_notify(state: dict) -> int:
    """Revisa el store y notifica usuarios aprobados con teléfono. Retorna cantidad enviada."""
    store = _load_store()
    users = store.get("users", {})
    notified_ids: list = state.get("notified", [])
    sent = 0

    for uid, user in users.items():
        # Solo usuarios activos, con teléfono, sin notificar aún
        if user.get("status") != "active":
            continue
        if uid in notified_ids:
            continue
        if user.get("code_sent"):
            notified_ids.append(uid)
            continue

        phone = (user.get("phone") or "").strip()
        if not phone:
            continue  # sin teléfono → no se puede notificar automáticamente

        name  = user.get("name", "Usuario")
        code  = user.get("access_code", "")
        if not code:
            continue

        msg = MSG_TEMPLATE.format(name=name, code=code)
        log.info(f"Enviando código a {name} ({phone})…")

        ok = _send_whatsapp(phone, msg)
        if ok:
            log.info(f"Código enviado OK a {name}")
            notified_ids.append(uid)
            _mark_code_sent(uid)
            sent += 1
        else:
            log.warning(f"Fallo al enviar a {name} — se reintentará en el próximo ciclo")

    state["notified"] = notified_ids
    return sent


def run() -> None:
    log.info("ATLAS Access Code Notifier arrancado")
    log.info(f"Vigilando: {ACCESS_STORE}")
    log.info(f"Intervalo de polling: {POLL_SECS}s")

    if not ACCESS_STORE.exists():
        log.error(f"access-store.json no encontrado: {ACCESS_STORE}")
        log.error("Asegúrate de que RAULI-VISION esté corriendo.")
        sys.exit(1)

    state = _load_state()
    last_mtime = 0.0

    while True:
        try:
            mtime = ACCESS_STORE.stat().st_mtime
            if mtime != last_mtime:
                last_mtime = mtime
                sent = check_and_notify(state)
                _save_state(state)
                if sent:
                    log.info(f"Ciclo: {sent} código(s) enviado(s)")
        except Exception as e:
            log.error(f"Error en ciclo: {e}")

        time.sleep(POLL_SECS)


if __name__ == "__main__":
    run()
