"""ATLAS comms bridge with hybrid AI, offline queue and secure audit logging."""
from __future__ import annotations

import argparse
import base64
import concurrent.futures
import hashlib
import json
import os
import re
import sqlite3
import subprocess
import shutil
import threading
import time
import urllib.error
import urllib.request
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple


ROOT_DIR = Path(__file__).resolve().parents[3]
LOG_DIR = ROOT_DIR / "logs"
LOG_FILE = LOG_DIR / "snapshot_safe_diagnostic.log"
DB_FILE = Path(
    (os.getenv("ATLAS_COMMS_DB_PATH") or str(LOG_DIR / "atlas_comms_hub.sqlite")).strip()
)
SCHEMA_FILE = ROOT_DIR / "contracts" / "atlas_comms_messages.schema.sql"
INVENTORY_SCHEMA_FILE = (
    ROOT_DIR / "contracts" / "panaderia_inventory_adjustment_command.schema.json"
)
SNAPSHOT_SCRIPT = ROOT_DIR / "scripts" / "atlas_snapshot_safe.ps1"

PANADERIA_BASE = (os.getenv("ATLAS_PANADERIA_BASE_URL") or "http://127.0.0.1:3001").rstrip(
    "/"
)
VISION_BASE = (os.getenv("ATLAS_VISION_BASE_URL") or "http://127.0.0.1:3000").rstrip("/")
PUSH_BASE = (os.getenv("ATLAS_PUSH_BASE_URL") or "http://127.0.0.1:8791").rstrip("/")
WHATSAPP_DOWNLOAD_URL = "https://www.whatsapp.com/download"
PANADERIA_APP_URL = (
    os.getenv("ATLAS_PANADERIA_APP_URL") or ""
).strip()
VISION_APP_URL = (os.getenv("ATLAS_VISION_APP_URL") or "").strip()

DEFAULT_SCHEDULE = "Lun-Dom 07:00-20:00"
DEFAULT_ORDER_GUIDE = "Puedes indicar producto, cantidad y hora deseada para preparar el pedido."

_RE_URGENT = re.compile(
    r"(urgente|emergencia|incendio|robo|asalto|grave|critico|critical|"
    r"no funciona|caido|caida|fuga|accidente|sangre|peligro)",
    re.IGNORECASE,
)
_RE_ORDER = re.compile(r"(pedido|orden|encargo|comprar|precio|delivery|envio)", re.IGNORECASE)
_RE_HOURS = re.compile(r"(horario|abren|abierto|cierran|hora)", re.IGNORECASE)
_RE_STOCK = re.compile(r"(stock|disponible|inventario|hay|agotado|existencia)", re.IGNORECASE)
_RE_CAMERA = re.compile(
    r"(camara|camaras|camera|cctv|stream|snapshot|vision|monitoreo|online|estado)",
    re.IGNORECASE,
)
_RE_DIGITS = re.compile(r"\D+")
_RE_PROVIDER_LATENCY = re.compile(r":(?P<ms>\d{1,6})ms$", re.IGNORECASE)
_RE_PANADERIA_TOOLS = re.compile(
    r"(venta|vender|cobrar|precio|cantidad|importe|gasto|compra|almacen|almac[eé]n|"
    r"inventario|stock|existencia|entrada|salida|produccion|producci[oó]n|resumen|"
    r"cier(re|ra)|cuadre|efectivo|tarjeta|producto)",
    re.IGNORECASE,
)
_RE_REASONING = re.compile(
    r"(analiza|analisis|diagnostico|diagn[oó]stico|resumen|explica|por qu[eé]|"
    r"comparar|comparaci[oó]n|plan|estrategia|contabilidad|tendencia|acumulado)",
    re.IGNORECASE,
)
_RE_FAST_UI = re.compile(
    r"^(abrir|ir a|ve a|mostrar|muestra|entra|abre|ventas de hoy|resumen del dia|stock harina)\b",
    re.IGNORECASE,
)
_RE_VISION_LOCAL = re.compile(
    r"(buscar|busqueda|b[uú]squeda|resumir|resumen|internet|video|tv|canal|canales|"
    r"tiktok|youtube|noticia|noticias|url|enlace)",
    re.IGNORECASE,
)
_CONTACT_TONE_PROFILES: Dict[str, Dict[str, str]] = {
    "VISION:CAMI": {
        "name": "Cami",
        "tone": "muy cercano, amable, directo y breve; hablar de tu",
    },
    "VISION:RULITIN": {
        "name": "Rulitin",
        "tone": "cercano, dinamico y muy accionable; lenguaje simple",
    },
    "VISION:IVIS": {
        "name": "Ivis",
        "tone": "cercano, claro y tranquilo; sin tecnicismos innecesarios",
    },
}
_VAULT_ENV_LOADED = False
_VAULT_ENV_LOCK = threading.Lock()

_DB_SCHEMA = """
CREATE TABLE IF NOT EXISTS comms_messages (
    message_id TEXT PRIMARY KEY,
    created_at TEXT NOT NULL,
    user_id TEXT,
    channel TEXT,
    urgency TEXT,
    request_summary TEXT,
    request_encrypted TEXT NOT NULL,
    response_summary TEXT,
    response_encrypted TEXT,
    inventory_status TEXT,
    camera_status TEXT,
    offline_mode INTEGER NOT NULL DEFAULT 0,
    synced INTEGER NOT NULL DEFAULT 1,
    sync_attempts INTEGER NOT NULL DEFAULT 0,
    last_error TEXT,
    metadata_json TEXT
);

CREATE TABLE IF NOT EXISTS comms_offline_queue (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    message_id TEXT NOT NULL,
    payload_encrypted TEXT NOT NULL,
    created_at TEXT NOT NULL,
    status TEXT NOT NULL DEFAULT 'pending',
    attempts INTEGER NOT NULL DEFAULT 0,
    last_error TEXT,
    dequeued_at TEXT,
    FOREIGN KEY(message_id) REFERENCES comms_messages(message_id)
);

CREATE INDEX IF NOT EXISTS idx_comms_messages_created_at ON comms_messages(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_comms_messages_synced ON comms_messages(synced, created_at DESC);
CREATE INDEX IF NOT EXISTS idx_comms_queue_status ON comms_offline_queue(status, created_at ASC);

CREATE TABLE IF NOT EXISTS comms_external_users (
    source TEXT NOT NULL,
    external_user_id TEXT NOT NULL,
    username TEXT,
    full_name TEXT,
    email TEXT,
    role TEXT,
    status TEXT,
    whatsapp_number TEXT,
    whatsapp_number_norm TEXT,
    whatsapp_state TEXT NOT NULL DEFAULT 'missing',
    requested_at TEXT,
    linked_at TEXT,
    updated_at TEXT NOT NULL,
    requested_by TEXT,
    request_note TEXT,
    meta_json TEXT,
    PRIMARY KEY(source, external_user_id)
);

CREATE INDEX IF NOT EXISTS idx_comms_external_users_state
ON comms_external_users(whatsapp_state, source, updated_at DESC);
CREATE INDEX IF NOT EXISTS idx_comms_external_users_phone
ON comms_external_users(whatsapp_number_norm);
"""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _truncate(text: str, max_len: int = 220) -> str:
    t = (text or "").strip().replace("\n", " ")
    if len(t) <= max_len:
        return t
    return t[: max(0, max_len - 3)] + "..."


def _normalize_phone(raw_phone: str) -> Tuple[str, str]:
    raw = (raw_phone or "").strip()
    if not raw:
        return "", ""
    if raw.lower().startswith("whatsapp:"):
        raw = raw.split(":", 1)[1].strip()
    if "@c.us" in raw:
        raw = raw.split("@", 1)[0].strip()
    normalized_digits = _RE_DIGITS.sub("", raw)
    if not normalized_digits:
        return "", ""
    if raw.startswith("+"):
        display = f"+{normalized_digits}"
    else:
        display = normalized_digits
    return display, normalized_digits


def _json_dump(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, separators=(",", ":"))


def _percentile_int(values: Iterable[int], percentile: int) -> Optional[int]:
    vals = sorted(int(v) for v in values if isinstance(v, (int, float)))
    if not vals:
        return None
    p = max(1, min(int(percentile or 50), 99))
    rank = int((p / 100.0) * len(vals) + 0.999999)
    idx = max(0, min(len(vals) - 1, rank - 1))
    return int(vals[idx])


def _resolve_claude_cli_bin() -> str:
    explicit = (os.getenv("ATLAS_CLAWD_CLAUDE_CLI_BIN") or "").strip()
    candidates = []
    if explicit:
        candidates.append(explicit)
    found = shutil.which("claude")
    if found:
        candidates.append(found)
    appdata = (os.getenv("APPDATA") or "").strip()
    if appdata:
        candidates.append(str(Path(appdata) / "npm" / "claude.cmd"))
    userprofile = (os.getenv("USERPROFILE") or "").strip()
    if userprofile:
        candidates.append(str(Path(userprofile) / "AppData" / "Roaming" / "npm" / "claude.cmd"))
    for candidate in candidates:
        if candidate and Path(candidate).exists():
            return candidate
    return explicit or "claude"


def _load_vault_env_once() -> None:
    global _VAULT_ENV_LOADED
    if _VAULT_ENV_LOADED:
        return
    with _VAULT_ENV_LOCK:
        if _VAULT_ENV_LOADED:
            return
        try:
            from modules.humanoid.config.vault import load_vault_env

            load_vault_env(override=False)
        except Exception:
            pass
        _VAULT_ENV_LOADED = True


def _http_json(
    method: str,
    url: str,
    payload: Optional[dict] = None,
    headers: Optional[Dict[str, str]] = None,
    timeout: float = 7.0,
) -> Tuple[bool, int, Dict[str, Any], str]:
    hdr = {"Content-Type": "application/json"}
    if headers:
        hdr.update(headers)
    body = None
    if payload is not None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(url=url, data=body, method=method.upper(), headers=hdr)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
            try:
                parsed = json.loads(raw)
                data = parsed if isinstance(parsed, dict) else {"raw": parsed}
            except Exception:
                data = {"raw": raw}
            return True, int(resp.status), data, ""
    except urllib.error.HTTPError as e:
        try:
            text = e.read().decode("utf-8", errors="replace")
            parsed = json.loads(text)
            data = parsed if isinstance(parsed, dict) else {"raw": parsed}
        except Exception:
            data = {}
        return False, int(e.code), data, f"http_{e.code}"
    except Exception as e:
        return False, 0, {}, str(e)


class AtlasCommsHub:
    def __init__(self) -> None:
        _load_vault_env_once()
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        DB_FILE.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._probe_cache_lock = threading.Lock()
        self._probe_cache_ts = 0.0
        self._probe_cache_inv: Dict[str, Any] = {}
        self._probe_cache_cam: Dict[str, Any] = {}
        self._net_cache_lock = threading.Lock()
        self._net_cache_ts = 0.0
        self._net_cache_offline = False
        self._net_cache_diag: Dict[str, Any] = {}
        self._clawd_state_lock = threading.Lock()
        self._clawd_backoff_until = 0.0
        self._clawd_fail_streak = 0
        self._clawd_last_error = ""
        self._init_db()

    def _encryption_secret(self) -> Tuple[str, str]:
        _load_vault_env_once()
        core = (os.getenv("ATLAS_CENTRAL_CORE") or "").strip()
        if core:
            return core, "ATLAS_CENTRAL_CORE"
        fallback = (os.getenv("APPROVALS_CHAIN_SECRET") or "").strip()
        if fallback:
            return fallback, "APPROVALS_CHAIN_SECRET"
        seed = f"{os.getenv('COMPUTERNAME','atlas')}|{ROOT_DIR}"
        return hashlib.sha256(seed.encode("utf-8")).hexdigest(), "MACHINE_FALLBACK"

    def _stream_xor(self, data: bytes, key: bytes, nonce: bytes) -> bytes:
        out = bytearray()
        idx = 0
        while len(out) < len(data):
            block = hashlib.sha256(key + nonce + idx.to_bytes(8, "big")).digest()
            out.extend(block)
            idx += 1
        return bytes(a ^ b for a, b in zip(data, out[: len(data)]))

    def _encrypt_text(self, text: str) -> str:
        secret, _source = self._encryption_secret()
        key = hashlib.sha256(secret.encode("utf-8")).digest()
        nonce = os.urandom(16)
        cipher = self._stream_xor((text or "").encode("utf-8"), key, nonce)
        return base64.b64encode(nonce + cipher).decode("ascii")

    def _decrypt_text(self, encoded: str) -> str:
        raw = base64.b64decode((encoded or "").encode("ascii"), validate=False)
        if len(raw) < 16:
            return ""
        nonce = raw[:16]
        cipher = raw[16:]
        secret, _source = self._encryption_secret()
        key = hashlib.sha256(secret.encode("utf-8")).digest()
        return self._stream_xor(cipher, key, nonce).decode("utf-8", errors="replace")

    def _db_conn(self) -> sqlite3.Connection:
        conn = sqlite3.connect(str(DB_FILE), timeout=15, check_same_thread=False)
        conn.execute("PRAGMA journal_mode=WAL;")
        conn.execute("PRAGMA synchronous=NORMAL;")
        conn.row_factory = sqlite3.Row
        return conn

    def _init_db(self) -> None:
        with self._lock:
            conn = self._db_conn()
            try:
                conn.executescript(_DB_SCHEMA)
                conn.commit()
            finally:
                conn.close()
        self._ensure_schema_file()

    def _ensure_schema_file(self) -> None:
        if not SCHEMA_FILE.exists():
            SCHEMA_FILE.parent.mkdir(parents=True, exist_ok=True)
            SCHEMA_FILE.write_text(_DB_SCHEMA.strip() + "\n", encoding="utf-8")

    def _append_snapshot(self, tag: str, payload: Dict[str, Any]) -> None:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        with LOG_FILE.open("a", encoding="utf-8") as f:
            f.write(f"{_utc_now()} {tag} {_json_dump(payload)}\n")

    def _probe_timeout_s(self) -> float:
        try:
            v = float((os.getenv("ATLAS_COMMS_PROBE_TIMEOUT_S") or "2.0").strip() or "2.0")
        except Exception:
            v = 2.0
        return max(0.8, min(v, 8.0))

    def _clawd_timeout_s(self) -> float:
        try:
            v = float((os.getenv("ATLAS_COMMS_CLAWD_TIMEOUT_S") or "2.5").strip() or "2.5")
        except Exception:
            v = 2.5
        return max(1.0, min(v, 12.0))

    def _clawd_backoff_base_s(self) -> float:
        try:
            v = float((os.getenv("ATLAS_COMMS_CLAWD_BACKOFF_S") or "20").strip() or "20")
        except Exception:
            v = 20.0
        return max(5.0, min(v, 180.0))

    def _probe_panaderia(self) -> Dict[str, Any]:
        status: Dict[str, Any] = {"reachable": False, "health": {}, "inventory": {}}
        timeout_s = self._probe_timeout_s()
        ok, code, data, err = _http_json("GET", f"{PANADERIA_BASE}/api/health", timeout=timeout_s)
        status["health"] = {"ok": ok, "code": code, "data": data, "error": err}
        status["reachable"] = bool(ok)

        s_ok, s_code, s_data, s_err = _http_json(
            "GET", f"{PANADERIA_BASE}/api/sentinel/health", timeout=timeout_s
        )
        status["inventory"]["sentinel"] = {
            "ok": s_ok,
            "code": s_code,
            "data": s_data,
            "error": s_err,
        }

        auth = (os.getenv("ATLAS_PANADERIA_AUTH_BEARER") or "").strip()
        headers = {"Authorization": f"Bearer {auth}"} if auth else {}
        i_ok, i_code, i_data, i_err = _http_json(
            "GET",
            f"{PANADERIA_BASE}/api/inventory/summary",
            timeout=timeout_s,
            headers=headers if headers else None,
        )
        status["inventory"]["summary"] = {
            "ok": i_ok,
            "code": i_code,
            "data": i_data,
            "error": i_err,
            "auth_used": bool(auth),
        }
        return status

    def _probe_vision(self) -> Dict[str, Any]:
        status: Dict[str, Any] = {"reachable": False, "health": {}, "cameras": {}}
        timeout_s = self._probe_timeout_s()
        ok, code, data, err = _http_json("GET", f"{VISION_BASE}/api/health", timeout=timeout_s)
        status["health"] = {"ok": ok, "code": code, "data": data, "error": err}
        status["reachable"] = bool(ok)
        c_ok, c_code, c_data, c_err = _http_json("GET", f"{PUSH_BASE}/vision/cameras", timeout=timeout_s)
        status["cameras"] = {"ok": c_ok, "code": c_code, "data": c_data, "error": c_err}
        return status

    def _resolve_target_sources(self, targets: Optional[Iterable[str]] = None) -> list[str]:
        if not targets:
            return ["panaderia", "vision"]
        out: list[str] = []
        for t in targets:
            v = (str(t or "").strip().lower())
            if v in ("panaderia", "vision") and v not in out:
                out.append(v)
        return out or ["panaderia", "vision"]

    def _panaderia_auth_headers(self) -> Tuple[Dict[str, str], str]:
        _load_vault_env_once()
        bearer = (
            os.getenv("ATLAS_PANADERIA_AUTH_BEARER")
            or os.getenv("PANADERIA_AUTH_BEARER")
            or ""
        ).strip()
        if bearer:
            return {"Authorization": f"Bearer {bearer}"}, "bearer_env"

        username = (
            os.getenv("ATLAS_PANADERIA_AUTH_USER")
            or os.getenv("PANADERIA_AUTH_USER")
            or ""
        ).strip()
        password = (
            os.getenv("ATLAS_PANADERIA_AUTH_PASSWORD")
            or os.getenv("PANADERIA_AUTH_PASSWORD")
            or ""
        ).strip()
        if not username or not password:
            return {}, "missing_credentials"

        ok, code, data, err = _http_json(
            "POST",
            f"{PANADERIA_BASE}/api/auth/login",
            payload={"username": username, "password": password},
            timeout=7.0,
        )
        token = str((data or {}).get("token") or "").strip()
        if ok and token:
            return {"Authorization": f"Bearer {token}"}, "login_token"
        return {}, f"login_failed:{err or code}"

    def _fetch_panaderia_users(self) -> Dict[str, Any]:
        headers, auth_source = self._panaderia_auth_headers()
        if not headers:
            return {"ok": False, "error": "panaderia_auth_missing", "auth_source": auth_source}
        ok, code, data, err = _http_json(
            "GET",
            f"{PANADERIA_BASE}/api/auth/users",
            headers=headers,
            timeout=8.0,
        )
        if not ok:
            return {
                "ok": False,
                "error": f"panaderia_users_fail:{err or code}",
                "http_code": code,
                "auth_source": auth_source,
            }
        users = (data or {}).get("users")
        if not isinstance(users, list):
            users = (data or {}).get("items")
        if not isinstance(users, list):
            users = []
        return {"ok": True, "items": users, "count": len(users), "auth_source": auth_source}

    def _fetch_vision_users(self) -> Dict[str, Any]:
        _load_vault_env_once()
        admin_token = (
            os.getenv("ATLAS_VISION_ADMIN_TOKEN")
            or os.getenv("VISION_ADMIN_TOKEN")
            or os.getenv("RAULI_VISION_ADMIN_TOKEN")
            or ""
        ).strip()
        if not admin_token:
            return {"ok": False, "error": "vision_admin_token_missing"}
        ok, code, data, err = _http_json(
            "GET",
            f"{VISION_BASE}/api/access/users",
            headers={"X-Admin-Token": admin_token},
            timeout=8.0,
        )
        if not ok:
            return {
                "ok": False,
                "error": f"vision_users_fail:{err or code}",
                "http_code": code,
            }
        users = (data or {}).get("items")
        if not isinstance(users, list):
            users = (data or {}).get("users")
        if not isinstance(users, list):
            users = []
        return {"ok": True, "items": users, "count": len(users)}

    def _normalize_external_user(self, source: str, raw: Dict[str, Any]) -> Dict[str, Any]:
        src = (source or "").strip().lower()
        item = raw if isinstance(raw, dict) else {}
        ext_id = str(item.get("id") or "").strip()
        if not ext_id:
            return {}
        if src == "panaderia":
            active = item.get("active")
            status = "active" if str(active).strip() in ("1", "true", "True") else "inactive"
            return {
                "source": src,
                "external_user_id": ext_id,
                "username": str(item.get("username") or "").strip(),
                "full_name": str(item.get("name") or item.get("username") or "").strip(),
                "email": str(item.get("email") or "").strip(),
                "role": str(item.get("role") or "").strip(),
                "status": status,
                "meta_json": _json_dump(item),
            }
        return {
            "source": src,
            "external_user_id": ext_id,
            "username": str(item.get("username") or "").strip(),
            "full_name": str(item.get("name") or item.get("email") or ext_id).strip(),
            "email": str(item.get("email") or "").strip(),
            "role": str(item.get("role") or "").strip(),
            "status": str(item.get("status") or "").strip(),
            "meta_json": _json_dump(item),
        }

    def _upsert_external_users(self, source: str, raw_items: list[Dict[str, Any]]) -> Dict[str, Any]:
        now = _utc_now()
        normalized = []
        for raw in raw_items:
            item = self._normalize_external_user(source, raw)
            if item:
                normalized.append(item)
        conn = self._db_conn()
        try:
            for item in normalized:
                conn.execute(
                    """
                    INSERT INTO comms_external_users (
                        source, external_user_id, username, full_name, email, role, status,
                        whatsapp_number, whatsapp_number_norm, whatsapp_state, requested_at, linked_at,
                        updated_at, requested_by, request_note, meta_json
                    ) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
                    ON CONFLICT(source, external_user_id) DO UPDATE SET
                        username=excluded.username,
                        full_name=excluded.full_name,
                        email=excluded.email,
                        role=excluded.role,
                        status=excluded.status,
                        updated_at=excluded.updated_at,
                        meta_json=excluded.meta_json
                    """,
                    (
                        item["source"],
                        item["external_user_id"],
                        item["username"],
                        item["full_name"],
                        item["email"],
                        item["role"],
                        item["status"],
                        None,
                        None,
                        "missing",
                        None,
                        None,
                        now,
                        None,
                        None,
                        item["meta_json"],
                    ),
                )
            conn.commit()
        finally:
            conn.close()
        return {"ok": True, "source": source, "upserted": len(normalized)}

    def sync_external_users(self, targets: Optional[Iterable[str]] = None) -> Dict[str, Any]:
        sources = self._resolve_target_sources(targets)
        result: Dict[str, Any] = {"ok": True, "targets": sources, "results": []}
        for source in sources:
            if source == "panaderia":
                fetched = self._fetch_panaderia_users()
            else:
                fetched = self._fetch_vision_users()
            if not fetched.get("ok"):
                result["ok"] = False
                result["results"].append({"source": source, **fetched})
                continue
            up = self._upsert_external_users(source, fetched.get("items") or [])
            result["results"].append(
                {
                    "source": source,
                    "fetched": int(fetched.get("count") or 0),
                    "upserted": int(up.get("upserted") or 0),
                    "auth_source": fetched.get("auth_source"),
                    "ok": True,
                }
            )
        return result

    def _request_message_template(self, display_name: str, source: str) -> str:
        who = display_name or "equipo"
        app_name = "RAULI-PANADERIA" if source == "panaderia" else "RAULI-VISION"
        return (
            f"Hola {who}, para activar el canal principal de soporte en WhatsApp para {app_name} "
            "necesitamos tu número en formato internacional (ejemplo: +34123456789). "
            f"Si no tienes WhatsApp instalado puedes descargarlo aquí: {WHATSAPP_DOWNLOAD_URL}"
        )

    def _app_link_for_source(self, source: str, override: str = "") -> str:
        custom = (override or "").strip()
        if custom:
            return custom
        tunnel_url = (os.getenv("CLOUDFLARE_TUNNEL_URL") or "").strip()
        if tunnel_url:
            return tunnel_url
        if source == "panaderia":
            return PANADERIA_APP_URL
        if source == "vision":
            return VISION_APP_URL
        return ""

    def build_welcome_message(
        self,
        *,
        source: str,
        display_name: str,
        app_link: str = "",
    ) -> str:
        src = (source or "").strip().lower()
        app_name = "RAULI Panaderia" if src == "panaderia" else "RAULI Vision"
        name = (display_name or "equipo").strip() or "equipo"
        link = self._app_link_for_source(src, override=app_link)
        lines = [
            f"Hola {name}, te da la bienvenida ATLAS Robot de Rauli.",
            f"Quedaste habilitado para comunicación directa por WhatsApp en {app_name}.",
        ]
        if link:
            lines.append(f"Descarga o abre la app aquí: {link}")
        lines.append("Cuando quieras, escríbeme por este canal y te respondo de inmediato.")
        return " ".join(lines)

    def _resolve_external_user_id(self, source: str, match: str) -> Tuple[bool, str, str]:
        src = (source or "").strip().lower()
        needle = (match or "").strip()
        if src not in ("panaderia", "vision"):
            return False, "", "invalid_source"
        if not needle:
            return False, "", "match_required"
        lowered = needle.lower()
        conn = self._db_conn()
        try:
            rows = conn.execute(
                """
                SELECT external_user_id
                FROM comms_external_users
                WHERE source=?
                  AND (
                      lower(external_user_id)=?
                      OR lower(username)=?
                      OR lower(full_name)=?
                      OR lower(email)=?
                  )
                ORDER BY updated_at DESC
                LIMIT 2
                """,
                (src, lowered, lowered, lowered, lowered),
            ).fetchall()
        finally:
            conn.close()
        if not rows:
            return False, "", "external_user_not_found"
        if len(rows) > 1:
            return False, "", "ambiguous_match"
        return True, str(rows[0]["external_user_id"]), "ok"

    def list_external_users(
        self,
        *,
        source: str = "",
        whatsapp_state: str = "",
        limit: int = 300,
    ) -> Dict[str, Any]:
        lim = max(1, min(int(limit or 300), 1000))
        src = (source or "").strip().lower()
        st = (whatsapp_state or "").strip().lower()
        where = ["1=1"]
        params: list[Any] = []
        if src in ("panaderia", "vision"):
            where.append("source=?")
            params.append(src)
        if st in ("missing", "requested", "linked"):
            where.append("whatsapp_state=?")
            params.append(st)
        conn = self._db_conn()
        try:
            rows = conn.execute(
                f"""
                SELECT source, external_user_id, username, full_name, email, role, status,
                       whatsapp_number, whatsapp_state, requested_at, linked_at, updated_at,
                       requested_by, request_note
                FROM comms_external_users
                WHERE {' AND '.join(where)}
                ORDER BY source ASC, updated_at DESC
                LIMIT ?
                """,
                (*params, lim),
            ).fetchall()
        finally:
            conn.close()

        items = []
        summary: Dict[str, int] = {"total": 0, "missing": 0, "requested": 0, "linked": 0}
        for r in rows:
            state = str(r["whatsapp_state"] or "missing")
            summary["total"] += 1
            summary[state] = summary.get(state, 0) + 1
            name = str(r["full_name"] or r["username"] or r["email"] or r["external_user_id"])
            items.append(
                {
                    "source": r["source"],
                    "external_user_id": r["external_user_id"],
                    "username": r["username"],
                    "full_name": r["full_name"],
                    "email": r["email"],
                    "role": r["role"],
                    "status": r["status"],
                    "whatsapp_number": r["whatsapp_number"],
                    "whatsapp_state": state,
                    "requested_at": r["requested_at"],
                    "linked_at": r["linked_at"],
                    "updated_at": r["updated_at"],
                    "requested_by": r["requested_by"],
                    "request_note": r["request_note"],
                    "request_message_template": (
                        self._request_message_template(name, str(r["source"] or ""))
                        if state != "linked"
                        else ""
                    ),
                }
            )
        return {"ok": True, "summary": summary, "items": items}

    def link_external_user_whatsapp(
        self,
        *,
        source: str,
        external_user_id: str,
        whatsapp_number: str,
        updated_by: str = "atlas_supervisor",
    ) -> Dict[str, Any]:
        src = (source or "").strip().lower()
        ext_id = (external_user_id or "").strip()
        phone_display, phone_norm = _normalize_phone(whatsapp_number)
        if src not in ("panaderia", "vision"):
            return {"ok": False, "error": "invalid_source"}
        if not ext_id:
            return {"ok": False, "error": "external_user_id_required"}
        if not phone_norm:
            return {"ok": False, "error": "invalid_whatsapp_number"}

        now = _utc_now()
        conn = self._db_conn()
        try:
            cur = conn.execute(
                """
                UPDATE comms_external_users
                SET whatsapp_number=?,
                    whatsapp_number_norm=?,
                    whatsapp_state='linked',
                    linked_at=?,
                    updated_at=?,
                    requested_by=?,
                    request_note=NULL
                WHERE source=? AND external_user_id=?
                """,
                (phone_display, phone_norm, now, now, (updated_by or "atlas_supervisor"), src, ext_id),
            )
            conn.commit()
            if int(cur.rowcount or 0) <= 0:
                exists = conn.execute(
                    """
                    SELECT 1
                    FROM comms_external_users
                    WHERE source=? AND external_user_id=?
                    LIMIT 1
                    """,
                    (src, ext_id),
                ).fetchone()
                if not exists:
                    return {
                        "ok": False,
                        "error": "external_user_not_found",
                        "source": src,
                        "external_user_id": ext_id,
                    }
        finally:
            conn.close()
        return {
            "ok": True,
            "source": src,
            "external_user_id": ext_id,
            "whatsapp_number": phone_display,
            "whatsapp_number_norm": phone_norm,
            "state": "linked",
        }

    def configure_external_user_whatsapp(
        self,
        *,
        source: str,
        external_user_id: str,
        whatsapp_number: str,
        updated_by: str = "atlas_supervisor",
        send_welcome: bool = True,
        welcome_name: str = "",
        app_link: str = "",
    ) -> Dict[str, Any]:
        linked = self.link_external_user_whatsapp(
            source=source,
            external_user_id=external_user_id,
            whatsapp_number=whatsapp_number,
            updated_by=updated_by,
        )
        if not linked.get("ok"):
            return linked
        if not send_welcome:
            return {**linked, "welcome_sent": False}
        conn = self._db_conn()
        try:
            row = conn.execute(
                """
                SELECT full_name, username, email
                FROM comms_external_users
                WHERE source=? AND external_user_id=?
                LIMIT 1
                """,
                ((source or "").strip().lower(), (external_user_id or "").strip()),
            ).fetchone()
        finally:
            conn.close()
        display_name = (
            (welcome_name or "").strip()
            or str((row["full_name"] if row else "") or (row["username"] if row else "") or (row["email"] if row else "") or external_user_id)
        )
        msg = self.build_welcome_message(
            source=source,
            display_name=display_name,
            app_link=app_link,
        )
        sent = self.send_whatsapp_to_external_user(
            source=source,
            external_user_id=external_user_id,
            text=msg,
        )
        return {
            **linked,
            "welcome_sent": bool(sent.get("ok")),
            "welcome_message": msg,
            "welcome_result": sent,
        }

    def configure_whatsapp_batch(
        self,
        *,
        source: str,
        items: list[Dict[str, Any]],
        updated_by: str = "atlas_supervisor",
        send_welcome: bool = True,
        app_link: str = "",
    ) -> Dict[str, Any]:
        src = (source or "").strip().lower()
        if src not in ("panaderia", "vision"):
            return {"ok": False, "error": "invalid_source", "results": []}
        rows = items if isinstance(items, list) else []
        out = []
        ok_count = 0
        for it in rows:
            entry = it if isinstance(it, dict) else {}
            ext_id = str(entry.get("external_user_id") or "").strip()
            match = str(
                entry.get("match")
                or entry.get("username")
                or entry.get("full_name")
                or entry.get("name")
                or entry.get("email")
                or ""
            ).strip()
            number = str(entry.get("whatsapp_number") or entry.get("phone") or "").strip()
            welcome_name = str(entry.get("welcome_name") or entry.get("name") or "").strip()

            if not ext_id:
                found, resolved_id, reason = self._resolve_external_user_id(src, match)
                if not found:
                    out.append(
                        {
                            "ok": False,
                            "source": src,
                            "match": match,
                            "error": reason,
                            "whatsapp_number": number,
                        }
                    )
                    continue
                ext_id = resolved_id

            result = self.configure_external_user_whatsapp(
                source=src,
                external_user_id=ext_id,
                whatsapp_number=number,
                updated_by=updated_by,
                send_welcome=send_welcome,
                welcome_name=welcome_name,
                app_link=app_link,
            )
            out.append(result)
            if result.get("ok"):
                ok_count += 1
        return {
            "ok": ok_count == len(rows) if rows else True,
            "source": src,
            "total": len(rows),
            "configured": ok_count,
            "failed": len(rows) - ok_count,
            "results": out,
        }

    def request_numbers_for_missing_users(
        self,
        *,
        source: str = "",
        requested_by: str = "atlas_supervisor",
        note: str = "",
        limit: int = 100,
    ) -> Dict[str, Any]:
        src = (source or "").strip().lower()
        lim = max(1, min(int(limit or 100), 500))
        where = ["whatsapp_state <> 'linked'"]
        params: list[Any] = []
        if src in ("panaderia", "vision"):
            where.append("source=?")
            params.append(src)

        conn = self._db_conn()
        try:
            rows = conn.execute(
                f"""
                SELECT source, external_user_id, full_name, username, email
                FROM comms_external_users
                WHERE {' AND '.join(where)}
                ORDER BY updated_at DESC
                LIMIT ?
                """,
                (*params, lim),
            ).fetchall()
            now = _utc_now()
            for r in rows:
                conn.execute(
                    """
                    UPDATE comms_external_users
                    SET whatsapp_state='requested',
                        requested_at=?,
                        updated_at=?,
                        requested_by=?,
                        request_note=?
                    WHERE source=? AND external_user_id=?
                    """,
                    (
                        now,
                        now,
                        (requested_by or "atlas_supervisor"),
                        (note or "Solicitud de número WhatsApp"),
                        str(r["source"]),
                        str(r["external_user_id"]),
                    ),
                )
            conn.commit()
        finally:
            conn.close()

        requests = []
        for r in rows:
            display_name = str(
                r["full_name"] or r["username"] or r["email"] or r["external_user_id"] or "equipo"
            ).strip()
            requests.append(
                {
                    "source": r["source"],
                    "external_user_id": r["external_user_id"],
                    "display_name": display_name,
                    "request_message": self._request_message_template(display_name, str(r["source"])),
                    "download_link": WHATSAPP_DOWNLOAD_URL,
                }
            )
        return {"ok": True, "requested": len(requests), "items": requests}

    def resolve_external_user_by_whatsapp(self, sender: str) -> Dict[str, Any]:
        _display, phone_norm = _normalize_phone(sender)
        if not phone_norm:
            return {"ok": False, "error": "invalid_sender"}
        conn = self._db_conn()
        try:
            row = conn.execute(
                """
                SELECT source, external_user_id, username, full_name, email, role, status, whatsapp_number
                FROM comms_external_users
                WHERE whatsapp_number_norm=?
                ORDER BY updated_at DESC
                LIMIT 1
                """,
                (phone_norm,),
            ).fetchone()
        finally:
            conn.close()
        if not row:
            return {"ok": False, "error": "not_linked", "phone_norm": phone_norm}
        display_name = str(row["full_name"] or row["username"] or row["email"] or row["external_user_id"])
        return {
            "ok": True,
            "source": row["source"],
            "external_user_id": row["external_user_id"],
            "user_id": f"{row['source']}:{row['external_user_id']}",
            "display_name": display_name,
            "role": row["role"],
            "status": row["status"],
            "whatsapp_number": row["whatsapp_number"],
            "phone_norm": phone_norm,
        }

    def send_whatsapp_to_external_user(
        self,
        *,
        source: str,
        external_user_id: str,
        text: str,
    ) -> Dict[str, Any]:
        src = (source or "").strip().lower()
        ext_id = (external_user_id or "").strip()
        msg = (text or "").strip()
        if src not in ("panaderia", "vision"):
            return {"ok": False, "error": "invalid_source"}
        if not ext_id:
            return {"ok": False, "error": "external_user_id_required"}
        if not msg:
            return {"ok": False, "error": "text_required"}
        conn = self._db_conn()
        try:
            row = conn.execute(
                """
                SELECT full_name, username, email, whatsapp_number, whatsapp_state
                FROM comms_external_users
                WHERE source=? AND external_user_id=?
                LIMIT 1
                """,
                (src, ext_id),
            ).fetchone()
        finally:
            conn.close()
        if not row:
            return {"ok": False, "error": "external_user_not_found"}
        number = str(row["whatsapp_number"] or "").strip()
        if not number:
            display_name = str(row["full_name"] or row["username"] or row["email"] or ext_id).strip()
            return {
                "ok": False,
                "error": "whatsapp_number_missing",
                "action_required": "request_number",
                "request_message": self._request_message_template(display_name, src),
                "download_link": WHATSAPP_DOWNLOAD_URL,
            }
        try:
            from modules.humanoid.comms.whatsapp_bridge import send_text

            result = send_text(msg, to=number)
        except Exception as e:
            return {"ok": False, "error": f"whatsapp_send_failed:{e}"}
        return {
            "ok": bool(result.get("ok")),
            "source": src,
            "external_user_id": ext_id,
            "to": number,
            "provider_result": result,
        }

    def _cloudflare_tunnel_up(self) -> Tuple[bool, str]:
        health_url = (os.getenv("ATLAS_CLOUDFLARE_TUNNEL_HEALTH_URL") or "").strip()
        if health_url:
            ok, code, _data, err = _http_json("GET", health_url, timeout=4.0)
            if ok and 200 <= code < 400:
                return True, "health_url_ok"
            return False, f"health_url_fail:{err or code}"
        try:
            proc = subprocess.run(
                ["tasklist", "/FI", "IMAGENAME eq cloudflared.exe"],
                capture_output=True,
                text=True,
                timeout=6,
            )
            if "cloudflared.exe" in (proc.stdout or "").lower():
                return True, "process_detected"
            return False, "process_not_found"
        except Exception as e:
            return False, f"process_check_error:{e}"

    def _internet_up(self) -> Tuple[bool, str]:
        try:
            timeout_s = float((os.getenv("ATLAS_COMMS_NET_TIMEOUT_S") or "1.2").strip() or "1.2")
        except Exception:
            timeout_s = 1.2
        timeout_s = max(0.8, min(timeout_s, 6.0))
        checks = ["http://1.1.1.1/cdn-cgi/trace", "https://www.cloudflare.com/cdn-cgi/trace"]
        last_err = "unknown"
        for url in checks:
            ok, code, _data, err = _http_json("GET", url, timeout=timeout_s)
            if ok and 200 <= code < 400:
                return True, f"ok:{url}"
            last_err = err or str(code)
        return False, f"offline:{last_err}"

    def _is_offline_mode(self) -> Tuple[bool, Dict[str, Any]]:
        try:
            cache_ttl = float((os.getenv("ATLAS_COMMS_NETWORK_CACHE_S") or "12").strip() or "12")
        except Exception:
            cache_ttl = 12.0
        cache_ttl = max(0.0, min(cache_ttl, 120.0))
        now = time.time()
        with self._net_cache_lock:
            if (
                self._net_cache_ts > 0
                and (now - self._net_cache_ts) <= cache_ttl
                and self._net_cache_diag
            ):
                return bool(self._net_cache_offline), dict(self._net_cache_diag)

        tunnel_up, tunnel_reason = self._cloudflare_tunnel_up()
        net_up, net_reason = self._internet_up()
        offline = not (tunnel_up and net_up)
        diag = {
            "tunnel_up": tunnel_up,
            "tunnel_reason": tunnel_reason,
            "internet_up": net_up,
            "internet_reason": net_reason,
        }
        if offline:
            self._append_snapshot("NETWORK_OFFLINE_MODE", diag)
        with self._net_cache_lock:
            self._net_cache_ts = time.time()
            self._net_cache_offline = bool(offline)
            self._net_cache_diag = dict(diag)
        return offline, diag

    def _detect_urgency(self, message: str, context: Optional[dict] = None) -> Dict[str, Any]:
        urgent = bool(_RE_URGENT.search((message or "").strip()))
        if not urgent and context:
            for value in context.values():
                if isinstance(value, str) and _RE_URGENT.search(value):
                    urgent = True
                    break
        return {"is_urgent": urgent, "level": "critical" if urgent else "normal"}

    def _build_context_summary(
        self, inv: Dict[str, Any], cam: Dict[str, Any], offline_mode: bool
    ) -> Dict[str, Any]:
        sentinel = ((inv.get("inventory") or {}).get("sentinel") or {}).get("data") or {}
        inv_state = sentinel.get("status") or "unknown"
        cam_count = 0
        cam_data = ((cam.get("cameras") or {}).get("data") or {})
        if isinstance(cam_data.get("count"), int):
            cam_count = int(cam_data["count"])
        elif isinstance(cam_data.get("cameras"), list):
            cam_count = len(cam_data["cameras"])
        return {
            "inventory_state": str(inv_state),
            "panaderia_reachable": bool(inv.get("reachable")),
            "vision_reachable": bool(cam.get("reachable")),
            "camera_count": cam_count,
            "offline_mode": bool(offline_mode),
        }

    def _probe_services_cached(self, max_age_s: float = 4.0) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        ttl = max(0.0, float(max_age_s))
        now = time.time()
        with self._probe_cache_lock:
            if (
                self._probe_cache_ts > 0
                and (now - self._probe_cache_ts) <= ttl
                and self._probe_cache_inv
                and self._probe_cache_cam
            ):
                return dict(self._probe_cache_inv), dict(self._probe_cache_cam)

        def _safe_probe(fn, fallback_err: str) -> Dict[str, Any]:
            try:
                data = fn()
                return data if isinstance(data, dict) else {"reachable": False, "error": fallback_err}
            except Exception as e:
                return {"reachable": False, "error": f"{fallback_err}:{e}"}

        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as ex:
            f_inv = ex.submit(_safe_probe, self._probe_panaderia, "probe_panaderia_failed")
            f_cam = ex.submit(_safe_probe, self._probe_vision, "probe_vision_failed")
            try:
                inv = f_inv.result(timeout=8.0)
            except Exception as e:
                inv = {"reachable": False, "error": f"probe_panaderia_timeout:{e}"}
            try:
                cam = f_cam.result(timeout=8.0)
            except Exception as e:
                cam = {"reachable": False, "error": f"probe_vision_timeout:{e}"}

        with self._probe_cache_lock:
            self._probe_cache_ts = time.time()
            self._probe_cache_inv = inv
            self._probe_cache_cam = cam
        return inv, cam

    def _cached_services_or_default(self) -> Tuple[Dict[str, Any], Dict[str, Any], bool]:
        with self._probe_cache_lock:
            if self._probe_cache_inv and self._probe_cache_cam:
                return dict(self._probe_cache_inv), dict(self._probe_cache_cam), True
        inv = {
            "reachable": False,
            "health": {"ok": False, "error": "not_probed_yet"},
            "inventory": {"sentinel": {"data": {"status": "unknown"}}},
        }
        cam = {
            "reachable": False,
            "health": {"ok": False, "error": "not_probed_yet"},
            "cameras": {"data": {"count": 0}},
        }
        return inv, cam, False

    def _recent_user_history(self, user_id: str, limit: int = 4) -> list[Dict[str, str]]:
        uid = (user_id or "").strip()
        if not uid:
            return []
        lim = max(1, min(int(limit or 4), 8))
        conn = self._db_conn()
        try:
            rows = conn.execute(
                """
                SELECT request_summary, response_summary
                FROM comms_messages
                WHERE user_id=?
                ORDER BY created_at DESC
                LIMIT ?
                """,
                (uid, lim),
            ).fetchall()
        finally:
            conn.close()
        out: list[Dict[str, str]] = []
        for r in reversed(rows):
            req = str(r["request_summary"] or "").strip()
            rsp = str(r["response_summary"] or "").strip()
            if req or rsp:
                out.append({"user": req, "assistant": rsp})
        return out

    def _resolve_contact_profile(
        self, user_id: str, context: Optional[Dict[str, Any]]
    ) -> Optional[Dict[str, str]]:
        keys: list[str] = []
        uid = (user_id or "").strip().upper()
        if uid:
            keys.append(uid)
            if ":" not in uid:
                keys.append(f"VISION:{uid}")
        ctx = context if isinstance(context, dict) else {}
        ext = ctx.get("external_user")
        if isinstance(ext, dict):
            src = str(ext.get("source") or "").strip().upper()
            ext_id = str(ext.get("external_user_id") or "").strip().upper()
            if src and ext_id:
                keys.insert(0, f"{src}:{ext_id}")
            elif ext_id:
                keys.insert(0, f"VISION:{ext_id}")
        for k in keys:
            prof = _CONTACT_TONE_PROFILES.get(k)
            if prof:
                return {
                    "key": k,
                    "name": str(prof.get("name") or "").strip(),
                    "tone": str(prof.get("tone") or "").strip(),
                }
        return None

    def _contact_prompt_hint(self, user_id: str, context: Optional[Dict[str, Any]]) -> str:
        prof = self._resolve_contact_profile(user_id, context)
        if not prof:
            return ""
        name = prof.get("name") or "Cliente"
        tone = prof.get("tone") or "cercano y claro"
        return f"Contacto: {name}. Preferencia: {tone}. Usar su nombre en la respuesta."

    def _apply_contact_style(
        self, text: str, user_id: str, context: Optional[Dict[str, Any]]
    ) -> str:
        t = (text or "").strip()
        if not t:
            return t
        prof = self._resolve_contact_profile(user_id, context)
        if not prof:
            return t
        name = str(prof.get("name") or "").strip()
        has_name_greeting = bool(
            name
            and re.match(
                rf"^(hola|buenas|hey|ey)\s+{re.escape(name)}\b",
                t,
                re.IGNORECASE,
            )
        )
        if name and not has_name_greeting and not re.match(rf"^{re.escape(name)}[,:]", t, re.IGNORECASE):
            t = f"{name}, {t}"
        if "paso a paso" not in t.lower() and len(t) < 320:
            if not re.search(r"[.!?]$", t):
                t += "."
            t += " Si quieres, te guio paso a paso."
        return t

    def _extract_latency_ms(self, provider: str) -> Optional[int]:
        p = (provider or "").strip()
        if not p:
            return None
        m = _RE_PROVIDER_LATENCY.search(p)
        if not m:
            return None
        try:
            v = int(m.group("ms"))
        except Exception:
            return None
        if v < 0 or v > 120000:
            return None
        return v

    def _call_clawd_subscription(
        self,
        message: str,
        context_summary: Dict[str, Any],
        conversation_history: Optional[list[Dict[str, str]]] = None,
    ) -> Tuple[bool, str, str]:
        api_url = (os.getenv("ATLAS_CLAWD_API_URL") or "").strip()
        if api_url.lower().startswith("internal://claude-cli"):
            ok, text, provider = self._call_clawd_claude_cli_internal(
                message=message,
                context_summary=context_summary,
                conversation_history=conversation_history,
            )
            if ok:
                with self._clawd_state_lock:
                    self._clawd_fail_streak = 0
                    self._clawd_backoff_until = 0.0
                    self._clawd_last_error = ""
                return True, text, provider
            reason = provider or "clawd_cli_fail"
            base = self._clawd_backoff_base_s()
            with self._clawd_state_lock:
                self._clawd_fail_streak = min(self._clawd_fail_streak + 1, 6)
                factor = min(self._clawd_fail_streak, 3)
                self._clawd_backoff_until = time.time() + (base * factor)
                self._clawd_last_error = reason
            return False, "", reason
        if (not api_url) or api_url.lower().startswith("internal://anthropic"):
            ok, text, provider = self._call_clawd_anthropic_internal(
                message=message,
                context_summary=context_summary,
                conversation_history=conversation_history,
            )
            if ok:
                with self._clawd_state_lock:
                    self._clawd_fail_streak = 0
                    self._clawd_backoff_until = 0.0
                    self._clawd_last_error = ""
                return True, text, provider
            reason = provider or "clawd_internal_fail"
            base = self._clawd_backoff_base_s()
            with self._clawd_state_lock:
                self._clawd_fail_streak = min(self._clawd_fail_streak + 1, 6)
                factor = min(self._clawd_fail_streak, 3)
                self._clawd_backoff_until = time.time() + (base * factor)
                self._clawd_last_error = reason
            return False, "", reason
        now = time.time()
        with self._clawd_state_lock:
            if now < self._clawd_backoff_until:
                remaining = max(0.0, self._clawd_backoff_until - now)
                return False, "", f"clawd_backoff:{remaining:.1f}s"
        token = (
            os.getenv("ATLAS_CLAWD_API_KEY")
            or os.getenv("ATLAS_CENTRAL_CORE")
            or os.getenv("APPROVALS_CHAIN_SECRET")
            or ""
        ).strip()
        headers = {"Authorization": f"Bearer {token}"} if token else None
        payload = {
            "message": message,
            "context": context_summary,
            "persona": "friendly_precise_assistant",
            "history": conversation_history or [],
        }
        timeout_s = self._clawd_timeout_s()
        ok, code, data, err = _http_json(
            "POST", api_url, payload=payload, headers=headers, timeout=timeout_s
        )
        if not ok:
            reason = f"clawd_http_fail:{err or code}"
            base = self._clawd_backoff_base_s()
            with self._clawd_state_lock:
                self._clawd_fail_streak = min(self._clawd_fail_streak + 1, 6)
                factor = min(self._clawd_fail_streak, 3)
                self._clawd_backoff_until = time.time() + (base * factor)
                self._clawd_last_error = reason
            return False, "", f"clawd_http_fail:{err or code}"
        text = (
            data.get("reply")
            or data.get("response")
            or data.get("text")
            or ((data.get("data") or {}).get("reply") if isinstance(data.get("data"), dict) else "")
            or ""
        )
        if not text:
            reason = "clawd_empty_reply"
            base = self._clawd_backoff_base_s()
            with self._clawd_state_lock:
                self._clawd_fail_streak = min(self._clawd_fail_streak + 1, 6)
                factor = min(self._clawd_fail_streak, 3)
                self._clawd_backoff_until = time.time() + (base * factor)
                self._clawd_last_error = reason
            return False, "", "clawd_empty_reply"
        with self._clawd_state_lock:
            self._clawd_fail_streak = 0
            self._clawd_backoff_until = 0.0
            self._clawd_last_error = ""
        return True, str(text).strip(), "clawd_api"

    def _call_clawd_anthropic_internal(
        self,
        message: str,
        context_summary: Dict[str, Any],
        conversation_history: Optional[list[Dict[str, str]]] = None,
    ) -> Tuple[bool, str, str]:
        try:
            from modules.humanoid.ai.external_llm import call_external
        except Exception as e:
            return False, "", f"clawd_internal_module_missing:{e}"

        api_key = (
            os.getenv("ATLAS_CLAWD_API_KEY")
            or os.getenv("ANTHROPIC_API_KEY")
            or os.getenv("ANTHROPIC_API_KEY_CEREBRO")
            or ""
        ).strip()
        if not api_key:
            try:
                from modules.humanoid.ai.provider_credentials import get_provider_api_key

                api_key = (get_provider_api_key("anthropic") or "").strip()
            except Exception:
                api_key = ""
        if not api_key:
            return False, "", "clawd_internal_anthropic_key_missing"

        model = (
            os.getenv("ATLAS_CLAWD_MODEL")
            or os.getenv("ANTHROPIC_MODEL")
            or "claude-sonnet-4-latest"
        ).strip()
        timeout_s = self._clawd_timeout_s()
        system = (
            "Eres ATLAS, asistente operativo de RAULI. "
            "Responde en espanol natural, breve y accionable."
        )
        prompt = self._build_ai_prompt(
            message=message,
            context_summary=context_summary,
            conversation_history=conversation_history,
            contact_hint="",
        )
        ok, text, latency_ms = call_external(
            "anthropic",
            model,
            prompt,
            system,
            api_key,
            timeout_s=timeout_s,
        )
        if not ok:
            return False, "", f"clawd_internal_anthropic_fail:{_truncate(str(text), 160)}"
        out = str(text or "").strip()
        if not out:
            return False, "", "clawd_internal_anthropic_empty_reply"
        return True, out, f"clawd_api:anthropic:{model}:{int(latency_ms)}ms"

    def _call_clawd_claude_cli_internal(
        self,
        message: str,
        context_summary: Dict[str, Any],
        conversation_history: Optional[list[Dict[str, str]]] = None,
    ) -> Tuple[bool, str, str]:
        cli_bin = _resolve_claude_cli_bin()
        if not cli_bin:
            return False, "", "clawd_cli_missing"
        timeout_s = self._clawd_timeout_s()
        system = (
            "Eres ATLAS, asistente operativo de RAULI. "
            "Responde en espanol natural, breve y accionable."
        )
        prompt = self._build_ai_prompt(
            message=message,
            context_summary=context_summary,
            conversation_history=conversation_history,
            contact_hint="",
        )
        cmd = [
            cli_bin,
            "--print",
            "--input-format",
            "text",
            "--output-format",
            "text",
            "--system-prompt",
            system,
        ]
        suffix = Path(cli_bin).suffix.lower()
        if suffix in {".cmd", ".bat"}:
            cmd = ["cmd.exe", "/c", *cmd]
        try:
            proc = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                input=prompt,
                timeout=timeout_s,
            )
        except FileNotFoundError:
            return False, "", "clawd_cli_missing"
        except subprocess.TimeoutExpired:
            return False, "", "clawd_cli_timeout"
        except Exception as exc:
            return False, "", f"clawd_cli_error:{_truncate(str(exc), 120)}"
        if proc.returncode != 0:
            err = (proc.stderr or "").strip() or "clawd_cli_failed"
            return False, "", f"clawd_cli_fail:{_truncate(err, 160)}"
        out = (proc.stdout or "").strip()
        if not out:
            return False, "", "clawd_cli_empty_reply"
        return True, out, "clawd_cli"

    def _build_ai_prompt(
        self,
        message: str,
        context_summary: Dict[str, Any],
        conversation_history: Optional[list[Dict[str, str]]] = None,
        contact_hint: str = "",
    ) -> str:
        history_lines = []
        for idx, item in enumerate((conversation_history or [])[-4:], start=1):
            u = _truncate(str((item or {}).get("user") or ""), 140)
            a = _truncate(str((item or {}).get("assistant") or ""), 180)
            if u or a:
                history_lines.append(f"{idx}. Usuario: {u}\n   ATLAS: {a}")
        history_text = "\n".join(history_lines) if history_lines else "Sin historial reciente."
        contact_block = f"\nPerfil del contacto:\n- {contact_hint}\n" if contact_hint else ""
        return (
            "Contexto operativo actual:\n"
            f"- inventario_state: {context_summary.get('inventory_state')}\n"
            f"- camaras_activas: {int(context_summary.get('camera_count') or 0)}\n"
            f"- panaderia_reachable: {bool(context_summary.get('panaderia_reachable'))}\n"
            f"- vision_reachable: {bool(context_summary.get('vision_reachable'))}\n"
            f"- offline_mode: {bool(context_summary.get('offline_mode'))}\n\n"
            f"Historial reciente:\n{history_text}\n"
            f"{contact_block}\n"
            f"Mensaje actual del usuario:\n{(message or '').strip()}\n\n"
            "Responde en espanol natural y cercano, con precision operativa.\n"
            "No inventes datos: si falta informacion, dilo y pide un dato puntual.\n"
            "Se breve (maximo 5 lineas) y accionable."
        )

    def _local_auto_enabled(self) -> bool:
        raw = (os.getenv("ATLAS_COMMS_LOCAL_AUTO_ENABLED") or "true").strip().lower()
        return raw in ("1", "true", "yes", "y", "on")

    def _classify_local_route(
        self,
        message: str,
        context: Optional[Dict[str, Any]] = None,
        offline_mode: bool = False,
    ) -> Tuple[bool, str, str]:
        msg = (message or "").strip()
        ctx = context if isinstance(context, dict) else {}
        source = str(
            ctx.get("source_app") or ctx.get("source") or ctx.get("channel") or ""
        ).strip().lower()
        if offline_mode:
            return True, "FAST", "offline_local"
        if source.startswith("rauli-panaderia") or "panaderia" in source:
            if _RE_FAST_UI.search(msg):
                return True, "FAST", "panaderia_ui"
            if _RE_PANADERIA_TOOLS.search(msg):
                if _RE_REASONING.search(msg):
                    return True, "REASON", "panaderia_ops_reason"
                return True, "TOOLS", "panaderia_ops"
            return False, "CHAT", "panaderia_general"
        if source.startswith("rauli-vision") or "vision" in source:
            if str(ctx.get("context_url") or "").strip():
                return True, "REASON", "vision_url"
            if _RE_VISION_LOCAL.search(msg):
                if _RE_REASONING.search(msg):
                    return True, "REASON", "vision_reason"
                return True, "CHAT", "vision_chat"
            return False, "CHAT", "vision_general"
        if _RE_FAST_UI.search(msg):
            return True, "FAST", "fast_ui"
        if _RE_REASONING.search(msg):
            return True, "REASON", "reasoning"
        return False, "CHAT", "general"

    def _call_local_auto(
        self,
        message: str,
        context_summary: Dict[str, Any],
        conversation_history: Optional[list[Dict[str, str]]] = None,
        *,
        route_hint: str = "CHAT",
        user_id: str = "",
        context: Optional[Dict[str, Any]] = None,
    ) -> Tuple[bool, str, str]:
        try:
            from modules.humanoid.ai.router import (
                _call_ollama,
                decide_route,
                infer_task_profile,
            )
        except Exception as e:
            return False, "", f"local_auto_module_missing:{_truncate(str(e), 120)}"

        try:
            timeout_s = int((os.getenv("ATLAS_COMMS_LOCAL_TIMEOUT_S") or "35").strip() or "35")
        except Exception:
            timeout_s = 35
        timeout_s = max(8, min(timeout_s, 120))
        contact_hint = self._contact_prompt_hint(user_id, context)
        prompt = self._build_ai_prompt(
            message=message,
            context_summary=context_summary,
            conversation_history=conversation_history,
            contact_hint=contact_hint,
        )
        profile = infer_task_profile(prompt, intent_hint=route_hint, modality="text")
        decision = decide_route(profile, prefer_free=True)
        model_key = str(decision.model_key or "").strip()
        provider_id = str(decision.provider_id or "").strip().lower()
        if provider_id != "ollama" or ":" not in model_key:
            return False, "", f"local_auto_unavailable:{decision.reason}"
        model_name = model_key.split(":", 1)[1].strip()
        system = (
            "Eres ATLAS, asistente operativo de RAULI. "
            "Responde en espanol natural, breve, preciso y accionable. "
            "Si el usuario esta registrando eventos operativos, pide solo el siguiente dato necesario."
        )
        ok, out, latency_ms = _call_ollama(model_name, prompt, system, timeout_s)
        if not ok:
            return False, "", f"local_auto_fail:{model_name}:{_truncate(out, 140)}"
        final = self._apply_contact_style(str(out or "").strip(), user_id, context)
        if not final:
            return False, "", f"local_auto_empty:{model_name}"
        return True, final, f"local_auto:{model_name}:{route_hint}:{int(latency_ms)}ms"

    def _call_openai_fallback(
        self,
        message: str,
        context_summary: Dict[str, Any],
        conversation_history: Optional[list[Dict[str, str]]] = None,
        user_id: str = "",
        context: Optional[Dict[str, Any]] = None,
    ) -> Tuple[bool, str, str]:
        try:
            from modules.humanoid.ai.external_llm import call_external
        except Exception as e:
            return False, "", f"openai_module_missing:{e}"

        api_key = (os.getenv("OPENAI_API_KEY") or "").strip()
        if not api_key:
            try:
                from modules.humanoid.ai.provider_credentials import get_provider_api_key

                api_key = (get_provider_api_key("openai") or "").strip()
            except Exception:
                api_key = ""
        if not api_key:
            return False, "", "openai_api_key_missing"

        model = (os.getenv("OPENAI_MODEL") or "gpt-4.1-mini").strip()
        try:
            timeout_s = int((os.getenv("ATLAS_COMMS_OPENAI_TIMEOUT_S") or "10").strip() or "10")
        except Exception:
            timeout_s = 10
        timeout_s = max(4, min(timeout_s, 45))

        system = (
            "Eres ATLAS, asistente operativo de RAULI. "
            "Responde en espanol claro, humano y profesional. "
            "Prioriza acciones concretas y velocidad."
        )
        contact_hint = self._contact_prompt_hint(user_id, context)
        prompt = self._build_ai_prompt(
            message=message,
            context_summary=context_summary,
            conversation_history=conversation_history,
            contact_hint=contact_hint,
        )
        ok, text, latency_ms = call_external(
            "openai",
            model,
            prompt,
            system,
            api_key,
            timeout_s=timeout_s,
        )
        if not ok:
            return False, "", f"openai_http_fail:{_truncate(text or 'unknown', 160)}"
        final = str(text or "").strip()
        if not final:
            return False, "", "openai_empty_reply"
        final = self._apply_contact_style(final, user_id, context)
        return True, final, f"openai:{model}:{int(latency_ms)}ms"

    def _call_bedrock_fallback(
        self,
        message: str,
        context_summary: Dict[str, Any],
        conversation_history: Optional[list[Dict[str, str]]] = None,
        user_id: str = "",
        context: Optional[Dict[str, Any]] = None,
    ) -> Tuple[bool, str, str]:
        region = (
            os.getenv("ATLAS_BEDROCK_REGION")
            or os.getenv("AWS_REGION")
            or os.getenv("AWS_DEFAULT_REGION")
            or "us-east-1"
        ).strip()
        model = (
            os.getenv("ATLAS_COMMS_BEDROCK_MODEL")
            or os.getenv("ATLAS_BEDROCK_BALANCED_MODEL")
            or os.getenv("ATLAS_MODEL_FAST")
            or "us.anthropic.claude-haiku-4-5-20251001-v1:0"
        ).strip()
        try:
            timeout_s = int((os.getenv("ATLAS_COMMS_BEDROCK_TIMEOUT_S") or "12").strip() or "12")
        except Exception:
            timeout_s = 12
        timeout_s = max(4, min(timeout_s, 45))
        system = (
            "Eres ATLAS, asistente operativo de RAULI. "
            "Responde en espanol claro, humano y accionable. "
            "Si falta un dato para operar, pide solo el siguiente dato necesario."
        )
        contact_hint = self._contact_prompt_hint(user_id, context)
        prompt = self._build_ai_prompt(
            message=message,
            context_summary=context_summary,
            conversation_history=conversation_history,
            contact_hint=contact_hint,
        )
        try:
            import boto3
            from botocore.config import Config as BotoConfig
        except Exception as e:
            return False, "", f"bedrock_module_missing:{_truncate(str(e), 120)}"
        try:
            client = boto3.client(
                "bedrock-runtime",
                region_name=region,
                config=BotoConfig(connect_timeout=timeout_s, read_timeout=timeout_s),
            )
            body = {
                "anthropic_version": "bedrock-2023-05-31",
                "max_tokens": 512,
                "system": system,
                "messages": [{"role": "user", "content": [{"type": "text", "text": prompt}]}],
            }
            t0 = time.perf_counter()
            response = client.invoke_model(
                modelId=model,
                body=json.dumps(body).encode("utf-8"),
                contentType="application/json",
                accept="application/json",
            )
            latency_ms = (time.perf_counter() - t0) * 1000
            payload = json.loads(response["body"].read())
            content = payload.get("content") or []
            chunks = [str(block.get("text") or "").strip() for block in content if isinstance(block, dict)]
            final = "\n".join([part for part in chunks if part]).strip()
            if not final:
                return False, "", "bedrock_empty_reply"
            final = self._apply_contact_style(final, user_id, context)
            return True, final, f"bedrock:{model}:{int(latency_ms)}ms"
        except Exception as e:
            return False, "", f"bedrock_http_fail:{_truncate(str(e), 160)}"

    def _offline_fallback_reply(self, message: str, context_summary: Dict[str, Any]) -> str:
        msg = (message or "").strip()
        inv_state = context_summary.get("inventory_state") or "unknown"
        cam_count = int(context_summary.get("camera_count") or 0)
        offline = bool(context_summary.get("offline_mode"))
        if _RE_HOURS.search(msg):
            base = f"Claro. El horario estimado es {os.getenv('ATLAS_STORE_SCHEDULE') or DEFAULT_SCHEDULE}."
            if offline:
                base += " Ahora estoy en modo offline; cuando vuelva el tunel te confirmo cualquier cambio."
            base += " Si quieres, te confirmo una franja exacta."
            return base
        if _RE_STOCK.search(msg):
            return (
                f"Te confirmo el estado actual: inventario {inv_state} y {cam_count} camaras activas. "
                "Si me dices producto y cantidad, te respondo puntual."
            )
        if _RE_ORDER.search(msg):
            return (
                "Perfecto, te ayudo con el pedido. "
                + DEFAULT_ORDER_GUIDE
                + f" Inventario: {inv_state}. Camaras activas: {cam_count}."
            )
        return (
            "Te leo. Ya revise inventario y camaras antes de responderte. "
            f"Inventario: {inv_state}. Camaras activas: {cam_count}. "
            "Si quieres, dime el siguiente paso y lo hacemos."
        )

    def _naturalize_reply(self, text: str) -> str:
        t = (text or "").strip()
        if not t:
            return "Te leo. Dame un momento y te respondo."
        t = re.sub(r"\s+", " ", t).strip()
        if len(t) > 900:
            t = t[:897].rstrip() + "..."
        if not re.search(r"[.!?]$", t):
            t += "."
        return t

    def _generate_reply(
        self,
        message: str,
        context_summary: Dict[str, Any],
        offline_mode: bool,
        conversation_history: Optional[list[Dict[str, str]]] = None,
        user_id: str = "",
        context: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        local_preferred, local_route, _local_reason = self._classify_local_route(
            message,
            context=context,
            offline_mode=offline_mode,
        )
        local_tried = False
        if self._local_auto_enabled() and (offline_mode or local_preferred):
            local_tried = True
            ok, text, provider = self._call_local_auto(
                message,
                context_summary,
                conversation_history=conversation_history,
                route_hint=local_route,
                user_id=user_id,
                context=context,
            )
            if ok:
                return {
                    "text": self._apply_contact_style(
                        self._naturalize_reply(text), user_id, context
                    ),
                    "provider": provider,
                    "offline": bool(offline_mode),
                }
        if not offline_mode:
            ok, text, provider = self._call_clawd_subscription(
                message, context_summary, conversation_history=conversation_history
            )
            if ok:
                return {
                    "text": self._apply_contact_style(
                        self._naturalize_reply(text), user_id, context
                    ),
                    "provider": provider,
                    "offline": False,
                }
            if self._local_auto_enabled() and not local_tried:
                ok, text, provider = self._call_local_auto(
                    message,
                    context_summary,
                    conversation_history=conversation_history,
                    route_hint=local_route,
                    user_id=user_id,
                    context=context,
                )
                if ok:
                    return {
                        "text": self._apply_contact_style(
                            self._naturalize_reply(text), user_id, context
                        ),
                        "provider": provider,
                        "offline": False,
                    }
            ok, text, provider = self._call_bedrock_fallback(
                message,
                context_summary,
                conversation_history=conversation_history,
                user_id=user_id,
                context=context,
            )
            if ok:
                return {
                    "text": self._apply_contact_style(
                        self._naturalize_reply(text), user_id, context
                    ),
                    "provider": provider,
                    "offline": False,
                }
            ok, text, provider = self._call_openai_fallback(
                message,
                context_summary,
                conversation_history=conversation_history,
                user_id=user_id,
                context=context,
            )
            if ok:
                return {
                    "text": self._apply_contact_style(
                        self._naturalize_reply(text), user_id, context
                    ),
                    "provider": provider,
                    "offline": False,
                }
        return {
            "text": self._apply_contact_style(
                self._naturalize_reply(
                    self._offline_fallback_reply(message, context_summary)
                ),
                user_id,
                context,
            ),
            "provider": "offline_fallback",
            "offline": True,
        }

    def _emit_ops_critical(self, text: str, data: Optional[Dict[str, Any]] = None) -> None:
        try:
            from modules.humanoid.comms import emit as comms_emit

            comms_emit("atlas_comms_hub", text, level="critical", data=data or {})
        except Exception:
            pass

    def _send_urgent_alert(self, message_id: str, user_id: str, message: str, diag: Dict[str, Any]) -> Dict[str, Any]:
        payload = {
            "event": "atlas_urgent_client_message",
            "message_id": message_id,
            "user_id": user_id,
            "message": _truncate(message, 1200),
            "diag": diag,
            "ts": _utc_now(),
        }
        alert_url = (os.getenv("ATLAS_CLOUDFLARE_ALERT_URL") or "").strip()
        if alert_url:
            token = (
                os.getenv("ATLAS_CENTRAL_CORE")
                or os.getenv("ATLAS_CLAWD_API_KEY")
                or os.getenv("APPROVALS_CHAIN_SECRET")
                or ""
            ).strip()
            headers = {"X-Atlas-Core": token} if token else None
            ok, code, _data, err = _http_json(
                "POST", alert_url, payload=payload, headers=headers, timeout=8.0
            )
            if ok:
                self._append_snapshot(
                    "ATLAS_COMMS_URGENT_ALERT",
                    {"message_id": message_id, "channel": "cloudflare", "code": code},
                )
                return {"ok": True, "channel": "cloudflare", "code": code}
            self._emit_ops_critical(
                "Urgent customer issue detected but cloudflare alert failed.",
                {"message_id": message_id, "error": err, "http_code": code},
            )
            return {"ok": False, "channel": "cloudflare", "error": err, "code": code}
        self._emit_ops_critical(
            "Urgent customer issue detected (no ATLAS_CLOUDFLARE_ALERT_URL configured).",
            {"message_id": message_id, "user_id": user_id, "message": _truncate(message, 260)},
        )
        self._append_snapshot(
            "ATLAS_COMMS_URGENT_ALERT",
            {"message_id": message_id, "channel": "ops_fallback", "configured": False},
        )
        return {"ok": True, "channel": "ops_fallback"}

    def _store_message(
        self,
        *,
        message_id: str,
        user_id: str,
        channel: str,
        urgency: str,
        request_text: str,
        response_text: str,
        inventory_status: str,
        camera_status: str,
        offline_mode: bool,
        synced: bool,
        metadata: Dict[str, Any],
    ) -> None:
        request_enc = self._encrypt_text(request_text)
        response_enc = self._encrypt_text(response_text)
        conn = self._db_conn()
        try:
            conn.execute(
                """
                INSERT INTO comms_messages (
                    message_id, created_at, user_id, channel, urgency,
                    request_summary, request_encrypted,
                    response_summary, response_encrypted,
                    inventory_status, camera_status,
                    offline_mode, synced, metadata_json
                ) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?)
                """,
                (
                    message_id,
                    _utc_now(),
                    user_id,
                    channel,
                    urgency,
                    _truncate(request_text, 200),
                    request_enc,
                    _truncate(response_text, 200),
                    response_enc,
                    inventory_status,
                    camera_status,
                    1 if offline_mode else 0,
                    1 if synced else 0,
                    _json_dump(metadata),
                ),
            )
            conn.commit()
        finally:
            conn.close()

    def _enqueue_offline(self, message_id: str, payload: Dict[str, Any]) -> None:
        payload_enc = self._encrypt_text(_json_dump(payload))
        conn = self._db_conn()
        try:
            conn.execute(
                """
                INSERT INTO comms_offline_queue (
                    message_id, payload_encrypted, created_at, status, attempts
                ) VALUES (?,?,?,?,?)
                """,
                (message_id, payload_enc, _utc_now(), "pending", 0),
            )
            conn.commit()
        finally:
            conn.close()

    def process_user_interaction(
        self,
        *,
        user_id: str,
        channel: str,
        message: str,
        context: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        msg = (message or "").strip()
        if not msg:
            return {"ok": False, "error": "message_empty"}

        with self._lock:
            normalized_user = (user_id or "guest").strip() or "guest"
            message_id = f"atlas-comms-{uuid.uuid4().hex[:12]}"
            try:
                probe_ttl = float((os.getenv("ATLAS_COMMS_PROBE_CACHE_S") or "12").strip() or "12")
            except Exception:
                probe_ttl = 12.0
            needs_live_context = bool(
                _RE_STOCK.search(msg)
                or _RE_ORDER.search(msg)
                or _RE_HOURS.search(msg)
                or _RE_CAMERA.search(msg)
                or bool((context or {}).get("force_live_probe"))
            )
            if needs_live_context:
                inv, cam = self._probe_services_cached(max_age_s=probe_ttl)
            else:
                inv, cam, from_cache = self._cached_services_or_default()
                if not from_cache:
                    # Warm up probes in background for next interactions, without blocking this reply.
                    try:
                        threading.Thread(
                            target=lambda: self._probe_services_cached(max_age_s=0.0),
                            daemon=True,
                        ).start()
                    except Exception:
                        pass
            offline_mode, offline_diag = self._is_offline_mode()
            urgency = self._detect_urgency(msg, context=context)
            context_summary = self._build_context_summary(inv, cam, offline_mode=offline_mode)
            history = self._recent_user_history(normalized_user, limit=4)
            reply = self._generate_reply(
                msg,
                context_summary=context_summary,
                offline_mode=offline_mode,
                conversation_history=history,
                user_id=normalized_user,
                context=context,
            )

            inventory_state = str(context_summary.get("inventory_state") or "unknown")
            camera_state = "ok" if bool(context_summary.get("camera_count")) else "unknown"
            metadata = {
                "context": context or {},
                "provider": reply.get("provider"),
                "offline_diag": offline_diag,
                "contact_profile": self._resolve_contact_profile(normalized_user, context),
            }
            synced = not offline_mode
            self._store_message(
                message_id=message_id,
                user_id=normalized_user,
                channel=(channel or "app").strip() or "app",
                urgency=urgency.get("level", "normal"),
                request_text=msg,
                response_text=str(reply.get("text") or ""),
                inventory_status=inventory_state,
                camera_status=camera_state,
                offline_mode=offline_mode,
                synced=synced,
                metadata=metadata,
            )

            if offline_mode:
                self._enqueue_offline(
                    message_id,
                    {
                        "message_id": message_id,
                        "user_id": user_id,
                        "channel": channel,
                        "request": msg,
                        "response": reply.get("text"),
                        "summary": _truncate(msg, 120),
                        "ts": _utc_now(),
                    },
                )

            alert_result = {"ok": False, "skipped": True}
            if bool(urgency.get("is_urgent")):
                alert_result = self._send_urgent_alert(message_id, (user_id or "guest"), msg, offline_diag)

            self._append_snapshot(
                "ATLAS_COMMS_SUMMARY",
                {
                    "message_id": message_id,
                    "user_id": user_id,
                    "channel": channel,
                    "urgency": urgency.get("level"),
                    "offline_mode": offline_mode,
                    "inventory_state": inventory_state,
                    "camera_state": camera_state,
                    "provider": reply.get("provider"),
                },
            )

            return {
                "ok": True,
                "message_id": message_id,
                "reply": reply.get("text"),
                "provider": reply.get("provider"),
                "offline_mode": offline_mode,
                "urgency": urgency,
                "alert": alert_result,
                "context": context_summary,
                "queue_enqueued": bool(offline_mode),
            }

    def get_history(self, limit: int = 30, decrypt: bool = True) -> Dict[str, Any]:
        lim = max(1, min(int(limit or 30), 200))
        conn = self._db_conn()
        try:
            rows = conn.execute(
                """
                SELECT message_id, created_at, user_id, channel, urgency,
                       request_summary, request_encrypted,
                       response_summary, response_encrypted,
                       inventory_status, camera_status,
                       offline_mode, synced, metadata_json
                FROM comms_messages
                ORDER BY created_at DESC
                LIMIT ?
                """,
                (lim,),
            ).fetchall()
        finally:
            conn.close()

        items = []
        for r in rows:
            req_preview = r["request_summary"] or ""
            rsp_preview = r["response_summary"] or ""
            metadata: Dict[str, Any] = {}
            try:
                raw_meta = r["metadata_json"]
                if raw_meta:
                    parsed = json.loads(raw_meta)
                    if isinstance(parsed, dict):
                        metadata = parsed
            except Exception:
                metadata = {}
            provider = str(metadata.get("provider") or "").strip()
            latency_ms = self._extract_latency_ms(provider)
            contact_profile = metadata.get("contact_profile")
            contact_name = ""
            if isinstance(contact_profile, dict):
                contact_name = str(contact_profile.get("name") or "").strip()
            if decrypt:
                try:
                    req_preview = _truncate(self._decrypt_text(r["request_encrypted"]), 260)
                except Exception:
                    pass
                try:
                    rsp_preview = _truncate(self._decrypt_text(r["response_encrypted"]), 260)
                except Exception:
                    pass
            items.append(
                {
                    "message_id": r["message_id"],
                    "created_at": r["created_at"],
                    "user_id": r["user_id"],
                    "channel": r["channel"],
                    "urgency": r["urgency"],
                    "request": req_preview,
                    "response": rsp_preview,
                    "inventory_status": r["inventory_status"],
                    "camera_status": r["camera_status"],
                    "offline_mode": bool(r["offline_mode"]),
                    "synced": bool(r["synced"]),
                    "provider": provider,
                    "latency_ms": latency_ms,
                    "contact_name": contact_name,
                }
            )
        latencies = [int(it["latency_ms"]) for it in items if isinstance(it.get("latency_ms"), int)]
        metrics = {
            "latency_count": len(latencies),
            "avg_ms": round((sum(latencies) / len(latencies)), 1) if latencies else None,
            "p95_ms": _percentile_int(latencies, 95),
            "p99_ms": _percentile_int(latencies, 99),
        }
        return {"ok": True, "items": items, "count": len(items), "metrics": metrics}

    def get_status(self) -> Dict[str, Any]:
        conn = self._db_conn()
        try:
            pending = int(
                conn.execute(
                    "SELECT COUNT(*) FROM comms_offline_queue WHERE status='pending'"
                ).fetchone()[0]
            )
            total = int(conn.execute("SELECT COUNT(*) FROM comms_messages").fetchone()[0])
            ext_total = int(conn.execute("SELECT COUNT(*) FROM comms_external_users").fetchone()[0])
            ext_rows = conn.execute(
                """
                SELECT whatsapp_state, COUNT(*) AS total
                FROM comms_external_users
                GROUP BY whatsapp_state
                """
            ).fetchall()
            row = conn.execute(
                "SELECT created_at FROM comms_messages ORDER BY created_at DESC LIMIT 1"
            ).fetchone()
        finally:
            conn.close()
        ext_summary: Dict[str, int] = {"total": ext_total, "missing": 0, "requested": 0, "linked": 0}
        for r in ext_rows:
            key = str(r["whatsapp_state"] or "missing")
            ext_summary[key] = int(r["total"] or 0)
        offline_mode, offline_diag = self._is_offline_mode()
        with self._clawd_state_lock:
            clawd_backoff_until = self._clawd_backoff_until
            clawd_fail_streak = self._clawd_fail_streak
            clawd_last_error = self._clawd_last_error
        _secret, source = self._encryption_secret()
        clawd_backoff_utc = (
            datetime.fromtimestamp(clawd_backoff_until, tz=timezone.utc).isoformat()
            if clawd_backoff_until > 0
            else None
        )
        return {
            "ok": True,
            "db_path": str(DB_FILE),
            "messages_total": total,
            "queue_pending": pending,
            "last_message_at": (row[0] if row else None),
            "offline_mode": offline_mode,
            "offline_diag": offline_diag,
            "encryption_source": source,
            "panaderia_base": PANADERIA_BASE,
            "vision_base": VISION_BASE,
            "primary_channel": "whatsapp",
            "external_users": ext_summary,
            "clawd_subscription": {
                "api_url_configured": bool((os.getenv("ATLAS_CLAWD_API_URL") or "").strip()),
                "api_url": (os.getenv("ATLAS_CLAWD_API_URL") or "").strip(),
                "fail_streak": int(clawd_fail_streak),
                "backoff_until_utc": clawd_backoff_utc,
                "last_error": _truncate(clawd_last_error, 160),
            },
        }

    def _send_sync_payload(self, payload: Dict[str, Any]) -> Tuple[bool, str]:
        sync_url = (os.getenv("ATLAS_CLOUDFLARE_SYNC_URL") or "").strip()
        token = (
            os.getenv("ATLAS_CENTRAL_CORE") or os.getenv("APPROVALS_CHAIN_SECRET") or ""
        ).strip()
        if sync_url:
            headers = {"X-Atlas-Core": token} if token else None
            ok, code, _data, err = _http_json(
                "POST",
                sync_url,
                payload=payload,
                headers=headers,
                timeout=8.0,
            )
            if ok:
                return True, f"sync_url_ok:{code}"
            return False, f"sync_url_fail:{err or code}"

        ok, code, _data, err = _http_json(
            "POST",
            f"{PUSH_BASE}/api/clawd/bridge/evidence",
            payload={
                "source": "atlas_comms_hub",
                "evidence_type": "offline_resync",
                "message": f"Resynced message {payload.get('message_id')}",
                "payload": payload,
                "token": token or None,
            },
            timeout=8.0,
        )
        if ok:
            return True, f"local_bridge_ok:{code}"
        return False, f"local_bridge_fail:{err or code}"

    def _mark_queue_synced(self, queue_id: int, message_id: str) -> None:
        conn = self._db_conn()
        try:
            conn.execute(
                """
                UPDATE comms_offline_queue
                SET status='synced', dequeued_at=?, last_error=NULL
                WHERE id=?
                """,
                (_utc_now(), int(queue_id)),
            )
            conn.execute(
                """
                UPDATE comms_messages
                SET synced=1, sync_attempts=sync_attempts+1, last_error=NULL
                WHERE message_id=?
                """,
                (message_id,),
            )
            conn.commit()
        finally:
            conn.close()

    def _mark_queue_error(
        self,
        queue_id: int,
        attempts: int,
        error: str,
        *,
        terminal: bool = False,
    ) -> None:
        max_attempts = max(1, int((os.getenv("ATLAS_COMMS_RESYNC_MAX_ATTEMPTS") or "5").strip() or "5"))
        should_fail = bool(terminal) or int(attempts) >= max_attempts
        conn = self._db_conn()
        try:
            conn.execute(
                """
                UPDATE comms_offline_queue
                SET attempts=?,
                    last_error=?,
                    status=CASE WHEN ? THEN 'failed' ELSE status END,
                    dequeued_at=CASE WHEN ? THEN ? ELSE dequeued_at END
                WHERE id=?
                """,
                (
                    int(attempts),
                    _truncate(error, 400),
                    1 if should_fail else 0,
                    1 if should_fail else 0,
                    _utc_now() if should_fail else None,
                    int(queue_id),
                ),
            )
            conn.execute(
                """
                UPDATE comms_messages
                SET sync_attempts=sync_attempts+1,
                    last_error=?,
                    synced=CASE WHEN ? THEN 1 ELSE synced END
                WHERE message_id=(SELECT message_id FROM comms_offline_queue WHERE id=?)
                """,
                (_truncate(error, 400), 1 if should_fail else 0, int(queue_id)),
            )
            conn.commit()
        finally:
            conn.close()

    def resync_pending(self, limit: int = 100) -> Dict[str, Any]:
        offline_mode, diag = self._is_offline_mode()
        if offline_mode:
            return {
                "ok": False,
                "error": "offline_mode_active",
                "diag": diag,
                "processed": 0,
                "synced": 0,
            }
        lim = max(1, min(int(limit or 100), 400))
        conn = self._db_conn()
        try:
            rows = conn.execute(
                """
                SELECT id, message_id, payload_encrypted, attempts
                FROM comms_offline_queue
                WHERE status='pending'
                ORDER BY created_at ASC
                LIMIT ?
                """,
                (lim,),
            ).fetchall()
        finally:
            conn.close()

        processed = 0
        synced = 0
        errors = []
        for row in rows:
            processed += 1
            qid = int(row["id"])
            mid = str(row["message_id"])
            attempts = int(row["attempts"] or 0)
            try:
                payload = json.loads(self._decrypt_text(row["payload_encrypted"]) or "{}")
            except Exception as e:
                self._mark_queue_error(qid, attempts + 1, f"decrypt_fail:{e}", terminal=True)
                errors.append({"id": qid, "error": f"decrypt_fail:{e}"})
                continue
            ok, reason = self._send_sync_payload(payload)
            if ok:
                synced += 1
                self._mark_queue_synced(qid, mid)
            else:
                self._mark_queue_error(qid, attempts + 1, reason)
                errors.append({"id": qid, "error": reason})

        self._append_snapshot(
            "ATLAS_COMMS_RESYNC",
            {"processed": processed, "synced": synced, "errors": len(errors)},
        )
        return {"ok": True, "processed": processed, "synced": synced, "errors": errors[:50]}

    def _run_snapshot_validation(self) -> Dict[str, Any]:
        if not SNAPSHOT_SCRIPT.exists():
            return {"ok": False, "stable": False, "error": f"missing_script:{SNAPSHOT_SCRIPT}"}
        try:
            proc = subprocess.run(
                [
                    "powershell",
                    "-NoProfile",
                    "-ExecutionPolicy",
                    "Bypass",
                    "-File",
                    str(SNAPSHOT_SCRIPT),
                ],
                cwd=str(ROOT_DIR),
                capture_output=True,
                text=True,
                timeout=240,
            )
        except Exception as e:
            return {"ok": False, "stable": False, "error": f"snapshot_exec_fail:{e}"}

        bridge_ok = False
        stable = False
        checks: Dict[str, Any] = {}
        try:
            from tools.atlas_clawd_bridge.bridge import get_bridge

            bridge = get_bridge()
            st = bridge.ensure_stable(run_snapshot=False, raise_on_unstable=False)
            status = st.get("status") or {}
            bridge_ok = bool(status.get("ok"))
            stable = bool(status.get("stable"))
            checks = status.get("checks") or {}
        except Exception:
            out = f"{proc.stdout}\n{proc.stderr}".lower()
            stable = proc.returncode == 0 and "snapshot_safe_end" in out
            checks = {"fallback_parse": stable}

        return {
            "ok": proc.returncode == 0,
            "stable": stable,
            "returncode": proc.returncode,
            "bridge_ok": bridge_ok,
            "checks": checks,
            "stdout_tail": _truncate(proc.stdout or "", 500),
            "stderr_tail": _truncate(proc.stderr or "", 500),
        }

    def _validate_inventory_payload(self, payload: Dict[str, Any]) -> Tuple[bool, str]:
        required = {"product_id", "quantity", "type", "notes"}
        missing = [k for k in required if k not in payload]
        if missing:
            return False, f"missing_fields:{missing}"
        if not isinstance(payload.get("product_id"), str) or not payload.get("product_id"):
            return False, "invalid_product_id"
        try:
            float(payload.get("quantity"))
        except Exception:
            return False, "invalid_quantity"
        if str(payload.get("type")) not in {"entrada", "salida", "ajuste", "merma"}:
            return False, "invalid_type"
        if not isinstance(payload.get("notes"), str):
            return False, "invalid_notes"
        if INVENTORY_SCHEMA_FILE.exists():
            try:
                schema = json.loads(INVENTORY_SCHEMA_FILE.read_text(encoding="utf-8"))
                allowed = set((schema.get("properties") or {}).keys())
                extras = [k for k in payload.keys() if k not in allowed]
                if extras:
                    return False, f"unknown_fields:{extras}"
            except Exception:
                pass
        return True, "ok"

    def adjust_inventory(
        self,
        *,
        payload: Dict[str, Any],
        confirmed: bool,
        requested_by: str = "atlas_comms_hub",
    ) -> Dict[str, Any]:
        if not confirmed:
            return {"ok": False, "error": "transaction_not_confirmed"}
        valid, reason = self._validate_inventory_payload(payload)
        if not valid:
            return {"ok": False, "error": f"payload_validation_failed:{reason}"}
        snapshot = self._run_snapshot_validation()
        if not snapshot.get("stable"):
            return {"ok": False, "error": "snapshot_validation_failed", "snapshot": snapshot}
        token = (os.getenv("ATLAS_PANADERIA_AUTH_BEARER") or "").strip()
        headers = {"Authorization": f"Bearer {token}"} if token else None
        ok, code, data, err = _http_json(
            "POST",
            f"{PANADERIA_BASE}/api/inventory/adjustment",
            payload=payload,
            headers=headers,
            timeout=10.0,
        )
        self._append_snapshot(
            "ATLAS_COMMS_INVENTORY_ADJUST",
            {
                "ok": ok,
                "http_code": code,
                "requested_by": requested_by,
                "product_id": payload.get("product_id"),
                "type": payload.get("type"),
            },
        )
        return {
            "ok": ok,
            "http_code": code,
            "error": err if not ok else None,
            "response": data,
            "snapshot": snapshot,
        }


_hub_singleton: Optional[AtlasCommsHub] = None
_hub_lock = threading.Lock()


def get_atlas_comms_hub() -> AtlasCommsHub:
    global _hub_singleton
    if _hub_singleton is None:
        with _hub_lock:
            if _hub_singleton is None:
                _hub_singleton = AtlasCommsHub()
    return _hub_singleton


def _print_json(data: Dict[str, Any]) -> None:
    print(json.dumps(data, ensure_ascii=False, indent=2))


def _main(argv: Optional[Iterable[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="ATLAS Comms Hub utility")
    parser.add_argument("--init-db", action="store_true", help="Initialize DB/schema and exit")
    parser.add_argument("--status", action="store_true", help="Print hub status")
    parser.add_argument("--resync", action="store_true", help="Process pending offline queue")
    parser.add_argument("--history", action="store_true", help="Print recent history")
    parser.add_argument("--limit", type=int, default=50, help="Limit for history/resync")
    args = parser.parse_args(list(argv) if argv is not None else None)

    hub = get_atlas_comms_hub()

    if args.init_db:
        hub._init_db()
        _print_json({"ok": True, "db_path": str(DB_FILE), "schema_file": str(SCHEMA_FILE)})
        return 0
    if args.resync:
        out = hub.resync_pending(limit=max(1, int(args.limit or 100)))
        _print_json(out)
        return 0 if out.get("ok") else 2
    if args.history:
        out = hub.get_history(limit=max(1, int(args.limit or 30)), decrypt=True)
        _print_json(out)
        return 0

    _print_json(hub.get_status())
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
