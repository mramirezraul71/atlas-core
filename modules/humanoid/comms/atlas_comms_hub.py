"""ATLAS comms bridge with hybrid AI, offline queue and secure audit logging."""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import re
import sqlite3
import subprocess
import threading
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

DEFAULT_SCHEDULE = "Lun-Dom 07:00-20:00"
DEFAULT_ORDER_GUIDE = "Puedes indicar producto, cantidad y hora deseada para preparar el pedido."

_RE_URGENT = re.compile(
    r"(urgente|emergencia|incendio|robo|asalto|grave|critico|critical|ayuda|auxilio|"
    r"no funciona|caido|caida|fuga|accidente|sangre|peligro)",
    re.IGNORECASE,
)
_RE_ORDER = re.compile(r"(pedido|orden|encargo|comprar|precio|delivery|envio)", re.IGNORECASE)
_RE_HOURS = re.compile(r"(horario|abren|abierto|cierran|hora)", re.IGNORECASE)
_RE_STOCK = re.compile(r"(stock|disponible|inventario|hay|agotado|existencia)", re.IGNORECASE)

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
"""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _truncate(text: str, max_len: int = 220) -> str:
    t = (text or "").strip().replace("\n", " ")
    if len(t) <= max_len:
        return t
    return t[: max(0, max_len - 3)] + "..."


def _json_dump(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, separators=(",", ":"))


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
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        DB_FILE.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._init_db()

    def _encryption_secret(self) -> Tuple[str, str]:
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

    def _probe_panaderia(self) -> Dict[str, Any]:
        status: Dict[str, Any] = {"reachable": False, "health": {}, "inventory": {}}
        ok, code, data, err = _http_json("GET", f"{PANADERIA_BASE}/api/health", timeout=4.0)
        status["health"] = {"ok": ok, "code": code, "data": data, "error": err}
        status["reachable"] = bool(ok)

        s_ok, s_code, s_data, s_err = _http_json(
            "GET", f"{PANADERIA_BASE}/api/sentinel/health", timeout=4.0
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
            timeout=4.0,
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
        ok, code, data, err = _http_json("GET", f"{VISION_BASE}/api/health", timeout=4.0)
        status["health"] = {"ok": ok, "code": code, "data": data, "error": err}
        status["reachable"] = bool(ok)
        c_ok, c_code, c_data, c_err = _http_json("GET", f"{PUSH_BASE}/vision/cameras", timeout=4.0)
        status["cameras"] = {"ok": c_ok, "code": c_code, "data": c_data, "error": c_err}
        return status

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
        checks = ["http://1.1.1.1/cdn-cgi/trace", "https://www.cloudflare.com/cdn-cgi/trace"]
        last_err = "unknown"
        for url in checks:
            ok, code, _data, err = _http_json("GET", url, timeout=4.0)
            if ok and 200 <= code < 400:
                return True, f"ok:{url}"
            last_err = err or str(code)
        return False, f"offline:{last_err}"

    def _is_offline_mode(self) -> Tuple[bool, Dict[str, Any]]:
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

    def _call_clawd_subscription(
        self, message: str, context_summary: Dict[str, Any]
    ) -> Tuple[bool, str, str]:
        api_url = (os.getenv("ATLAS_CLAWD_API_URL") or "").strip()
        if not api_url:
            return False, "", "clawd_api_url_missing"
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
        }
        ok, code, data, err = _http_json(
            "POST", api_url, payload=payload, headers=headers, timeout=12.0
        )
        if not ok:
            return False, "", f"clawd_http_fail:{err or code}"
        text = (
            data.get("reply")
            or data.get("response")
            or data.get("text")
            or ((data.get("data") or {}).get("reply") if isinstance(data.get("data"), dict) else "")
            or ""
        )
        if not text:
            return False, "", "clawd_empty_reply"
        return True, str(text).strip(), "clawd_api"

    def _offline_fallback_reply(self, message: str, context_summary: Dict[str, Any]) -> str:
        msg = (message or "").strip()
        inv_state = context_summary.get("inventory_state") or "unknown"
        cam_count = int(context_summary.get("camera_count") or 0)
        offline = bool(context_summary.get("offline_mode"))
        if _RE_HOURS.search(msg):
            base = f"Horario estimado: {os.getenv('ATLAS_STORE_SCHEDULE') or DEFAULT_SCHEDULE}."
            if offline:
                base += " Estoy en modo offline y confirmare cambios cuando vuelva el tunel."
            return base
        if _RE_STOCK.search(msg):
            return f"Estado de inventario: {inv_state}. Camaras activas detectadas: {cam_count}."
        if _RE_ORDER.search(msg):
            return (
                "Puedo ayudarte con el pedido. "
                + DEFAULT_ORDER_GUIDE
                + f" Inventario: {inv_state}. Camaras activas: {cam_count}."
            )
        return (
            "Recibi tu mensaje y consulte inventario/camaras antes de responder. "
            f"Inventario={inv_state}, camaras_activas={cam_count}."
        )

    def _generate_reply(
        self, message: str, context_summary: Dict[str, Any], offline_mode: bool
    ) -> Dict[str, Any]:
        if not offline_mode:
            ok, text, provider = self._call_clawd_subscription(message, context_summary)
            if ok:
                return {"text": text, "provider": provider, "offline": False}
        return {
            "text": self._offline_fallback_reply(message, context_summary),
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
            message_id = f"atlas-comms-{uuid.uuid4().hex[:12]}"
            inv = self._probe_panaderia()
            cam = self._probe_vision()
            offline_mode, offline_diag = self._is_offline_mode()
            urgency = self._detect_urgency(msg, context=context)
            context_summary = self._build_context_summary(inv, cam, offline_mode=offline_mode)
            reply = self._generate_reply(msg, context_summary=context_summary, offline_mode=offline_mode)

            inventory_state = str(context_summary.get("inventory_state") or "unknown")
            camera_state = "ok" if bool(context_summary.get("camera_count")) else "unknown"
            metadata = {
                "context": context or {},
                "provider": reply.get("provider"),
                "offline_diag": offline_diag,
            }
            synced = not offline_mode
            self._store_message(
                message_id=message_id,
                user_id=(user_id or "guest").strip() or "guest",
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
                       offline_mode, synced
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
                }
            )
        return {"ok": True, "items": items, "count": len(items)}

    def get_status(self) -> Dict[str, Any]:
        conn = self._db_conn()
        try:
            pending = int(
                conn.execute(
                    "SELECT COUNT(*) FROM comms_offline_queue WHERE status='pending'"
                ).fetchone()[0]
            )
            total = int(conn.execute("SELECT COUNT(*) FROM comms_messages").fetchone()[0])
            row = conn.execute(
                "SELECT created_at FROM comms_messages ORDER BY created_at DESC LIMIT 1"
            ).fetchone()
        finally:
            conn.close()
        offline_mode, offline_diag = self._is_offline_mode()
        _secret, source = self._encryption_secret()
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

    def _mark_queue_error(self, queue_id: int, attempts: int, error: str) -> None:
        conn = self._db_conn()
        try:
            conn.execute(
                """
                UPDATE comms_offline_queue
                SET attempts=?, last_error=?
                WHERE id=?
                """,
                (int(attempts), _truncate(error, 400), int(queue_id)),
            )
            conn.execute(
                """
                UPDATE comms_messages
                SET sync_attempts=sync_attempts+1, last_error=?
                WHERE message_id=(SELECT message_id FROM comms_offline_queue WHERE id=?)
                """,
                (_truncate(error, 400), int(queue_id)),
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
                self._mark_queue_error(qid, attempts + 1, f"decrypt_fail:{e}")
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
