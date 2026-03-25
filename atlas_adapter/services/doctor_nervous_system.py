"""
ATLAS DOCTOR v1.0 — Sistema Nervioso Central del Ecosistema ATLAS
=================================================================
Monitoriza 15 puertos, 5 daemons, Triada IA, Memory Engine y RAULI-VISION.
Detecta anomalías en 5 capas y ejecuta reparaciones escalonadas (TIER 0-4).

Uso standalone:
    python -m atlas_adapter.services.doctor_nervous_system --once
    python -m atlas_adapter.services.doctor_nervous_system --dry-run
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import socket
import sqlite3
import subprocess
import sys
import time
import threading
import urllib.request
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# ── Configuración ──────────────────────────────────────────────────────────────
BASE_DIR = Path(__file__).resolve().parent.parent.parent
LOGS_DIR = BASE_DIR / "logs"
DB_PATH = LOGS_DIR / "atlas_doctor_events.sqlite"
VENV_PYTHON = str(BASE_DIR / "venv" / "Scripts" / "python.exe")
PROXY_SCRIPT = str(BASE_DIR / "_external" / "RAULI-VISION" / "cliente-local" / "simple-server.py")
PROXY_DIR = str(BASE_DIR / "_external" / "RAULI-VISION" / "cliente-local")
ESPEJO_EXE = str(BASE_DIR / "_external" / "RAULI-VISION" / "espejo" / "espejo.exe")
ESPEJO_DIR = str(BASE_DIR / "_external" / "RAULI-VISION" / "espejo")

HEAL_INTERVAL = int(os.getenv("ATLAS_DOCTOR_INTERVAL_SEC", "30"))
MAX_RETRIES = int(os.getenv("ATLAS_DOCTOR_MAX_RETRIES", "3"))
DRY_RUN = os.getenv("ATLAS_DOCTOR_DRY_RUN", "false").lower() in ("1", "true", "yes")

_log = logging.getLogger("atlas.doctor")

# ── Tipos de datos ──────────────────────────────────────────────────────────────
TIER_CRASH    = 0   # 0.1s  — servicio core caído
TIER_CRITICAL = 1   # 1s    — funcionalidad crítica degradada
TIER_DEGRADED = 2   # 5s    — fallback activado
TIER_WARNING  = 3   # 30s   — análisis LLM + alerta
TIER_EVOLVE   = 4   # 300s  — auto-refactor (self_programming)

TIER_LABELS = {0: "CRASH", 1: "CRÍTICO", 2: "DEGRADADO", 3: "WARNING", 4: "EVOLUCIÓN"}
TIER_COLORS = {0: "red", 1: "orange", 2: "yellow", 3: "blue", 4: "purple"}


@dataclass
class Anomaly:
    component: str
    layer: str           # api | hardware | vision | quant | cognitive
    tier: int
    description: str
    context: Dict[str, Any] = field(default_factory=dict)
    ts: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())


@dataclass
class HealAction:
    anomaly: Anomaly
    action_type: str     # restart | fallback | reduce | alert | noop
    action_detail: str
    healed: bool = False
    outcome: str = ""
    duration_ms: int = 0
    ts: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())


# ── Utilidades de red ──────────────────────────────────────────────────────────
def _tcp(host: str, port: int, timeout: float = 1.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False


def _http(url: str, timeout: float = 2.0) -> Tuple[bool, int, str]:
    """Retorna (ok, status_code, body_snippet)."""
    try:
        req = urllib.request.Request(url, headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            body = r.read().decode("utf-8", "replace")[:300]
            return (200 <= int(r.status) < 300), int(r.status), body
    except Exception as e:
        return False, 0, str(e)[:100]


def _spawn(cmd: List[str], cwd: Optional[str] = None) -> Optional[int]:
    """Lanza proceso en background. Retorna PID o None. Respeta DRY_RUN."""
    if DRY_RUN:
        _log.info("[DRY_RUN] spawn: %s", " ".join(cmd))
        return -1
    flags = 0
    if sys.platform == "win32":
        flags = subprocess.CREATE_NEW_PROCESS_GROUP | 0x08000000  # CREATE_NO_WINDOW
    try:
        p = subprocess.Popen(cmd, cwd=cwd, creationflags=flags,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return p.pid
    except Exception as e:
        _log.error("spawn failed %s: %s", cmd[0], e)
        return None


def _wait_port(port: int, timeout_sec: float = 12.0) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if _tcp("127.0.0.1", port, timeout=0.5):
            return True
        time.sleep(0.8)
    return False


# ── Base de datos de eventos ───────────────────────────────────────────────────
def _init_db() -> None:
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    con = sqlite3.connect(str(DB_PATH))
    con.execute("""
        CREATE TABLE IF NOT EXISTS doctor_events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            ts TEXT,
            component TEXT,
            layer TEXT,
            tier INTEGER,
            tier_label TEXT,
            description TEXT,
            action_type TEXT,
            action_detail TEXT,
            healed INTEGER,
            outcome TEXT,
            duration_ms INTEGER,
            context_json TEXT
        )
    """)
    con.commit()
    con.close()


def _record_event(action: HealAction) -> None:
    try:
        con = sqlite3.connect(str(DB_PATH))
        con.execute("""
            INSERT INTO doctor_events
            (ts, component, layer, tier, tier_label, description, action_type,
             action_detail, healed, outcome, duration_ms, context_json)
            VALUES (?,?,?,?,?,?,?,?,?,?,?,?)
        """, (
            action.ts,
            action.anomaly.component,
            action.anomaly.layer,
            action.anomaly.tier,
            TIER_LABELS[action.anomaly.tier],
            action.anomaly.description,
            action.action_type,
            action.action_detail,
            int(action.healed),
            action.outcome,
            action.duration_ms,
            json.dumps(action.anomaly.context),
        ))
        con.commit()
        con.close()
    except Exception as e:
        _log.debug("DB write error: %s", e)


def _recent_events(limit: int = 50) -> List[Dict]:
    try:
        con = sqlite3.connect(str(DB_PATH))
        con.row_factory = sqlite3.Row
        rows = con.execute(
            "SELECT * FROM doctor_events ORDER BY id DESC LIMIT ?", (limit,)
        ).fetchall()
        con.close()
        return [dict(r) for r in rows]
    except Exception:
        return []


def _similar_count(component: str, hours: int = 24) -> int:
    """Cuántas veces ha fallado este componente en las últimas N horas."""
    try:
        con = sqlite3.connect(str(DB_PATH))
        cutoff = datetime.now(timezone.utc).isoformat()[:13]  # YYYY-MM-DDTHH
        rows = con.execute(
            "SELECT COUNT(*) FROM doctor_events WHERE component=? AND ts>=?",
            (component, cutoff[:10])  # últimas 24h aprox
        ).fetchone()
        con.close()
        return int(rows[0]) if rows else 0
    except Exception:
        return 0


# ── DETECTORES POR CAPA ────────────────────────────────────────────────────────

def _detect_api_layer() -> List[Anomaly]:
    anomalies = []

    # ATLAS API :8791 — solo TCP (evitar auto-probe circular dentro del mismo proceso)
    if not _tcp("127.0.0.1", 8791):
        anomalies.append(Anomaly("atlas_api", "api", TIER_CRASH,
                                 "ATLAS API :8791 TCP down — uvicorn caído"))

    # ATLAS-Quant :8795
    if not _tcp("127.0.0.1", 8795):
        anomalies.append(Anomaly("quant_api", "quant", TIER_CRITICAL,
                                 "Quant API :8795 TCP down"))
    else:
        ok, status, body = _http("http://127.0.0.1:8795/health")
        if not ok:
            anomalies.append(Anomaly("quant_api", "quant", TIER_DEGRADED,
                                     f"Quant /health degradado (status={status})"))

    # Quant LiveLoop
    ok_loop, _, body_loop = _http("http://127.0.0.1:8795/operation/loop/status")
    if ok_loop:
        try:
            d = json.loads(body_loop)
            cycle_time = (d.get("data") or {}).get("last_cycle_time_sec", 0) or 0
            if float(cycle_time) > 6.0:
                anomalies.append(Anomaly("quant_loop", "quant", TIER_WARNING,
                                         f"LiveLoop cycle_time {cycle_time:.1f}s > 6s",
                                         {"cycle_time": cycle_time}))
        except Exception:
            pass

    # Memory Engine (ChromaDB health)
    try:
        from autonomous.health_monitor.health_aggregator import HealthAggregator
        agg = HealthAggregator()
        report = agg.get_health_report() if hasattr(agg, 'get_health_report') else {}
        mem_score = (report.get("components") or {}).get("memory", {}).get("score", 100)
        if isinstance(mem_score, (int, float)) and mem_score < 50:
            anomalies.append(Anomaly("memory_engine", "api", TIER_DEGRADED,
                                     f"Memory Engine score bajo: {mem_score}",
                                     {"score": mem_score}))
    except Exception:
        pass

    return anomalies


def _detect_hardware_layer() -> List[Anomaly]:
    anomalies = []

    # NEXUS :8000
    if not _tcp("127.0.0.1", 8000):
        anomalies.append(Anomaly("nexus_api", "hardware", TIER_CRITICAL,
                                 "NEXUS :8000 TCP down — robot RAULI sin API"))
    else:
        ok, status, body = _http("http://127.0.0.1:8000/health")
        if not ok:
            anomalies.append(Anomaly("nexus_api", "hardware", TIER_DEGRADED,
                                     f"NEXUS /health degradado (status={status})"))

    # NEXUS robot backend :8002
    if not _tcp("127.0.0.1", 8002):
        anomalies.append(Anomaly("nexus_robot", "hardware", TIER_WARNING,
                                 "NEXUS robot backend :8002 TCP down"))

    return anomalies


def _detect_vision_layer() -> List[Anomaly]:
    anomalies = []

    # espejo.exe :8080
    if not _tcp("127.0.0.1", 8080):
        anomalies.append(Anomaly("vision_espejo", "vision", TIER_CRASH,
                                 "RAULI-VISION espejo.exe :8080 caído — Cloudflare 502"))
    else:
        ok, status, _ = _http("http://127.0.0.1:8080/api/health")
        if not ok:
            anomalies.append(Anomaly("vision_espejo", "vision", TIER_CRITICAL,
                                     f"espejo.exe HTTP degradado (status={status})"))

    # Python proxy :3000
    if not _tcp("127.0.0.1", 3000):
        anomalies.append(Anomaly("vision_proxy", "vision", TIER_CRASH,
                                 "RAULI-VISION proxy :3000 caído — móvil sin acceso"))

    # Dashboard :5174
    if not _tcp("127.0.0.1", 5174):
        anomalies.append(Anomaly("vision_dashboard", "vision", TIER_WARNING,
                                 "RAULI-VISION dashboard :5174 caído"))

    return anomalies


def _detect_quant_layer() -> List[Anomaly]:
    anomalies = []

    # ProductionGuard — gates_rejected rate
    ok, _, body = _http("http://127.0.0.1:8795/production/status")
    if ok:
        try:
            d = json.loads(body)
            rejected = (d.get("data") or {}).get("gates_rejected_last_hour", 0) or 0
            if int(rejected) > 10:
                anomalies.append(Anomaly("quant_guard", "quant", TIER_WARNING,
                                         f"ProductionGuard: {rejected} gates rechazados/hora",
                                         {"rejected": rejected}))
        except Exception:
            pass

    # Paper account health
    ok, _, body = _http("http://127.0.0.1:8795/paper/account")
    if ok:
        try:
            d = json.loads(body)
            equity = float((d.get("data") or {}).get("equity", 0) or 0)
            if equity < 500:  # menos de $500 en paper — posible reseteo
                anomalies.append(Anomaly("quant_paper", "quant", TIER_WARNING,
                                         f"Paper account equity bajo: ${equity:.2f}",
                                         {"equity": equity}))
        except Exception:
            pass

    return anomalies


def _detect_cognitive_layer() -> List[Anomaly]:
    anomalies = []

    # Triada IA vía /api/battery/status — timeout corto para no bloquear
    ok, _, body = _http("http://127.0.0.1:8791/api/battery/status", timeout=1.5)
    if ok:
        try:
            d = json.loads(body)
            data_val = d.get("data") or {}
            battery = data_val.get("battery", data_val) if isinstance(data_val, dict) else {}
            providers = battery if isinstance(battery, dict) else {}
            down = [k for k, v in providers.items()
                    if isinstance(v, dict) and not (v.get("ok") or v.get("status") == "ok")]
            if len(down) >= 2:
                anomalies.append(Anomaly("triada_ia", "cognitive", TIER_CRITICAL,
                                         f"Triada IA: {len(down)} proveedores caídos: {down}",
                                         {"down": down}))
            elif len(down) == 1:
                anomalies.append(Anomaly("triada_ia", "cognitive", TIER_WARNING,
                                         f"Triada IA: proveedor degradado: {down}",
                                         {"down": down}))
        except Exception:
            pass

    # Ops bus lifelog
    try:
        log_path = LOGS_DIR / "ops_bus.log"
        if log_path.exists():
            size = log_path.stat().st_size
            if size == 0:
                anomalies.append(Anomaly("ops_bus", "cognitive", TIER_WARNING,
                                         "ops_bus.log vacío — logging detenido"))
    except Exception:
        pass

    return anomalies


# ── AUTO-HEALING BRAIN ─────────────────────────────────────────────────────────
_fail_counts: Dict[str, int] = {}


def _heal(anomaly: Anomaly) -> HealAction:
    t0 = time.monotonic()
    comp = anomaly.component
    _fail_counts[comp] = _fail_counts.get(comp, 0) + 1
    count = _fail_counts[comp]

    action = HealAction(anomaly=anomaly, action_type="noop", action_detail="sin acción")

    # ── TIER 0/1: CRASH o CRÍTICO ──────────────────────────────────────────────
    if comp == "atlas_api" and anomaly.tier <= TIER_CRITICAL:
        if count <= MAX_RETRIES:
            pid = _spawn(
                [VENV_PYTHON, "-m", "uvicorn", "atlas_adapter.atlas_http_api:app",
                 "--host", "0.0.0.0", "--port", "8791"],
                cwd=str(BASE_DIR)
            )
            ok = pid and _wait_port(8791, 20)
            action.action_type = "restart"
            action.action_detail = f"uvicorn restart PID={pid}"
            action.healed = bool(ok)
            action.outcome = "UP :8791" if ok else "no responde tras restart"
        else:
            action.action_type = "alert"
            action.action_detail = f"MAX_RETRIES alcanzado ({count}x) — escalado a ops_bus"
            _escalate(anomaly)

    elif comp == "vision_espejo":
        if count <= MAX_RETRIES:
            pid = _spawn(
                [ESPEJO_EXE],
                cwd=ESPEJO_DIR
            )
            ok = pid and _wait_port(8080, 12)
            action.action_type = "restart"
            action.action_detail = f"espejo.exe restart PID={pid}"
            action.healed = bool(ok)
            action.outcome = "espejo UP :8080" if ok else "no responde"
        else:
            action.action_type = "alert"
            action.action_detail = "espejo.exe falla repetidamente — alerta"
            _escalate(anomaly)

    elif comp == "vision_proxy":
        if count <= MAX_RETRIES:
            python = VENV_PYTHON if Path(VENV_PYTHON).exists() else "python"
            pid = _spawn([python, PROXY_SCRIPT], cwd=PROXY_DIR)
            ok = pid and _wait_port(3000, 12)
            action.action_type = "restart"
            action.action_detail = f"simple-server.py restart PID={pid}"
            action.healed = bool(ok)
            action.outcome = "proxy UP :3000" if ok else "no responde"
        else:
            _escalate(anomaly)

    elif comp == "nexus_api":
        action.action_type = "alert"
        action.action_detail = "NEXUS down — requiere inicio manual del robot RAULI"
        if count >= 2:
            _escalate(anomaly)

    elif comp == "quant_api":
        action.action_type = "alert"
        action.action_detail = "Quant API down — ejecutar atlas_quant_start.ps1"
        if count >= 2:
            _escalate(anomaly)

    # ── TIER 2: DEGRADADO ──────────────────────────────────────────────────────
    elif anomaly.tier == TIER_DEGRADED:
        action.action_type = "fallback"
        action.action_detail = f"Componente {comp} en modo degradado — monitoreando"

    # ── TIER 3: WARNING ────────────────────────────────────────────────────────
    elif anomaly.tier == TIER_WARNING:
        action.action_type = "alert"
        action.action_detail = f"{comp}: {anomaly.description[:100]}"
        _escalate(anomaly)

    # ── Reset fail counter si se sanó ──────────────────────────────────────────
    if action.healed:
        _fail_counts[comp] = 0

    action.duration_ms = int((time.monotonic() - t0) * 1000)
    return action


def _escalate(anomaly: Anomaly) -> None:
    """Envía alerta por ops_bus (Telegram + log)."""
    try:
        from modules.humanoid.comms.ops_bus import emit as ops_emit
        msg = (
            f"🚨 [ATLAS DOCTOR] TIER{anomaly.tier} {TIER_LABELS[anomaly.tier]}\n"
            f"Componente: {anomaly.component}\n"
            f"Capa: {anomaly.layer}\n"
            f"{anomaly.description}"
        )
        ops_emit("atlas.doctor", msg, level="ERROR", audio=False)
    except Exception as e:
        _log.debug("ops_emit failed: %s", e)
    # También log local
    _log.warning("[DOCTOR] ESCALATE tier=%s comp=%s: %s",
                 TIER_LABELS[anomaly.tier], anomaly.component, anomaly.description)


# ── CLASE PRINCIPAL ─────────────────────────────────────────────────────────────
class AtlasDoctor:
    """Sistema Nervioso Central de ATLAS — detecta y repara anomalías en todas las capas."""

    def __init__(self) -> None:
        self._running = False
        self._task: Optional[asyncio.Task] = None
        self._lock = asyncio.Lock()
        self.last_cycle_ts: Optional[str] = None
        self.last_anomalies: List[Anomaly] = []
        self.last_actions: List[HealAction] = []
        self.cycle_count: int = 0
        _init_db()

    # ── Colección del estado completo ──────────────────────────────────────────
    async def collect_full_state(self) -> Dict[str, List[Anomaly]]:
        """Corre todos los detectores en threads paralelos."""
        detectors = [
            _detect_api_layer,
            _detect_hardware_layer,
            _detect_vision_layer,
            _detect_quant_layer,
            _detect_cognitive_layer,
        ]
        results = await asyncio.gather(
            *[asyncio.to_thread(fn) for fn in detectors],
            return_exceptions=True
        )
        all_anomalies: List[Anomaly] = []
        for r in results:
            if isinstance(r, list):
                all_anomalies.extend(r)
        return all_anomalies

    # ── Loop principal ─────────────────────────────────────────────────────────
    async def nervous_system_loop(self) -> None:
        _log.info("[DOCTOR] Sistema Nervioso iniciado (interval=%ss, dry_run=%s)",
                  HEAL_INTERVAL, DRY_RUN)
        while self._running:
            try:
                async with self._lock:
                    await self._run_cycle()
            except Exception as exc:
                _log.error("[DOCTOR] Error en ciclo: %s", exc)
            await asyncio.sleep(HEAL_INTERVAL)

    async def _run_cycle(self) -> None:
        t0 = time.monotonic()
        self.cycle_count += 1
        self.last_cycle_ts = datetime.now(timezone.utc).isoformat()

        anomalies = await self.collect_full_state()
        self.last_anomalies = anomalies

        actions: List[HealAction] = []
        for anomaly in anomalies:
            tier_label = TIER_LABELS[anomaly.tier]
            _log.warning("[DOCTOR] DETECT TIER%s: %s — %s",
                         anomaly.tier, anomaly.component, anomaly.description)
            action = await asyncio.to_thread(_heal, anomaly)
            actions.append(action)
            _record_event(action)
            status = "HEALED" if action.healed else ("ALERT" if action.action_type == "alert" else "MONITORED")
            _log.info("[DOCTOR] %s: %s → %s", status, anomaly.component, action.outcome or action.action_detail)

        self.last_actions = actions
        elapsed = time.monotonic() - t0

        if not anomalies:
            _log.debug("[DOCTOR] Ciclo %d — sin anomalías (%.1fs)", self.cycle_count, elapsed)
        else:
            healed = sum(1 for a in actions if a.healed)
            _log.info("[DOCTOR] Ciclo %d — %d anomalías, %d sanadas (%.1fs)",
                      self.cycle_count, len(anomalies), healed, elapsed)

    # ── Ciclo único (CLI / tests) ──────────────────────────────────────────────
    async def run_once(self) -> Dict:
        _init_db()
        anomalies = await self.collect_full_state()
        self.last_anomalies = anomalies
        actions: List[HealAction] = []
        for anomaly in anomalies:
            action = await asyncio.to_thread(_heal, anomaly)
            actions.append(action)
            _record_event(action)
        self.last_actions = actions
        return self.status_report()

    # ── Status público ─────────────────────────────────────────────────────────
    def status_report(self) -> Dict:
        tiers = {}
        for a in self.last_anomalies:
            tiers[TIER_LABELS[a.tier]] = tiers.get(TIER_LABELS[a.tier], 0) + 1
        healed = sum(1 for x in self.last_actions if x.healed)
        return {
            "ok": len(self.last_anomalies) == 0,
            "cycle": self.cycle_count,
            "last_ts": self.last_cycle_ts,
            "anomalies_count": len(self.last_anomalies),
            "healed_count": healed,
            "tiers": tiers,
            "anomalies": [
                {"component": a.component, "layer": a.layer,
                 "tier": TIER_LABELS[a.tier], "description": a.description}
                for a in self.last_anomalies
            ],
            "actions": [
                {"component": x.anomaly.component, "type": x.action_type,
                 "detail": x.action_detail, "healed": x.healed, "outcome": x.outcome}
                for x in self.last_actions
            ],
        }

    def history(self, limit: int = 50) -> List[Dict]:
        return _recent_events(limit)

    def similar_count(self, component: str) -> int:
        return _similar_count(component)

    # ── Lifecycle ──────────────────────────────────────────────────────────────
    async def start_nervous_system(self) -> None:
        if self._running:
            return
        self._running = True
        self._task = asyncio.create_task(self.nervous_system_loop(),
                                         name="atlas-doctor-loop")
        _log.info("✓ ATLAS DOCTOR activo")

    async def stop_nervous_system(self) -> None:
        self._running = False
        if self._task and not self._task.done():
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        _log.info("[DOCTOR] Sistema Nervioso detenido")

    def emergency_stop(self) -> Dict:
        """Para todos los subsistemas críticos de forma segura."""
        results = {}
        # Detener Quant LiveLoop
        try:
            ok, status, _ = _http("http://127.0.0.1:8795/operation/loop/stop",
                                   timeout=3.0)
            results["quant_loop"] = "stopped" if ok else f"error status={status}"
        except Exception as e:
            results["quant_loop"] = f"error: {e}"
        _escalate(Anomaly("system", "api", TIER_CRITICAL,
                           "EMERGENCY STOP ejecutado manualmente"))
        results["doctor"] = "emergency_stop_called"
        _log.critical("[DOCTOR] EMERGENCY STOP ejecutado")
        return results


# ── Singleton global ───────────────────────────────────────────────────────────
_DOCTOR: Optional[AtlasDoctor] = None


def get_doctor() -> AtlasDoctor:
    global _DOCTOR
    if _DOCTOR is None:
        _DOCTOR = AtlasDoctor()
    return _DOCTOR


# ── Entrada CLI ────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="ATLAS DOCTOR — diagnóstico standalone")
    parser.add_argument("--once", action="store_true", help="Un solo ciclo y salir")
    parser.add_argument("--dry-run", action="store_true", help="No ejecutar reparaciones")
    args = parser.parse_args()

    if args.dry_run:
        os.environ["ATLAS_DOCTOR_DRY_RUN"] = "true"

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    doctor = AtlasDoctor()

    async def _main():
        result = await doctor.run_once()
        print(json.dumps(result, indent=2, ensure_ascii=False))

    asyncio.run(_main())
