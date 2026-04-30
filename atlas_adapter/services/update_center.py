from __future__ import annotations

import json
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


class UpdateCenterService:
    """Estado persistente y políticas de actualización para Tools/Software."""

    DEFAULT_CONFIG = {
        "enabled": True,
        "dry_run": True,
        "scan_interval_sec": 1800,
        "allowed_surfaces": ["tools", "software"],
        "allowed_policy_classes": ["auto_update_safe"],
        "max_updates_per_cycle": 3,
        "window_start": "00:00",
        "window_end": "23:59",
    }

    MANUAL_REVIEW_IDS = {
        "fastapi",
        "pydantic",
        "sqlalchemy",
        "uvicorn",
        "python",
        "node",
        "git",
        "duckdb",
        "pandas",
        "xgboost",
    }

    NEVER_AUTO_IDS = {
        "ccxt",
        "tradier",
        "tradier_client",
        "live_order_router",
        "order_execution_core",
        "portfolio_risk_engine",
    }

    NEVER_AUTO_KEYWORDS = (
        "trading-runtime",
        "trading_core",
        "trading-core",
        "tradier",
        "live_order",
        "order_execution",
        "broker_execution",
        "portfolio_risk",
        "kill_switch",
        "live_trading",
    )

    MANUAL_KEYWORDS = (
        "runtime",
        "framework",
        "dependency",
        "python",
        "pip",
        "node",
        "ollama",
        "numpy",
        "pandas",
        "scipy",
        "xgboost",
        "lightgbm",
        "sklearn",
        "statsmodels",
        "yfinance",
        "gymnasium",
        "backtrader",
        "stable-baselines",
    )

    def __init__(self, base_dir: Path) -> None:
        self.base_dir = Path(base_dir).resolve()
        self.root = (self.base_dir / "logs" / "update_center").resolve()
        self.jobs_dir = (self.root / "jobs").resolve()
        self.state_file = (self.root / "state.json").resolve()
        self.events_file = (self.root / "events.jsonl").resolve()
        self.config_file = (self.base_dir / "config" / "update_center_config.json").resolve()
        self._lock = threading.RLock()
        self.root.mkdir(parents=True, exist_ok=True)
        self.jobs_dir.mkdir(parents=True, exist_ok=True)
        self._ensure_state()

    def _read_json(self, path: Path, default: Any) -> Any:
        try:
            if not path.exists():
                return default
            raw = json.loads(path.read_text(encoding="utf-8-sig", errors="replace"))
            return raw if raw is not None else default
        except Exception:
            return default

    def _write_json(self, path: Path, payload: Any) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    def _append_event(self, payload: Dict[str, Any]) -> None:
        self.events_file.parent.mkdir(parents=True, exist_ok=True)
        with self.events_file.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(payload, ensure_ascii=False) + "\n")

    def _ensure_state(self) -> None:
        with self._lock:
            if self.state_file.exists():
                return
            payload = {
                "updated_at": _utc_now_iso(),
                "auto_cycle": {"running": False, "last_started_at": None, "last_finished_at": None},
                "surfaces": {"tools": {}, "software": {}},
                "jobs": {},
                "last_errors": {},
            }
            self._write_json(self.state_file, payload)

    def load_config(self) -> Dict[str, Any]:
        with self._lock:
            cfg = self._read_json(self.config_file, {})
            out = dict(self.DEFAULT_CONFIG)
            if isinstance(cfg, dict):
                out.update(cfg)
            return out

    def save_config(self, patch: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            cfg = self.load_config()
            cfg.update({k: v for k, v in (patch or {}).items() if v is not None})
            self._write_json(self.config_file, cfg)
            self.record_event("update.center.config.updated", "system", {"config": cfg})
            return cfg

    def record_event(self, event_type: str, surface: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        with self._lock:
            event = {
                "ts": _utc_now_iso(),
                "event_type": event_type,
                "surface": surface,
                "payload": payload or {},
            }
            self._append_event(event)
            state = self._read_json(self.state_file, {})
            if not isinstance(state, dict):
                state = {}
            state["updated_at"] = event["ts"]
            self._write_json(self.state_file, state)
            return event

    def recent_events(self, limit: int = 80) -> List[Dict[str, Any]]:
        if not self.events_file.exists():
            return []
        try:
            lines = self.events_file.read_text(encoding="utf-8-sig", errors="replace").splitlines()
        except Exception:
            return []
        out: List[Dict[str, Any]] = []
        for line in lines[-max(1, min(500, int(limit))):]:
            try:
                parsed = json.loads(line)
                if isinstance(parsed, dict):
                    out.append(parsed)
            except Exception:
                continue
        return out

    def classify_item(self, item: Dict[str, Any], surface: str) -> Dict[str, Any]:
        item_id = str(item.get("id") or "").strip().lower()
        name = str(item.get("name") or "").strip().lower()
        category = str(item.get("category") or "").strip().lower()
        haystack = " ".join([item_id, name, category])
        critical = bool(item.get("critical"))
        policy_class = "auto_update_safe"
        policy_label = "AUTO"
        reason = "Componente apto para auto-update guardado."
        tags: List[str] = []

        if item_id in self.NEVER_AUTO_IDS or any(k in haystack for k in self.NEVER_AUTO_KEYWORDS):
            policy_class = "never_auto_update"
            policy_label = "BLOQUEADO"
            reason = "Componente runtime/live-sensitive; auto-update bloqueado."
            tags.extend(["runtime_sensitive", "live_sensitive"])
        elif "driver" in category:
            policy_class = "security_critical"
            policy_label = "CRITICO"
            reason = "Driver o capa de sistema crítica; requiere revisión manual."
            tags.append("security_critical")
        elif critical or item_id in self.MANUAL_REVIEW_IDS or "framework" in category or any(k in haystack for k in self.MANUAL_KEYWORDS):
            policy_class = "manual_review_required"
            policy_label = "MANUAL"
            reason = "Framework/base crítica; actualización solo con validación manual."
            tags.append("runtime_sensitive")

        auto_allowed = policy_class == "auto_update_safe"
        out = dict(item)
        out.update(
            {
                "policy_class": policy_class,
                "policy_status_label": policy_label,
                "policy_reason": reason,
                "auto_update_allowed": auto_allowed,
                "policy_tags": sorted(set(tags)),
                "surface": surface,
            }
        )
        return out

    def annotate_rows(self, rows: List[Dict[str, Any]], surface: str) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        for row in rows or []:
            if not isinstance(row, dict):
                continue
            out.append(self.classify_item(row, surface))
        return out

    def policy_summary(self, rows: List[Dict[str, Any]]) -> Dict[str, int]:
        counts = {"auto_update_safe": 0, "manual_review_required": 0, "never_auto_update": 0, "security_critical": 0}
        for row in rows or []:
            if not isinstance(row, dict):
                continue
            key = str(row.get("policy_class") or "")
            if key in counts:
                counts[key] += 1
        return counts

    def refresh_surface_state(self, surface: str, payload: Dict[str, Any], source: str = "scan") -> Dict[str, Any]:
        surface = str(surface or "").strip().lower()
        with self._lock:
            state = self._read_json(self.state_file, {})
            if not isinstance(state, dict):
                state = {}
            surfaces = state.setdefault("surfaces", {})
            if not isinstance(surfaces, dict):
                surfaces = {}
                state["surfaces"] = surfaces
            rows_key = "tools" if surface == "tools" else "software"
            rows = payload.get(rows_key) if isinstance(payload, dict) else []
            rows = rows if isinstance(rows, list) else []
            annotated = self.annotate_rows(rows, surface)
            payload = dict(payload or {})
            payload[rows_key] = annotated
            payload["policy_summary"] = self.policy_summary(annotated)
            if surface == "software":
                nc = payload.get("network_candidates")
                if isinstance(nc, list):
                    payload["network_candidates"] = self.annotate_rows(nc, surface)
            surfaces[surface] = {
                "updated_at": _utc_now_iso(),
                "source": source,
                "generated_at": payload.get("generated_at"),
                "summary": payload.get("summary", {}),
                "policy_summary": payload.get("policy_summary", {}),
            }
            state["updated_at"] = _utc_now_iso()
            self._write_json(self.state_file, state)
            return payload

    def register_job(self, job_id: str, surface: str, action: str, origin: str, extra: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        with self._lock:
            state = self._read_json(self.state_file, {})
            if not isinstance(state, dict):
                state = {}
            jobs = state.setdefault("jobs", {})
            if not isinstance(jobs, dict):
                jobs = {}
                state["jobs"] = jobs
            payload = {
                "job_id": job_id,
                "surface": surface,
                "action": action,
                "origin": origin,
                "status": "queued",
                "created_at": _utc_now_iso(),
                "updated_at": _utc_now_iso(),
            }
            if extra:
                payload.update(extra)
            jobs[str(job_id)] = payload
            self._write_json(self.state_file, state)
            self._write_json(self.jobs_dir / f"{job_id}.json", payload)
            return payload

    def sync_job(self, job_id: str, status_payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            state = self._read_json(self.state_file, {})
            jobs = state.setdefault("jobs", {})
            current = jobs.get(job_id, {"job_id": job_id})
            current.update(status_payload or {})
            current["updated_at"] = _utc_now_iso()
            jobs[job_id] = current
            self._write_json(self.state_file, state)
            self._write_json(self.jobs_dir / f"{job_id}.json", current)
            return current

    def set_auto_cycle(self, running: bool, reason: str = "") -> None:
        with self._lock:
            state = self._read_json(self.state_file, {})
            cycle = state.setdefault("auto_cycle", {})
            now = _utc_now_iso()
            cycle["running"] = bool(running)
            if running:
                cycle["last_started_at"] = now
            else:
                cycle["last_finished_at"] = now
            if reason:
                cycle["last_reason"] = reason
            state["updated_at"] = now
            self._write_json(self.state_file, state)

    def should_run_cycle(self) -> bool:
        cfg = self.load_config()
        if not bool(cfg.get("enabled", False)):
            return False
        state = self._read_json(self.state_file, {})
        cycle = state.get("auto_cycle", {}) if isinstance(state, dict) else {}
        if bool(cycle.get("running")):
            return False
        interval_sec = max(60, int(cfg.get("scan_interval_sec", 1800) or 1800))
        last_finish = str(cycle.get("last_finished_at") or "")
        if not last_finish:
            return True
        try:
            then = datetime.fromisoformat(last_finish.replace("Z", "+00:00"))
            elapsed = (datetime.now(timezone.utc) - then).total_seconds()
            return elapsed >= interval_sec
        except Exception:
            return True

    def get_state(self, events_limit: int = 50) -> Dict[str, Any]:
        with self._lock:
            state = self._read_json(self.state_file, {})
            if not isinstance(state, dict):
                state = {}
            state["config"] = self.load_config()
            state["recent_events"] = self.recent_events(limit=events_limit)
            return state

