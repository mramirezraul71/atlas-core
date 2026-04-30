"""Session memory and run artifacts for atlas_agent."""
from __future__ import annotations

import json
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List

try:
    from .config import AgentConfig
except Exception:  # pragma: no cover - script-mode fallback
    from config import AgentConfig


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass
class StepEvent:
    step: int
    kind: str
    payload: Dict[str, Any]
    ts: str = field(default_factory=_utc_now)


class SessionStore:
    """Persist step-by-step events and session summary."""

    def __init__(self, config: AgentConfig):
        config.runs_dir.mkdir(parents=True, exist_ok=True)
        session_id = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8]
        self.session_id = session_id
        self.session_dir = (config.runs_dir / session_id).resolve()
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.events_file = self.session_dir / "events.jsonl"
        self.summary_file = self.session_dir / "summary.json"
        self._events: List[StepEvent] = []

    def log(self, step: int, kind: str, payload: Dict[str, Any]) -> None:
        event = StepEvent(step=step, kind=kind, payload=payload)
        self._events.append(event)
        with self.events_file.open("a", encoding="utf-8") as f:
            f.write(
                json.dumps(
                    {
                        "step": event.step,
                        "kind": event.kind,
                        "ts": event.ts,
                        "payload": event.payload,
                    },
                    ensure_ascii=False,
                )
                + "\n"
            )

    def finalize(self, result: Dict[str, Any]) -> None:
        summary = {
            "session_id": self.session_id,
            "created_at": _utc_now(),
            "events_count": len(self._events),
            "result": result,
        }
        self.summary_file.write_text(
            json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
        )


def list_recent_summaries(runs_dir: Path, limit: int = 20) -> List[Dict[str, Any]]:
    base = runs_dir.resolve()
    if not base.exists():
        return []
    summaries: List[Dict[str, Any]] = []
    for p in sorted(base.glob("*/summary.json"), key=lambda x: x.stat().st_mtime, reverse=True):
        if len(summaries) >= max(1, limit):
            break
        try:
            summaries.append(json.loads(p.read_text(encoding="utf-8", errors="replace")))
        except Exception:
            continue
    return summaries


def get_session_summary(runs_dir: Path, session_id: str) -> Dict[str, Any] | None:
    p = runs_dir.resolve() / session_id / "summary.json"
    if not p.exists():
        return None
    try:
        return json.loads(p.read_text(encoding="utf-8", errors="replace"))
    except Exception:
        return None
