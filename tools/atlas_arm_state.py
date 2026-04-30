from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict


BASE_DIR = Path(__file__).resolve().parent.parent
STATE_DIR = BASE_DIR / "state"
STATE_DIR.mkdir(parents=True, exist_ok=True)
STATE_FILE = STATE_DIR / "atlas_arm_state.json"


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def default_state() -> Dict[str, Any]:
    return {
        "ts": _now_iso(),
        "arms": {
            "panaderia": {"healthy": True, "isolated": False, "reason": ""},
            "vision": {"healthy": True, "isolated": False, "reason": ""},
        },
    }


def load_state() -> Dict[str, Any]:
    if not STATE_FILE.exists():
        st = default_state()
        save_state(st)
        return st
    try:
        return json.loads(STATE_FILE.read_text(encoding="utf-8"))
    except Exception:
        st = default_state()
        save_state(st)
        return st


def save_state(state: Dict[str, Any]) -> None:
    state["ts"] = _now_iso()
    STATE_FILE.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")


def set_arm_state(arm: str, healthy: bool, reason: str = "") -> Dict[str, Any]:
    st = load_state()
    arms = st.setdefault("arms", {})
    node = arms.setdefault(arm, {"healthy": True, "isolated": False, "reason": ""})
    node["healthy"] = bool(healthy)
    node["isolated"] = not bool(healthy)
    node["reason"] = str(reason or "")
    save_state(st)
    return st


def is_arm_isolated(arm: str) -> bool:
    st = load_state()
    node = ((st.get("arms") or {}).get(arm) or {})
    return bool(node.get("isolated"))
