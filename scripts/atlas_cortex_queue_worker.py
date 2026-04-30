from __future__ import annotations

import argparse
import json
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.atlas_cortex_orchestrator import process_vision_event


BUS_DIR = PROJECT_ROOT / "state" / "atlas_bus"
INBOX = BUS_DIR / "vision_inbox"
PROCESSED = BUS_DIR / "vision_processed"
FAILED = BUS_DIR / "vision_failed"
DECISIONS = BUS_DIR / "atlas_decisions"


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _ensure_dirs() -> None:
    for d in [INBOX, PROCESSED, FAILED, DECISIONS]:
        d.mkdir(parents=True, exist_ok=True)


def _save_decision(event_id: str, payload: Dict[str, Any]) -> None:
    safe = "".join(c for c in event_id if c.isalnum() or c in ("-", "_"))[:80] or "event"
    out = DECISIONS / f"{safe}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def process_file(path: Path, apply_changes: bool) -> Dict[str, Any]:
    raw = path.read_text(encoding="utf-8-sig")
    payload = json.loads(raw)
    result = process_vision_event(payload, apply_changes=apply_changes)
    event_id = str(payload.get("event_id") or path.stem)
    decision = {
        "ts": _now_iso(),
        "event_file": str(path),
        "event_id": event_id,
        "accepted": result.accepted,
        "action": result.action,
        "reason": result.reason,
        "commands": result.commands,
        "dispatch_results": result.dispatch_results,
    }
    _save_decision(event_id, decision)
    return decision


def main() -> int:
    parser = argparse.ArgumentParser(
        description="ATLAS Cortex queue worker: procesa eventos RAULI-VISION en state/atlas_bus/vision_inbox."
    )
    parser.add_argument("--apply", action="store_true", help="Aplica ajustes a panaderia.")
    parser.add_argument(
        "--once",
        action="store_true",
        help="Procesa una pasada y termina.",
    )
    args = parser.parse_args()

    _ensure_dirs()
    files = sorted(INBOX.glob("*.json"))
    if not files:
        print("No hay eventos en inbox.")
        return 0

    processed = 0
    failed = 0
    for f in files:
        try:
            decision = process_file(f, apply_changes=bool(args.apply))
            target = PROCESSED / f.name
            shutil.move(str(f), str(target))
            processed += 1
            print(
                json.dumps(
                    {
                        "event_file": f.name,
                        "action": decision["action"],
                        "accepted": decision["accepted"],
                    },
                    ensure_ascii=False,
                )
            )
        except Exception as e:
            target = FAILED / f.name
            try:
                shutil.move(str(f), str(target))
            except Exception:
                pass
            failed += 1
            print(
                json.dumps(
                    {"event_file": f.name, "error": f"{type(e).__name__}: {e}"},
                    ensure_ascii=False,
                )
            )

    print(json.dumps({"processed": processed, "failed": failed}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
