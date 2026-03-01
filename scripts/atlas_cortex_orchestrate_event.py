from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.atlas_cortex_orchestrator import process_vision_event


def sample_event() -> Dict[str, Any]:
    return {
        "event_id": "vision-sample-001",
        "source": "rauli-vision",
        "timestamp_iso": datetime.now(timezone.utc).isoformat(),
        "camera_id": "cam-pan-01",
        "store_id": "panaderia-central",
        "anomaly": {"detected": False, "severity": "low", "reason": ""},
        "detections": [
            {
                "product_id": "prod-pan-001",
                "sku": "PAN001",
                "count": 3,
                "quality_score": 0.92,
                "confidence": 0.94,
                "action_hint": "entrada",
            },
            {
                "product_id": "prod-pan-010",
                "sku": "PAN010",
                "count": 1,
                "quality_score": 0.88,
                "confidence": 0.91,
                "action_hint": "merma",
            },
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="ATLAS Cortex: procesa evento RAULI-VISION y decide acciones hacia panaderia."
    )
    parser.add_argument("--event-file", help="Ruta a JSON de evento RAULI-VISION")
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Aplica ajustes en API de panaderia (si no, dry-run).",
    )
    parser.add_argument(
        "--sample",
        action="store_true",
        help="Usa un evento de muestra interno.",
    )
    args = parser.parse_args()

    if args.sample:
        payload = sample_event()
    elif args.event_file:
        payload = json.loads(Path(args.event_file).read_text(encoding="utf-8"))
    else:
        raise SystemExit("Debes usar --sample o --event-file <path>")

    result = process_vision_event(payload, apply_changes=bool(args.apply))
    out = {
        "accepted": result.accepted,
        "action": result.action,
        "reason": result.reason,
        "commands": result.commands,
        "dispatch_results": result.dispatch_results,
    }
    print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
