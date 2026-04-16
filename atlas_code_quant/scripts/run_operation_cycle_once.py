from __future__ import annotations

import json
import os
import urllib.error
import urllib.request


def main() -> int:
    base = os.getenv("ATLAS_QUANT_API_URL", "http://127.0.0.1:8795").rstrip("/")
    api_key = os.getenv("ATLAS_QUANT_API_KEY", "atlas-quant-local")
    payload = {
        "order": {
            "symbol": "MSTX",
            "strategy_type": "equity_long",
            "account_scope": "paper",
            "side": "buy",
            "size": 1,
        },
        "action": "submit",
        "capture_context": False,
    }
    body = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        url=f"{base}/api/v2/quant/operation/test-cycle",
        data=body,
        method="POST",
        headers={
            "Content-Type": "application/json",
            "Accept": "application/json",
            "x-api-key": api_key,
        },
    )
    timeout_sec = int(os.getenv("ATLAS_QUANT_CYCLE_TIMEOUT_SEC", "90"))
    try:
        with urllib.request.urlopen(req, timeout=timeout_sec) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
            print(raw)
            return 0
    except urllib.error.HTTPError as exc:
        print(exc.read().decode("utf-8", errors="replace"))
        return 1
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

