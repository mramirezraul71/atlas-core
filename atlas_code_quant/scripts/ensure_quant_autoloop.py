from __future__ import annotations

import json
import os
import urllib.request
import urllib.error


def _post(url: str, payload: dict, api_key: str) -> tuple[int, dict]:
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        url=url,
        data=data,
        method="POST",
        headers={
            "Content-Type": "application/json",
            "Accept": "application/json",
            "x-api-key": api_key,
        },
    )
    try:
        with urllib.request.urlopen(req, timeout=20) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
            return int(resp.status), json.loads(raw) if raw else {}
    except urllib.error.HTTPError as exc:
        raw = exc.read().decode("utf-8", errors="replace")
        try:
            return int(exc.code), json.loads(raw) if raw else {}
        except Exception:
            return int(exc.code), {"raw": raw}


def _get(url: str, api_key: str) -> tuple[int, dict]:
    req = urllib.request.Request(
        url=url,
        method="GET",
        headers={"Accept": "application/json", "x-api-key": api_key},
    )
    with urllib.request.urlopen(req, timeout=20) as resp:
        raw = resp.read().decode("utf-8", errors="replace")
        return int(resp.status), json.loads(raw) if raw else {}


def main() -> int:
    base = os.getenv("ATLAS_QUANT_API_URL", "http://127.0.0.1:8795").rstrip("/")
    api_key = os.getenv("ATLAS_QUANT_API_KEY", "atlas-quant-local").strip()

    status_code, status_payload = _get(f"{base}/operation/loop/status", api_key)
    running = bool((status_payload.get("data") or {}).get("running"))
    if not running:
        start_body = {"interval_sec": 60, "max_per_cycle": 1, "selector_session_mode": "balanced"}
        _, start_payload = _post(f"{base}/operation/loop/start", start_body, api_key)
        print(json.dumps({"action": "start", "result": start_payload}, ensure_ascii=False))
    else:
        print(json.dumps({"action": "already_running", "status": status_payload}, ensure_ascii=False))

    _, final_status = _get(f"{base}/operation/loop/status", api_key)
    print(json.dumps({"final_status": final_status}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

