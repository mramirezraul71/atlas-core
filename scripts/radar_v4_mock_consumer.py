from __future__ import annotations

import argparse
import json
import urllib.error
import urllib.request


def get_json(url: str) -> dict:
    req = urllib.request.Request(url, headers={"Accept": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as response:
        return json.loads(response.read().decode("utf-8"))


def consume_summary(base_url: str, symbol: str) -> dict:
    url = f"{base_url.rstrip('/')}/api/radar/v4/summary?symbol={symbol}"
    return get_json(url)


def consume_stream_once(base_url: str) -> list[str]:
    url = f"{base_url.rstrip('/')}/api/radar/stream?once=true"
    req = urllib.request.Request(url, headers={"Accept": "text/event-stream"})
    with urllib.request.urlopen(req, timeout=15) as response:
        text = response.read().decode("utf-8")
    return [line for line in text.splitlines() if line.startswith("event: ")]


def main() -> int:
    parser = argparse.ArgumentParser(description="Mock consumidor V4 del Institutional Radar")
    parser.add_argument("--base-url", default="http://127.0.0.1:8795", help="URL base del adapter")
    parser.add_argument("--symbol", default="SPY", help="Símbolo a consultar")
    parser.add_argument("--skip-stream", action="store_true", help="No validar stream SSE")
    args = parser.parse_args()

    try:
        summary = consume_summary(args.base_url, args.symbol)
    except urllib.error.HTTPError as exc:
        print(f"[ERROR] summary HTTP {exc.code}: {exc.reason}")
        return 1
    except Exception as exc:  # pragma: no cover
        print(f"[ERROR] summary: {exc}")
        return 1

    required_keys = {
        "radar_status",
        "top_signals",
        "gate_recent_decisions",
        "provider_health_summary",
        "degradations_active",
        "structural_context_summary",
        "fast_context_summary",
        "last_update",
        "freshness",
        "stream_available",
    }
    missing = sorted(required_keys - set(summary.keys()))
    if missing:
        print(f"[ERROR] payload v4 incompleto. faltan claves: {missing}")
        return 2

    print("[OK] /api/radar/v4/summary válido")
    print(json.dumps(summary["radar_status"], ensure_ascii=False, indent=2))

    if args.skip_stream:
        return 0

    try:
        events = consume_stream_once(args.base_url)
    except Exception as exc:  # pragma: no cover
        print(f"[WARN] stream no disponible: {exc}")
        return 0

    print(f"[OK] stream respondió. eventos: {events}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
