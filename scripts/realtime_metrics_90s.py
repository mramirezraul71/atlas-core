"""Métricas realtime 90s — F9 Fase 3.

Hace 9 muestreos cada 10s sobre 3 endpoints:
  /api/radar/opportunities?limit=5
  /api/system/metrics
  /api/radar/dashboard/summary?symbol=SPY

Imprime tabla de status/latencia y persiste reports/_realtime_metrics_90s.json
"""
from __future__ import annotations

import json
import time
import urllib.request
import urllib.error
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parent.parent
BASE = "http://127.0.0.1:8765"
ENDPOINTS = [
    ("radar_opportunities", "/api/radar/opportunities?limit=5"),
    ("system_metrics", "/api/system/metrics"),
    ("dashboard_summary_spy", "/api/radar/dashboard/summary?symbol=SPY"),
]


def _probe(path: str, timeout: float = 5.0) -> dict[str, Any]:
    url = f"{BASE}{path}"
    started = time.time()
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            body = resp.read()
            elapsed = (time.time() - started) * 1000
            try:
                data = json.loads(body)
            except Exception:
                data = {}
            return {
                "status_code": resp.status,
                "latency_ms": round(elapsed, 1),
                "ok": resp.status < 400,
                "data_sample": (
                    {"keys": list(data.keys())[:8]}
                    if isinstance(data, dict)
                    else {"len": len(data) if hasattr(data, "__len__") else None}
                ),
                "raw_size_bytes": len(body),
            }
    except urllib.error.HTTPError as exc:
        elapsed = (time.time() - started) * 1000
        return {
            "status_code": exc.code,
            "latency_ms": round(elapsed, 1),
            "ok": False,
            "error": str(exc)[:200],
        }
    except Exception as exc:
        elapsed = (time.time() - started) * 1000
        return {
            "status_code": -1,
            "latency_ms": round(elapsed, 1),
            "ok": False,
            "error": str(exc)[:200],
        }


def main() -> int:
    samples: list[dict[str, Any]] = []
    started_at = time.time()
    print(f"[{time.strftime('%H:%M:%S')}] start 90s realtime metrics — 9 samples × 10s")
    for i in range(9):
        snapshot_t = time.time()
        sample = {"sample_idx": i, "ts": snapshot_t, "probes": {}}
        for name, path in ENDPOINTS:
            sample["probes"][name] = _probe(path)
        samples.append(sample)
        # Resumen línea
        line_parts = []
        for name, _ in ENDPOINTS:
            p = sample["probes"][name]
            mark = "OK" if p.get("ok") else f"ERR({p.get('status_code')})"
            line_parts.append(f"{name}={mark}/{p.get('latency_ms')}ms")
        print(f"  sample#{i+1} | " + " | ".join(line_parts))
        # Esperar al siguiente tick (10s exactos)
        next_tick = snapshot_t + 10
        sleep = next_tick - time.time()
        if sleep > 0 and i < 8:
            time.sleep(sleep)

    finished_at = time.time()
    duration = finished_at - started_at

    # Métricas por endpoint
    summary = {}
    for name, _ in ENDPOINTS:
        latencies = [s["probes"][name]["latency_ms"] for s in samples]
        oks = [s["probes"][name].get("ok", False) for s in samples]
        summary[name] = {
            "samples": len(samples),
            "ok_count": sum(1 for x in oks if x),
            "fail_count": sum(1 for x in oks if not x),
            "ok_rate_pct": round(100.0 * sum(1 for x in oks if x) / len(samples), 1),
            "latency_min_ms": round(min(latencies), 1),
            "latency_max_ms": round(max(latencies), 1),
            "latency_avg_ms": round(sum(latencies) / len(latencies), 1),
        }

    out = {
        "started_at": started_at,
        "finished_at": finished_at,
        "duration_sec": round(duration, 2),
        "endpoints": [p for _, p in ENDPOINTS],
        "samples": samples,
        "summary_per_endpoint": summary,
    }
    out_path = REPO / "reports" / "_realtime_metrics_90s.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2, default=str)
    print(f"\n[OK] saved at {out_path}")
    print("\n=== Summary ===")
    for name, s in summary.items():
        print(
            f"  {name:30s} | ok {s['ok_count']}/{s['samples']} "
            f"({s['ok_rate_pct']}%) | latency min={s['latency_min_ms']} "
            f"avg={s['latency_avg_ms']} max={s['latency_max_ms']} ms"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
