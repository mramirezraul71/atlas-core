#!/usr/bin/env python3
"""
Calibración híbrida autónoma del Radar Kalshi.

Combina:
  - Reglas deterministas (p. ej. cotizaciones stale en /api/radar/markets).
  - AutoTune del servidor (propose/apply) si el modo no es manual.
  - Opcional: planner LLM vía POST /brain/process que devuelve solo JSON de parches.

Requiere PUSH (:8791) en marcha. Uso típico (Windows):

  python scripts/radar_hybrid_calibrate.py --base-url http://127.0.0.1:8791 --dry-run
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from typing import Any, Optional
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

CAP_QUOTE_AGE_MS = 7 * 24 * 60 * 60 * 1000


def _json_req(
    method: str,
    url: str,
    body: Optional[dict[str, Any]] = None,
    timeout: float = 60.0,
) -> dict[str, Any]:
    data = None
    headers = {"Accept": "application/json"}
    if body is not None:
        raw = json.dumps(body).encode("utf-8")
        data = raw
        headers["Content-Type"] = "application/json"
    req = Request(url, data=data, headers=headers, method=method)
    try:
        with urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except HTTPError as e:
        try:
            payload = e.read().decode("utf-8")
            return json.loads(payload)
        except Exception:
            return {"ok": False, "error": str(e), "status": e.code}
    except URLError as e:
        return {"ok": False, "error": str(e.reason if hasattr(e, "reason") else e)}


def _extract_json_object(text: str) -> Optional[dict[str, Any]]:
    t = (text or "").strip()
    if t.startswith("```"):
        t = re.sub(r"^```[a-zA-Z]*\s*", "", t)
        t = re.sub(r"\s*```$", "", t)
    i, j = t.find("{"), t.rfind("}")
    if i < 0 or j <= i:
        return None
    try:
        return json.loads(t[i : j + 1])
    except json.JSONDecodeError:
        return None


def _clamp_patch(raw: dict[str, Any]) -> dict[str, Any]:
    """Filtra y acota claves compatibles con RadarRuntimeConfigBody."""
    out: dict[str, Any] = {}
    if "edge_threshold" in raw and raw["edge_threshold"] is not None:
        v = float(raw["edge_threshold"])
        out["edge_threshold"] = max(0.0005, min(0.95, v))
    if "edge_net_min" in raw and raw["edge_net_min"] is not None:
        v = float(raw["edge_net_min"])
        out["edge_net_min"] = max(0.0, min(0.5, v))
    if "confidence_min" in raw and raw["confidence_min"] is not None:
        v = float(raw["confidence_min"])
        out["confidence_min"] = max(0.0, min(1.0, v))
    if "kelly_fraction" in raw and raw["kelly_fraction"] is not None:
        v = float(raw["kelly_fraction"])
        out["kelly_fraction"] = max(0.01, min(1.0, v))
    if "spread_max_ticks" in raw and raw["spread_max_ticks"] is not None:
        out["spread_max_ticks"] = max(1, min(500, int(raw["spread_max_ticks"])))
    if "min_depth_yes" in raw and raw["min_depth_yes"] is not None:
        out["min_depth_yes"] = max(1, min(1_000_000, int(raw["min_depth_yes"])))
    if "min_depth_no" in raw and raw["min_depth_no"] is not None:
        out["min_depth_no"] = max(1, min(1_000_000, int(raw["min_depth_no"])))
    if "max_open_positions" in raw and raw["max_open_positions"] is not None:
        out["max_open_positions"] = max(1, min(10_000, int(raw["max_open_positions"])))
    if "max_orders_per_minute" in raw and raw["max_orders_per_minute"] is not None:
        out["max_orders_per_minute"] = max(1, min(10_000, int(raw["max_orders_per_minute"])))
    if "max_total_exposure_pct" in raw and raw["max_total_exposure_pct"] is not None:
        v = float(raw["max_total_exposure_pct"])
        out["max_total_exposure_pct"] = max(0.01, min(1.0, v))
    if "max_quote_age_ms" in raw and raw["max_quote_age_ms"] is not None:
        v = int(raw["max_quote_age_ms"])
        out["max_quote_age_ms"] = max(0, min(CAP_QUOTE_AGE_MS, v))
    if "max_latency_ms" in raw and raw["max_latency_ms"] is not None:
        v = int(raw["max_latency_ms"])
        out["max_latency_ms"] = max(0, min(CAP_QUOTE_AGE_MS, v))
    if "cooldown_seconds" in raw and raw["cooldown_seconds"] is not None:
        v = int(raw["cooldown_seconds"])
        out["cooldown_seconds"] = max(0, min(86400 * 7, v))
    return out


def _stale_gate_ratio(rows: list[dict[str, Any]]) -> float:
    if not rows:
        return 0.0
    stale = 0
    for r in rows:
        reason = str(r.get("gate_reason") or "").lower()
        if "quote_age" in reason or "stale" in reason or "quote age" in reason:
            stale += 1
    return stale / float(len(rows))


def _rule_based_patch(
    cfg: dict[str, Any],
    markets_rows: list[dict[str, Any]],
    exec_metrics: dict[str, Any],
) -> dict[str, Any]:
    patch: dict[str, Any] = {}
    ratio = _stale_gate_ratio(markets_rows)
    if ratio >= 0.25:
        cur = int(cfg.get("max_quote_age_ms") or 60_000)
        bumped = min(CAP_QUOTE_AGE_MS, max(cur * 2, 3_600_000))
        if bumped > cur:
            patch["max_quote_age_ms"] = bumped
    fill_ratio = float(exec_metrics.get("fill_ratio") or 0.0)
    if fill_ratio and fill_ratio < 0.15:
        enm = float(cfg.get("edge_net_min") or 0.03)
        patch.setdefault("edge_net_min", max(0.0, enm - 0.002))
    return patch


def _llm_patch(base: str, metrics_summary: dict[str, Any], timeout: float) -> dict[str, Any]:
    prompt = (
        "Eres el planner de calibración del Radar Kalshi (solo paper). "
        "Con las métricas agregadas siguientes, propón ajustes opcionales.\n"
        "Responde EXCLUSIVAMENTE un JSON con cualquier subconjunto de claves: "
        "edge_threshold, edge_net_min, confidence_min, kelly_fraction, "
        "spread_max_ticks, min_depth_yes, min_depth_no, max_open_positions, "
        "max_orders_per_minute, max_total_exposure_pct, max_quote_age_ms, "
        "max_latency_ms, cooldown_seconds.\n"
        "Si no hay cambios seguros, devuelve {}.\n"
        f"Métricas: {json.dumps(metrics_summary, ensure_ascii=False)}\n"
        "JSON:"
    )
    r = _json_req("POST", f"{base.rstrip('/')}/brain/process", {"text": prompt}, timeout)
    if not r.get("ok"):
        return {}
    raw = r.get("response") or ""
    obj = _extract_json_object(str(raw))
    if not isinstance(obj, dict):
        return {}
    return _clamp_patch(obj)


def _lifelog(base: str, episode: dict[str, Any], timeout: float) -> None:
    body = {
        "event_type": "radar_calibration_episode",
        "source": "radar_hybrid_calibrate",
        "perception": json.dumps(episode.get("perception") or {}),
        "action": json.dumps(episode.get("action") or {}),
        "outcome": episode.get("outcome") or "",
        "success": bool(episode.get("success")),
        "importance": float(episode.get("importance") or 0.45),
        "tags": ["radar", "calibration", "hybrid"],
    }
    out = _json_req(
        "POST",
        f"{base.rstrip('/')}/api/cognitive-memory/lifelog/log",
        body,
        timeout,
    )
    if not out.get("ok"):
        print(f"[lifelog] skip o fallo: {out}", file=sys.stderr)


def main() -> int:
    ap = argparse.ArgumentParser(description="Calibración híbrida Radar Kalshi")
    ap.add_argument("--base-url", default="http://127.0.0.1:8791", help="Base PUSH")
    ap.add_argument("--dry-run", action="store_true", help="No POST /api/radar/config")
    ap.add_argument("--persist-env", action="store_true", help="persist_env=true al aplicar")
    ap.add_argument("--use-llm-patch", action="store_true", help="Planner /brain/process")
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--autotune-apply", action="store_true", help="propose+apply si modo≠manual")
    ap.add_argument(
        "--autotune-rollback",
        action="store_true",
        help="Solo POST /api/radar/autotune/rollback y salir",
    )
    ap.add_argument("--lifelog", action="store_true", help="Registrar en lifelog cognitivo")
    args = ap.parse_args()
    base = args.base_url.rstrip("/")

    if args.autotune_rollback:
        rb = _json_req("POST", f"{base}/api/radar/autotune/rollback", timeout=args.timeout)
        print(json.dumps(rb, indent=2, default=str))
        return 0 if rb.get("ok") else 3

    cfg = _json_req("GET", f"{base}/api/radar/config", timeout=args.timeout)
    if not cfg.get("ok") and cfg.get("environment") is None:
        print(f"No se pudo leer config: {cfg}", file=sys.stderr)
        return 1

    metrics = _json_req("GET", f"{base}/api/radar/metrics", timeout=args.timeout)
    markets = _json_req("GET", f"{base}/api/radar/markets", timeout=args.timeout)
    rows = list(markets.get("rows") or [])

    exec_obj = dict(metrics.get("execution") or {})
    perf_obj = dict(metrics.get("performance") or {})

    metrics_summary = {
        "fill_ratio": exec_obj.get("fill_ratio"),
        "attempts": exec_obj.get("attempts"),
        "errors": exec_obj.get("errors"),
        "hit_rate": perf_obj.get("hit_rate"),
        "pnl_net_usd": perf_obj.get("pnl_net_usd"),
        "max_quote_age_ms": cfg.get("max_quote_age_ms"),
        "edge_net_min": cfg.get("edge_net_min"),
        "stale_gate_ratio_sample": round(_stale_gate_ratio(rows[:120]), 4),
    }

    llm_part = (
        _llm_patch(base, metrics_summary, args.timeout) if args.use_llm_patch else {}
    )
    rule_part = _rule_based_patch(cfg, rows[:120], exec_obj)
    merged: dict[str, Any] = {**llm_part, **rule_part}

    episode_pre = {
        "perception": metrics_summary,
        "action": {"merged_patch_before_autotune": merged},
    }

    autotune_msgs: list[str] = []
    if args.autotune_apply:
        st = _json_req("GET", f"{base}/api/radar/autotune/status", timeout=args.timeout)
        mode = ((st.get("config") or {}).get("mode") or "manual").lower()
        if mode == "manual":
            autotune_msgs.append("autotune en manual: no apply")
        else:
            pr = _json_req("POST", f"{base}/api/radar/autotune/propose", timeout=args.timeout)
            autotune_msgs.append(f"propose ok={pr.get('ok')}")
            ap_dict = _json_req("POST", f"{base}/api/radar/autotune/apply", timeout=args.timeout)
            autotune_msgs.append(f"apply ok={ap_dict.get('ok')} msg={ap_dict.get('message')}")

    if not merged:
        print(json.dumps({"ok": True, "message": "no_rule_or_llm_changes", "autotune": autotune_msgs}, indent=2))
        if args.lifelog:
            _lifelog(
                base,
                {
                    **episode_pre,
                    "outcome": "noop",
                    "success": True,
                    "importance": 0.35,
                },
                args.timeout,
            )
        return 0

    body = {
        **merged,
        "environment": cfg.get("environment") or "demo",
        "execution_mode": cfg.get("execution_mode") or "paper",
        "paper_balance_usd": float(cfg.get("paper_balance_usd") or 1000.0),
        "persist_env": bool(args.persist_env),
    }

    if args.dry_run:
        print(
            json.dumps(
                {"ok": True, "dry_run": True, "would_post": body, "autotune": autotune_msgs},
                indent=2,
                default=str,
            )
        )
        if args.lifelog:
            _lifelog(
                base,
                {
                    **episode_pre,
                    "action": {**(episode_pre.get("action") or {}), "would_post": body},
                    "outcome": "dry_run",
                    "success": True,
                    "importance": 0.4,
                },
                args.timeout,
            )
        return 0

    applied = _json_req("POST", f"{base}/api/radar/config", body, timeout=args.timeout)
    print(json.dumps({"ok": applied.get("ok", False), "applied": applied, "autotune": autotune_msgs}, indent=2, default=str))

    if args.lifelog:
        _lifelog(
            base,
            {
                **episode_pre,
                "action": {**(episode_pre.get("action") or {}), "posted_config": body},
                "outcome": json.dumps(applied)[:4000],
                "success": bool(applied.get("ok")),
                "importance": 0.55,
            },
            args.timeout,
        )

    return 0 if applied.get("ok") else 2


if __name__ == "__main__":
    raise SystemExit(main())
