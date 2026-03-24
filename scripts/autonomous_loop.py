#!/usr/bin/env python
"""autonomous_loop.py — Loop autónomo externo para ATLAS-Quant.

Puente entre scanner y OperationCenter usando endpoints disponibles en el proceso actual.
Compatible con ambos servidores (8792 y 8795).

Flujo por ciclo:
  1. GET /scanner/report → obtener candidatos
  2. GET /api/v2/quant/operation/status → verificar auton_mode + failsafe
  3. POST /api/v2/quant/operation/test-cycle → evaluar + submit si paper_autonomous
  4. Log resultado y esperar al siguiente ciclo

Uso:
    python scripts/autonomous_loop.py
    python scripts/autonomous_loop.py --port 8792 --interval 90
    python scripts/autonomous_loop.py --port 8795 --interval 120 --max-per-cycle 2
"""
from __future__ import annotations

import argparse
import json
import logging
import logging.handlers
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from urllib.request import Request, urlopen
from urllib.error import URLError

# ── Setup ─────────────────────────────────────────────────────────────────────
_ROOT = Path(__file__).resolve().parents[1]
_LOG_FILE = _ROOT / "logs" / "atlas_live_loop.log"
_LOG_FILE.parent.mkdir(parents=True, exist_ok=True)

_fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(name)s — %(message)s", "%H:%M:%S")
_root_logger = logging.getLogger()
_root_logger.setLevel(logging.INFO)
_sh = logging.StreamHandler(sys.stdout); _sh.setFormatter(_fmt)
_fh = logging.handlers.RotatingFileHandler(_LOG_FILE, maxBytes=20*1024*1024, backupCount=5, encoding="utf-8")
_fh.setFormatter(_fmt)
_root_logger.addHandler(_sh)
_root_logger.addHandler(_fh)

logger = logging.getLogger("atlas.execution.live_loop")


# ── HTTP helpers ──────────────────────────────────────────────────────────────

def _api_call(base: str, path: str, method: str = "GET", body: dict | None = None, api_key: str = "atlas-quant-local") -> dict | None:
    url = f"{base}{path}"
    headers = {"x-api-key": api_key, "Content-Type": "application/json"}
    data = json.dumps(body).encode() if body else None
    try:
        req = Request(url, data=data, headers=headers, method=method)
        with urlopen(req, timeout=15) as resp:
            return json.loads(resp.read().decode())
    except URLError as e:
        logger.warning("API call failed %s %s: %s", method, path, e)
        return None
    except Exception as e:
        logger.error("API error %s %s: %s", method, path, e)
        return None


def _detect_base(ports: list[int], api_key: str) -> str | None:
    """Detecta el primer puerto que responde /health con loop endpoints."""
    for port in ports:
        base = f"http://127.0.0.1:{port}"
        r = _api_call(base, "/health", api_key=api_key)
        if r and r.get("status") == "ok":
            # Preferir el que tiene scanner activo
            scan = _api_call(base, "/scanner/report", api_key=api_key)
            if scan and scan.get("ok"):
                logger.info("Servidor detectado en puerto %d (scanner activo)", port)
                return base
    # Fallback: primer que responde
    for port in ports:
        base = f"http://127.0.0.1:{port}"
        r = _api_call(base, "/health", api_key=api_key)
        if r and r.get("status") == "ok":
            logger.info("Servidor detectado en puerto %d (fallback)", port)
            return base
    return None


# ── Loop principal ────────────────────────────────────────────────────────────

def run_loop(base: str, interval_sec: int, max_per_cycle: int, api_key: str) -> None:
    logger.info("=" * 60)
    logger.info("ATLAS autonomous_loop iniciado")
    logger.info("  base=%s interval=%ds max_per_cycle=%d", base, interval_sec, max_per_cycle)
    logger.info("=" * 60)

    cycle = 0
    total_submitted = 0
    total_blocked = 0

    while True:
        cycle += 1
        t_cycle = time.perf_counter()
        ts_now = datetime.now(timezone.utc).strftime("%H:%M:%S")
        logger.info("[cycle %d] -- %s --------------------------", cycle, ts_now)

        # ── 1. Verificar estado operacional ──────────────────────────────────
        op = _api_call(base, "/api/v2/quant/operation/status", api_key=api_key)
        if not op or not op.get("ok"):
            op = _api_call(base, "/operation/status", api_key=api_key)

        if op and op.get("data"):
            d = op["data"]
            auton_mode = d.get("config", {}).get("auton_mode", "unknown")
            auton_active = d.get("auton_mode_active", False)
            failsafe = d.get("failsafe", {}).get("active", False)
            equity = d.get("monitor_summary", {}).get("balances", {}).get("total_equity", 0)

            if failsafe:
                logger.warning("[cycle %d] FAILSAFE activo — saltando ciclo", cycle)
                time.sleep(interval_sec)
                continue

            if not auton_active or auton_mode not in {"paper_autonomous"}:
                logger.warning("[cycle %d] auton_mode=%s active=%s — modo no autónomo, saltando", cycle, auton_mode, auton_active)
                time.sleep(interval_sec)
                continue

            logger.info("[cycle %d] auton_mode=%s equity=%.2f", cycle, auton_mode, equity or 0)
        else:
            logger.warning("[cycle %d] No se pudo obtener estado operacional", cycle)

        # ── 2. Obtener candidatos del scanner ─────────────────────────────────
        scan = _api_call(base, "/scanner/report", api_key=api_key)
        if not scan or not scan.get("ok"):
            scan = _api_call(base, "/api/v2/quant/scanner/report", api_key=api_key)

        if not scan or not scan.get("data"):
            logger.info("[cycle %d] Sin reporte de scanner disponible", cycle)
            time.sleep(interval_sec)
            continue

        candidates = scan["data"].get("candidates", [])
        if not candidates:
            logger.info("[cycle %d] Sin candidatos del scanner", cycle)
            time.sleep(interval_sec)
            continue

        # Ordenar por score descendente, tomar top N
        sorted_cands = sorted(
            candidates,
            key=lambda c: float(c.get("selection_score") or 0),
            reverse=True,
        )[:max_per_cycle]

        logger.info("[cycle %d] %d candidatos -> procesando top %d", cycle, len(candidates), len(sorted_cands))

        # ── 3. Evaluar + enviar cada candidato ────────────────────────────────
        cycle_submitted = 0
        for cand in sorted_cands:
            symbol = str(cand.get("symbol") or "").strip()
            if not symbol:
                continue

            score = float(cand.get("selection_score") or 0)
            direction = str(cand.get("direction") or "long").lower()
            side = "buy" if direction in {"long", "alcista", "bull", "up"} else "sell_short"
            tf = cand.get("timeframe", "?")

            logger.info("[cycle %d] -> %s score=%.1f dir=%s tf=%s", cycle, symbol, score, direction, tf)

            # Construir request para test-cycle
            order_body = {
                "order": {
                    "symbol": symbol,
                    "side": side,
                    "size": 1,
                    "order_type": "market",
                    "account_scope": "paper",
                    "preview": False,
                    "asset_class": "equity",
                },
                "action": "submit",
                "capture_context": False,
            }

            result = _api_call(
                base,
                "/api/v2/quant/operation/test-cycle",
                method="POST",
                body=order_body,
                api_key=api_key,
            )
            if not result:
                result = _api_call(
                    base,
                    "/operation/test-cycle",
                    method="POST",
                    body=order_body,
                    api_key=api_key,
                )

            if result and result.get("ok"):
                data_r = result.get("data", {})
                blocked = data_r.get("blocked", False)
                decision = data_r.get("decision", "?")
                reasons = data_r.get("reasons", [])
                prob = data_r.get("probability", {})
                win_rate = prob.get("win_rate_pct", 0) if prob else 0

                if blocked:
                    total_blocked += 1
                    reason_str = ", ".join(str(r) for r in reasons[:3])
                    logger.info(
                        "  BLOCKED %s | decision=%s | %s",
                        symbol, decision, reason_str
                    )
                else:
                    cycle_submitted += 1
                    total_submitted += 1
                    logger.info(
                        "  SUBMIT  %s | decision=%s | win_rate=%.1f%% | score=%.1f",
                        symbol, decision, win_rate or 0, score
                    )
            else:
                err = result.get("error", "?") if result else "sin respuesta"
                logger.warning("  ERROR %s | %s", symbol, err)

        elapsed_ms = (time.perf_counter() - t_cycle) * 1000
        logger.info(
            "[cycle %d] completado | submitted=%d blocked=%d | %.0fms | total_submitted=%d",
            cycle, cycle_submitted, total_blocked, elapsed_ms, total_submitted
        )

        # ── 4. Esperar al siguiente ciclo ─────────────────────────────────────
        logger.info("[cycle %d] próximo ciclo en %ds", cycle, interval_sec)
        time.sleep(interval_sec)


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(description="ATLAS autonomous loop")
    parser.add_argument("--port", type=int, default=None, help="Puerto del servidor Quant (auto-detect si omitido)")
    parser.add_argument("--interval", type=int, default=90, help="Segundos entre ciclos (default: 90)")
    parser.add_argument("--max-per-cycle", type=int, default=1, help="Max candidatos por ciclo (default: 1)")
    parser.add_argument("--api-key", default="atlas-quant-local", help="API key")
    args = parser.parse_args()

    if args.port:
        base = f"http://127.0.0.1:{args.port}"
        r = _api_call(base, "/health", api_key=args.api_key)
        if not r or r.get("status") != "ok":
            print(f"ERROR: servidor no responde en puerto {args.port}")
            return 1
    else:
        base = _detect_base([8792, 8795, 8791], args.api_key)
        if not base:
            print("ERROR: no se encontró ningún servidor Quant activo en puertos 8792/8795/8791")
            return 1

    try:
        run_loop(base, args.interval, args.max_per_cycle, args.api_key)
    except KeyboardInterrupt:
        logger.info("Loop detenido por el usuario (Ctrl+C)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
