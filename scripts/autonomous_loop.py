#!/usr/bin/env python
"""autonomous_loop.py — Loop autonomo externo ATLAS-Quant v2.

Mejoras v2:
  - Gate de horario de mercado (9:30-15:45 ET solo)
  - Deduplicacion: no envia el mismo simbolo hasta el proximo ciclo
  - Timeout reducido a 25s para no bloquear si el brain no responde
  - Cooldown por simbolo (no repetir dentro de N minutos)
  - Arranged para Task Scheduler (start/stop limpio)

Uso:
    python scripts/autonomous_loop.py
    python scripts/autonomous_loop.py --port 8792 --interval 120
    python scripts/autonomous_loop.py --no-market-hours-gate  # para testing
"""
from __future__ import annotations

import argparse
import json
import logging
import logging.handlers
import sys
import time
from collections import defaultdict
from datetime import datetime, timezone, timedelta
from pathlib import Path
from urllib.request import Request, urlopen
from urllib.error import URLError, HTTPError

# ── Logging ───────────────────────────────────────────────────────────────────
_ROOT = Path(__file__).resolve().parents[1]
_LOG_FILE = _ROOT / "logs" / "atlas_live_loop.log"
_LOG_FILE.parent.mkdir(parents=True, exist_ok=True)

_fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s", "%H:%M:%S")
_root_logger = logging.getLogger()
_root_logger.setLevel(logging.INFO)
_sh = logging.StreamHandler(sys.stdout)
_sh.setFormatter(_fmt)
_fh = logging.handlers.RotatingFileHandler(
    _LOG_FILE, maxBytes=20 * 1024 * 1024, backupCount=5, encoding="utf-8"
)
_fh.setFormatter(_fmt)
_root_logger.addHandler(_sh)
_root_logger.addHandler(_fh)

logger = logging.getLogger("atlas.execution.live_loop")

# ── Constantes ────────────────────────────────────────────────────────────────
# Horario mercado ET: 9:30 - 15:45 (cierre 15 min antes para evitar slippage final)
_MARKET_OPEN_H,  _MARKET_OPEN_M  = 9,  30
_MARKET_CLOSE_H, _MARKET_CLOSE_M = 15, 45
# Cooldown: un simbolo no se repite dentro de este intervalo (minutos)
_SYMBOL_COOLDOWN_MIN = 30


# ── Helpers HTTP ──────────────────────────────────────────────────────────────

def _api_call(
    base: str,
    path: str,
    method: str = "GET",
    body: dict | None = None,
    api_key: str = "atlas-quant-local",
    timeout: int = 25,
) -> dict | None:
    url = f"{base}{path}"
    headers = {"x-api-key": api_key, "Content-Type": "application/json"}
    data = json.dumps(body).encode() if body else None
    try:
        req = Request(url, data=data, headers=headers, method=method)
        with urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode())
    except (URLError, HTTPError) as e:
        logger.debug("API %s %s: %s", method, path, e)
        return None
    except Exception as e:
        logger.debug("API error %s %s: %s", method, path, e)
        return None


def _detect_base(ports: list[int], api_key: str) -> str | None:
    for port in ports:
        base = f"http://127.0.0.1:{port}"
        r = _api_call(base, "/health", timeout=5, api_key=api_key)
        if r and r.get("status") == "ok":
            # Preferir el que responde scanner
            scan = _api_call(base, "/scanner/report", timeout=8, api_key=api_key)
            if scan and scan.get("ok"):
                return base
    # Fallback: primer que responde health
    for port in ports:
        base = f"http://127.0.0.1:{port}"
        r = _api_call(base, "/health", timeout=5, api_key=api_key)
        if r and r.get("status") == "ok":
            return base
    return None


# ── Gate de horario de mercado ────────────────────────────────────────────────

def _now_et() -> datetime:
    """Hora actual en ET (UTC-4 EDT / UTC-5 EST). Aproximacion sin pytz."""
    utc = datetime.now(timezone.utc)
    # EDT: segunda domingo de marzo a primera domingo de noviembre
    # Aproximacion simple: UTC-4 de marzo a noviembre, UTC-5 el resto
    month = utc.month
    offset = -4 if 3 <= month <= 11 else -5
    return utc + timedelta(hours=offset)


def _is_market_open(override: bool = False) -> bool:
    if override:
        return True
    now = _now_et()
    # Saltar fines de semana
    if now.weekday() >= 5:
        return False
    market_open  = now.replace(hour=_MARKET_OPEN_H,  minute=_MARKET_OPEN_M,  second=0, microsecond=0)
    market_close = now.replace(hour=_MARKET_CLOSE_H, minute=_MARKET_CLOSE_M, second=0, microsecond=0)
    return market_open <= now <= market_close


def _minutes_to_open() -> float:
    """Minutos hasta la proxima apertura del mercado."""
    now = _now_et()
    if now.weekday() >= 5:
        days_ahead = 7 - now.weekday()
    else:
        days_ahead = 0
    target = now.replace(hour=_MARKET_OPEN_H, minute=_MARKET_OPEN_M, second=0, microsecond=0)
    target += timedelta(days=days_ahead)
    if target <= now:
        target += timedelta(days=1)
        while target.weekday() >= 5:
            target += timedelta(days=1)
    return max(0.0, (target - now).total_seconds() / 60)


# ── Loop principal ────────────────────────────────────────────────────────────

def run_loop(
    base: str,
    interval_sec: int,
    max_per_cycle: int,
    api_key: str,
    no_market_hours_gate: bool = False,
) -> None:
    logger.info("=" * 60)
    logger.info("ATLAS autonomous_loop v2 iniciado")
    logger.info("  base=%s interval=%ds max_per_cycle=%d", base, interval_sec, max_per_cycle)
    logger.info("  market_hours_gate=%s", not no_market_hours_gate)
    logger.info("  symbol_cooldown=%dmin", _SYMBOL_COOLDOWN_MIN)
    logger.info("=" * 60)

    cycle = 0
    total_submitted = 0
    # Cooldown: {symbol -> datetime ultima orden}
    symbol_last_order: dict[str, datetime] = {}

    while True:
        now_et = _now_et()

        # ── Gate horario de mercado ───────────────────────────────────────────
        if not _is_market_open(override=no_market_hours_gate):
            mins = _minutes_to_open()
            if cycle == 0 or int(mins) % 30 == 0:  # log cada 30 min
                logger.info(
                    "Mercado cerrado (%s ET). Proximo ciclo en %.0f min.",
                    now_et.strftime("%H:%M"), mins
                )
            sleep_sec = min(300, max(30, (mins - 1) * 60)) if mins > 1 else 60
            time.sleep(sleep_sec)
            continue

        cycle += 1
        t_cycle = time.perf_counter()
        logger.info(
            "[cycle %d] %s ET -- %s UTC",
            cycle,
            now_et.strftime("%H:%M:%S"),
            datetime.now(timezone.utc).strftime("%H:%M:%S"),
        )

        # ── Verificar estado operacional ──────────────────────────────────────
        op = _api_call(base, "/api/v2/quant/operation/status", timeout=10, api_key=api_key)
        if not op or not op.get("ok"):
            op = _api_call(base, "/operation/status", timeout=10, api_key=api_key)

        if op and op.get("data"):
            d = op["data"]
            auton_mode   = d.get("config", {}).get("auton_mode", "unknown")
            auton_active = d.get("auton_mode_active", False)
            failsafe     = d.get("failsafe", {}).get("active", False)

            if failsafe:
                logger.warning("[cycle %d] FAILSAFE activo - saltando", cycle)
                time.sleep(interval_sec)
                continue

            if not auton_active or auton_mode != "paper_autonomous":
                logger.warning("[cycle %d] auton_mode=%s active=%s - no autonomo", cycle, auton_mode, auton_active)
                time.sleep(interval_sec)
                continue

        else:
            logger.warning("[cycle %d] Sin estado operacional - continuando de todas formas", cycle)

        # ── Obtener candidatos ────────────────────────────────────────────────
        scan = _api_call(base, "/scanner/report", timeout=15, api_key=api_key)
        if not scan or not scan.get("ok"):
            scan = _api_call(base, "/api/v2/quant/scanner/report", timeout=15, api_key=api_key)

        if not scan or not scan.get("data"):
            logger.info("[cycle %d] Sin candidatos del scanner", cycle)
            time.sleep(interval_sec)
            continue

        candidates = scan["data"].get("candidates", [])
        if not candidates:
            logger.info("[cycle %d] Scanner sin candidatos activos", cycle)
            time.sleep(interval_sec)
            continue

        # Ordenar por score
        sorted_cands = sorted(
            candidates,
            key=lambda c: float(c.get("selection_score") or 0),
            reverse=True,
        )

        # ── Filtrar cooldown ──────────────────────────────────────────────────
        now_utc = datetime.now(timezone.utc)
        cooldown_td = timedelta(minutes=_SYMBOL_COOLDOWN_MIN)
        eligible = []
        for c in sorted_cands:
            sym = str(c.get("symbol") or "").strip()
            if not sym:
                continue
            last = symbol_last_order.get(sym)
            if last and (now_utc - last) < cooldown_td:
                mins_left = int((cooldown_td - (now_utc - last)).total_seconds() / 60)
                logger.debug("  %s en cooldown (%d min restantes)", sym, mins_left)
                continue
            eligible.append(c)
            if len(eligible) >= max_per_cycle:
                break

        if not eligible:
            cooldown_syms = [str(c.get("symbol","")) for c in sorted_cands[:3]]
            logger.info("[cycle %d] Todos los candidatos en cooldown: %s", cycle, ", ".join(cooldown_syms))
            time.sleep(interval_sec)
            continue

        logger.info(
            "[cycle %d] %d candidatos | %d elegibles (cooldown %dmin) -> procesando top %d",
            cycle, len(candidates), len(eligible), _SYMBOL_COOLDOWN_MIN, len(eligible)
        )

        # ── Enviar ordenes ────────────────────────────────────────────────────
        cycle_submitted = 0
        for cand in eligible:
            sym       = str(cand.get("symbol") or "").strip()
            score     = float(cand.get("selection_score") or 0)
            direction = str(cand.get("direction") or "long").lower()
            side      = "buy" if direction in {"long", "alcista", "bull", "up"} else "sell_short"
            tf        = cand.get("timeframe", "?")

            logger.info("[cycle %d] -> %s score=%.1f dir=%s tf=%s", cycle, sym, score, direction, tf)

            order_body = {
                "order": {
                    "symbol": sym,
                    "side": side,
                    "size": 1,
                    "order_type": "market",
                    "account_scope": "paper",
                    "preview": False,
                    "asset_class": "equity",
                },
                "action": "submit",
                "capture_context": False,  # evita esperar al brain (reduce 15s de timeout)
            }

            result = _api_call(
                base, "/api/v2/quant/operation/test-cycle",
                method="POST", body=order_body, timeout=25, api_key=api_key,
            )
            if not result:
                result = _api_call(
                    base, "/operation/test-cycle",
                    method="POST", body=order_body, timeout=25, api_key=api_key,
                )

            if result and result.get("ok"):
                data_r   = result.get("data", {})
                blocked  = data_r.get("blocked", False)
                decision = data_r.get("decision", "?")
                reasons  = data_r.get("reasons", [])
                exec_r   = data_r.get("execution", {}) or {}
                tradier  = (exec_r.get("response") or {}).get("tradier_response") or {}
                order_id = tradier.get("id")

                if blocked:
                    reason_str = ", ".join(str(r) for r in reasons[:2]) if reasons else "?"
                    logger.info("  BLOCKED %s | %s | %s", sym, decision, reason_str)
                elif order_id:
                    cycle_submitted += 1
                    total_submitted += 1
                    symbol_last_order[sym] = now_utc
                    logger.info(
                        "  SUBMIT  %s | id=%s | decision=%s | score=%.1f",
                        sym, order_id, decision, score
                    )
                else:
                    # submit_ready pero sin id (broker no respondio con id)
                    cycle_submitted += 1
                    total_submitted += 1
                    symbol_last_order[sym] = now_utc
                    logger.info(
                        "  SUBMIT  %s | decision=%s | score=%.1f | (sin id broker)",
                        sym, decision, score
                    )
            else:
                logger.warning("  ERROR   %s | sin respuesta del servidor", sym)

        elapsed_ms = (time.perf_counter() - t_cycle) * 1000
        logger.info(
            "[cycle %d] FIN | submitted=%d | total=%d | %.0fms",
            cycle, cycle_submitted, total_submitted, elapsed_ms
        )
        time.sleep(interval_sec)


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(description="ATLAS autonomous loop v2")
    parser.add_argument("--port",               type=int,   default=None)
    parser.add_argument("--interval",           type=int,   default=120,  help="Segundos entre ciclos (default 120)")
    parser.add_argument("--max-per-cycle",      type=int,   default=1)
    parser.add_argument("--api-key",            default="atlas-quant-local")
    parser.add_argument("--no-market-hours-gate", action="store_true", help="Operar fuera de horario (testing)")
    args = parser.parse_args()

    if args.port:
        base = f"http://127.0.0.1:{args.port}"
        r = _api_call(base, "/health", timeout=5, api_key=args.api_key)
        if not r or r.get("status") != "ok":
            print(f"ERROR: servidor no responde en puerto {args.port}")
            return 1
    else:
        base = _detect_base([8792, 8795], args.api_key)
        if not base:
            print("ERROR: no se encontro servidor Quant activo en puertos 8792/8795")
            return 1

    logger.info("Servidor: %s", base)

    try:
        run_loop(base, args.interval, args.max_per_cycle, args.api_key, args.no_market_hours_gate)
    except KeyboardInterrupt:
        logger.info("Loop detenido (Ctrl+C)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
