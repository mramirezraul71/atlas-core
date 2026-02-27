#!/usr/bin/env python3
"""Ciclo Atlas: Snapshot (5s) -> Revisión -> Push al Dashboard. Async, self-healing."""
import asyncio
import logging
import os
import sys
import time
from pathlib import Path

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE))
ENV = BASE / "config" / "atlas.env"
if ENV.exists():
    from dotenv import load_dotenv
    load_dotenv(ENV, override=True)

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(levelname)s %(message)s")
logger = logging.getLogger("atlas.cycle")

SNAPSHOT_INTERVAL = float(os.getenv("ATLAS_CYCLE_SNAPSHOT_INTERVAL", "5.0"))


def capture_snapshot() -> dict:
    ts = time.time()
    env_keys = list(os.environ.keys())[:10]
    return {"timestamp": ts, "env_sample": {k: "***" if "KEY" in k.upper() or "SECRET" in k.upper() else str(os.environ.get(k, ""))[:40] for k in env_keys}}


def evaluate_snapshot(snap: dict) -> dict | None:
    if not snap or not snap.get("timestamp"):
        return None
    return {"target": "NEXUS_ARM", "action": "update_state", "value": 1}


async def run_cycle(client) -> None:
    snap = capture_snapshot()
    cmd = evaluate_snapshot(snap)
    if not cmd:
        return
    state = await client.get_state()
    last = (state or {}).get("state") if isinstance(state, dict) else None
    if last and last.get("target") == cmd.get("target") and last.get("value") == cmd.get("value"):
        logger.debug("skip duplicate command")
        return
    ok = await client.send_command(cmd.get("target", "NEXUS_ARM"), cmd.get("action", "update_state"), cmd.get("value", 1))
    if ok:
        logger.info("push ok: %s", cmd)


async def main() -> None:
    from modules.brain.dashboard_client import AtlasDashboardClient
    client = AtlasDashboardClient()
    backoff = 1.0
    backoff_max = 60.0
    try:
        while True:
            try:
                await run_cycle(client)
                backoff = 1.0
            except Exception as e:
                logger.exception("cycle error: %s", e)
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, backoff_max)
            await asyncio.sleep(SNAPSHOT_INTERVAL)
    finally:
        await client.close()


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--seconds", type=float, default=0, help="Run N seconds then exit (0=forever)")
    args = p.parse_args()

    async def _run():
        if args.seconds > 0:
            await asyncio.wait_for(main(), timeout=args.seconds)
        else:
            await main()

    try:
        asyncio.run(_run())
    except asyncio.TimeoutError:
        logger.info("cycle finished after %.1fs", args.seconds)
    except KeyboardInterrupt:
        pass
