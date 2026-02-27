#!/usr/bin/env python3
"""nexus_worker.py — Controlador físico ATLAS_NEXUS. Escucha el Dashboard, ejecuta comandos y envía ACK."""
from __future__ import annotations

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

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("nexus_worker")

DASHBOARD_BASE = (os.getenv("ATLAS_DASHBOARD_URL") or "http://127.0.0.1:8791").rstrip("/")
POLL_INTERVAL = float(os.getenv("NEXUS_WORKER_POLL_INTERVAL", "1.0"))
SIMULATE_MOTOR_SEC = float(os.getenv("NEXUS_WORKER_SIMULATE_MOTOR_SEC", "2.0"))


class HardwareController:
    """Simulador del puente físico: motores/actuadores. Por ahora imprime y simula latencia."""

    def __init__(self, motor_delay_sec: float = SIMULATE_MOTOR_SEC):
        self._motor_delay_sec = motor_delay_sec

    async def execute(self, target: str, action: str, value: object) -> None:
        """Ejecuta la acción en el hardware (simulada)."""
        sep = "=" * 50
        print(f"\n{sep}")
        print(f"  [NEXUS] [{target}] Ejecutando: {action} -> {value}")
        print(f"{sep}\n")
        await asyncio.sleep(self._motor_delay_sec)
        logger.info("[%s] Comando completado: %s -> %s", target, action, value)


async def fetch_state(session) -> dict | None:
    """GET /api/push/state. Retorna None si falla la conexión."""
    try:
        import aiohttp
        async with session.get(
            f"{DASHBOARD_BASE}/api/push/state",
            timeout=aiohttp.ClientTimeout(total=5),
        ) as r:
            if r.status != 200:
                return None
            return await r.json()
    except Exception as e:
        logger.debug("fetch_state: %s", e)
        return None


async def send_ack(session, payload: dict) -> bool:
    """POST /api/push/ack. Retorna True si se envió correctamente."""
    try:
        import aiohttp
        async with session.post(
            f"{DASHBOARD_BASE}/api/push/ack",
            json=payload,
            timeout=aiohttp.ClientTimeout(total=5),
        ) as r:
            return r.status == 200
    except Exception as e:
        logger.debug("send_ack: %s", e)
        return False


async def run_worker() -> None:
    """Bucle infinito: polling al Dashboard, ejecutar comando, ACK. Tolerante a fallos."""
    import aiohttp
    controller = HardwareController()
    last_executed_ts: float | None = None

    async with aiohttp.ClientSession() as session:
        while True:
            try:
                data = await fetch_state(session)
                if not data or not data.get("ok"):
                    await asyncio.sleep(POLL_INTERVAL)
                    continue

                state = data.get("state")
                if not state or not isinstance(state, dict):
                    await asyncio.sleep(POLL_INTERVAL)
                    continue

                ts = state.get("ts")
                if ts is None:
                    await asyncio.sleep(POLL_INTERVAL)
                    continue

                if last_executed_ts is not None and ts == last_executed_ts:
                    await asyncio.sleep(POLL_INTERVAL)
                    continue

                target = state.get("target", "NEXUS_ARM")
                action = state.get("action", "update_state")
                value = state.get("value", 1)

                await controller.execute(target, action, value)
                last_executed_ts = ts

                ack_payload = {
                    "target": target,
                    "status": "completed",
                    "executed_ts": ts,
                }
                if await send_ack(session, ack_payload):
                    logger.info("ACK enviado: target=%s ts=%s", target, ts)
                else:
                    logger.warning("ACK no enviado (Dashboard no respondió)")

            except asyncio.CancelledError:
                raise
            except Exception as e:
                logger.debug("ciclo: %s", e)
            await asyncio.sleep(POLL_INTERVAL)


def main() -> None:
    logger.info("NEXUS Worker iniciando (Dashboard=%s, poll=%.1fs)", DASHBOARD_BASE, POLL_INTERVAL)
    try:
        asyncio.run(run_worker())
    except KeyboardInterrupt:
        logger.info("NEXUS Worker detenido")


if __name__ == "__main__":
    main()
