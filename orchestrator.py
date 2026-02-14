#!/usr/bin/env python3
"""Cerebro central ATLAS: bucle de consciencia continua. Unifica Dashboard, Navegador y Codificador."""
from __future__ import annotations

import asyncio
import logging
import os
import sys
import traceback
from datetime import datetime
from pathlib import Path

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE))
ENV = BASE / "config" / "atlas.env"
if ENV.exists():
    from dotenv import load_dotenv
    load_dotenv(ENV, override=True)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("atlas.orchestrator")

ANS_MEMORY_LOG = BASE / "logs" / "ANS_memory.log"
CYCLE_SLEEP = int(os.environ.get("ORCHESTRATOR_CYCLE_SLEEP", "60"))
RECOVERY_SLEEP = int(os.environ.get("ORCHESTRATOR_RECOVERY_SLEEP", "5"))
TRADIER_PRICE_THRESHOLD = float(os.environ.get("ATLAS_TRADIER_PRICE_THRESHOLD", "2.0"))
TRADIER_API_KEY = os.environ.get("TRADIER_API_KEY", "").strip()
ATLAS_WORKSPACE = BASE / "atlas_workspace"
ATLAS_WORKSPACE.mkdir(parents=True, exist_ok=True)


def _fetch_tradier_options_chain(ticker: str) -> dict:
    """Obtiene cadena de opciones (Tradier). Sin API key usa mock con Call/Put de SPY."""
    if not TRADIER_API_KEY:
        import random
        bid_call = round(random.uniform(0.9, 2.8), 2)
        bid_put = round(random.uniform(0.7, 2.2), 2)
        return {
            "ticker": ticker or "SPY",
            "calls": [{"strike": 600, "bid": bid_call, "symbol": "SPY250117C00600000"}],
            "puts": [{"strike": 598, "bid": bid_put, "symbol": "SPY250117P00598000"}],
        }
    try:
        import urllib.request
        url = f"https://api.tradier.com/v1/markets/options/chains?symbol={ticker}&expiration=2025-01-17"
        req = urllib.request.Request(url, headers={"Authorization": f"Bearer {TRADIER_API_KEY}", "Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=10) as r:
            import json
            return json.loads(r.read().decode("utf-8"))
    except Exception as e:
        logger.debug("Tradier API: %s", e)
        return {"ticker": ticker, "calls": [{"strike": 600, "bid": 1.5}], "puts": [{"strike": 598, "bid": 1.2}]}


def _append_traceback(exc: Exception) -> None:
    try:
        ANS_MEMORY_LOG.parent.mkdir(parents=True, exist_ok=True)
        with open(ANS_MEMORY_LOG, "a", encoding="utf-8") as f:
            f.write(f"\n--- {datetime.utcnow().isoformat()}Z ---\n")
            traceback.print_exc(file=f)
    except Exception:
        pass


class AtlasOrchestrator:
    """Cerebro central: Snapshot -> Evaluación -> Delegación (Navegador | Codificador | Dashboard). Bucle de supervivencia extrema."""

    def __init__(self) -> None:
        from modules.brain.dashboard_client import AtlasDashboardClient
        from agents.navigator_agent import AtlasNavigator
        from agents.coder_agent import AtlasCoder
        self._dashboard = AtlasDashboardClient()
        self._navigator = AtlasNavigator()
        self._coder = AtlasCoder()
        self._cycle = 0

    async def _snapshot(self) -> dict:
        """Percepción: estado del Dashboard y alertas locales."""
        state = await self._dashboard.get_state()
        alerts = []
        try:
            alerts_file = BASE / "logs" / "alerts.txt"
            if alerts_file.exists():
                with open(alerts_file, "r", encoding="utf-8", errors="ignore") as f:
                    alerts = [l.strip() for l in f.readlines()[-10:] if l.strip()]
        except Exception:
            pass
        return {"dashboard_state": state, "alerts": alerts, "cycle": self._cycle}

    def _evaluate_snapshot(self, snap: dict) -> str:
        """Razonamiento: integración Tradier. Si prima < umbral X -> Oportunidad Detectada; si no, elige herramienta."""
        self._cycle += 1
        try:
            chain = _fetch_tradier_options_chain("SPY")
            calls = chain.get("calls") or []
            puts = chain.get("puts") or []
            for c in calls:
                bid = c.get("bid") if isinstance(c.get("bid"), (int, float)) else None
                if bid is not None and bid < TRADIER_PRICE_THRESHOLD:
                    snap["_oportunidad"] = {"tipo": "SPY_CALL", "bid": bid, "strike": c.get("strike")}
                    return "oportunidad_trade"
            for p in puts:
                bid = p.get("bid") if isinstance(p.get("bid"), (int, float)) else None
                if bid is not None and bid < TRADIER_PRICE_THRESHOLD:
                    snap["_oportunidad"] = {"tipo": "SPY_PUT", "bid": bid, "strike": p.get("strike")}
                    return "oportunidad_trade"
        except Exception as e:
            logger.debug("evaluate tradier: %s", e)
        state = (snap.get("dashboard_state") or {}) if isinstance(snap.get("dashboard_state"), dict) else {}
        cmd = state.get("state")
        if cmd and isinstance(cmd, dict) and cmd.get("target"):
            return "dashboard"
        if snap.get("alerts") and "navigat" in str(snap.get("alerts", "")).lower():
            return "navigator"
        if self._cycle % 3 == 0:
            return "navigator"
        if self._cycle % 3 == 1:
            return "coder"
        return "dashboard"

    async def _delegate_navigator(self) -> None:
        nav = self._navigator
        try:
            await nav.start_browser()
            out = await nav.fetch_page_content("https://example.com")
            await nav.close_browser()
            if out.get("status") == "ok":
                logger.info("[Delegación] Navegador: title=%s, text_len=%s", out.get("title", ""), len(out.get("text", "")))
            else:
                logger.warning("[Delegación] Navegador: %s", out.get("message", ""))
        except Exception as e:
            logger.error("[Delegación] Navegador falló: %s", e)
            raise
        finally:
            try:
                await nav.close_browser()
            except Exception:
                pass

    def _delegate_coder(self) -> None:
        result = self._coder.auto_program_loop(
            "Genera un script que imprima la fecha y hora actual (solo código Python).",
            max_intentos=2,
            filename="lifecycle_script.py",
        )
        if result.get("ok"):
            logger.info("[Delegación] Codificador: %s", (result.get("stdout") or "")[:200])
        else:
            logger.warning("[Delegación] Codificador: %s", result.get("stderr", "")[:200])

    async def _delegate_dashboard(self) -> None:
        ok = await self._dashboard.send_command("NEXUS_ARM", "update_state", 1)
        if ok:
            logger.info("[Delegación] Dashboard: comando enviado (NEXUS_ARM)")
        else:
            logger.warning("[Delegación] Dashboard: no se pudo enviar comando")

    async def _delegate_oportunidad_trade(self, snap: dict) -> None:
        """Oportunidad detectada: 1) POST ALERT_TRADE al Dashboard, 2) trade_log.txt vía AtlasCoder."""
        opp = snap.get("_oportunidad") or {}
        tipo = opp.get("tipo", "SPY_CALL")
        bid = opp.get("bid", 0)
        strike = opp.get("strike", "")
        ts = datetime.utcnow().isoformat() + "Z"
        ok = await self._dashboard.send_command("NEXUS_ARM", "ALERT_TRADE", tipo)
        if ok:
            logger.info("[Delegación] Oportunidad: ALERT_TRADE enviado target=NEXUS_ARM value=%s", tipo)
        else:
            logger.warning("[Delegación] Oportunidad: fallo al enviar ALERT_TRADE")
        log_line = f"[{ts}] Oportunidad detectada: {tipo} bid={bid} strike={strike}\n"
        script = f'''with open("trade_log.txt", "a", encoding="utf-8") as f:\n    f.write({repr(log_line)})\n'''
        try:
            self._coder.write_script("_write_trade_log.py", script)
            r = self._coder.execute_script("_write_trade_log.py")
            if r.get("ok"):
                logger.info("[Delegación] Oportunidad: trade_log.txt actualizado en atlas_workspace/")
            else:
                logger.warning("[Delegación] Oportunidad: fallo escribir trade_log %s", r.get("stderr", "")[:80])
        except Exception as e:
            logger.warning("[Delegación] Oportunidad: coder trade_log %s", e)
            try:
                with (ATLAS_WORKSPACE / "trade_log.txt").open("a", encoding="utf-8") as f:
                    f.write(log_line)
            except Exception:
                pass

    async def run_lifecycle(self) -> None:
        """Bucle de consciencia continua. Nunca se detiene."""
        logger.info("ATLAS Orquestador despierta — bucle de vida iniciado (ciclo cada %ss)", CYCLE_SLEEP)
        while True:
            try:
                snap = await self._snapshot()
                logger.info("[Snapshot] ciclo=%s dashboard_ok=%s alertas=%s",
                    snap.get("cycle"),
                    bool((snap.get("dashboard_state") or {}).get("ok")),
                    len(snap.get("alerts", [])),
                )
                decision = self._evaluate_snapshot(snap)
                logger.info("[Evaluación] decisión=%s", decision)
                if decision == "oportunidad_trade":
                    await self._delegate_oportunidad_trade(snap)
                elif decision == "navigator":
                    await self._delegate_navigator()
                elif decision == "coder":
                    self._delegate_coder()
                else:
                    await self._delegate_dashboard()
                logger.info("[Reposo] próximo ciclo en %ss", CYCLE_SLEEP)
                await asyncio.sleep(CYCLE_SLEEP)
            except KeyboardInterrupt:
                logger.info("ATLAS Orquestador detenido por usuario")
                raise
            except Exception as e:
                logger.error("Ciclo falló (auto-sanación): %s", e)
                _append_traceback(e)
                logger.info("Recuperación en %ss...", RECOVERY_SLEEP)
                await asyncio.sleep(RECOVERY_SLEEP)


def main() -> None:
    try:
        from modules.brain.dashboard_client import AtlasDashboardClient
        from agents.navigator_agent import AtlasNavigator
        from agents.coder_agent import AtlasCoder
    except ImportError as e:
        logger.error("Import fallido: %s", e)
        sys.exit(1)
    orch = AtlasOrchestrator()
    try:
        asyncio.run(orch.run_lifecycle())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
