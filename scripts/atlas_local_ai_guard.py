#!/usr/bin/env python3
"""
ATLAS Local AI Guard

Verifica el stack local de IA frente a un perfil objetivo, compara el routing
actual por roles con los modelos realmente instalados en Ollama y emite eventos
observables a bitácora / Telegram sin tocar la estructura del repo.

Uso:
  python scripts/atlas_local_ai_guard.py
  python scripts/atlas_local_ai_guard.py --profile conservative --json
  python scripts/atlas_local_ai_guard.py --no-emit
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence


REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from modules.humanoid.ans.evolution_bitacora import append_evolution_log
from modules.humanoid.notify import send_telegram


CURRENT_ROUTE_DEFAULTS: Dict[str, str] = {
    "FAST": "ollama:llama3.2:3b",
    "CHAT": "ollama:qwen3:30b",
    "CODE": "ollama:qwen3-coder:30b",
    "REASON": "ollama:qwen3:30b",
    "TOOLS": "ollama:qwen3:30b",
    "VISION": "ollama:qwen3-vl:30b",
    "ARCHITECT": "ollama:mistral-small:24b",
    "OPTIMIZER": "ollama:qwen3-coder:30b",
}

TARGET_PROFILES: Dict[str, Dict[str, str]] = {
    "conservative": {
        "FAST": "ollama:llama3.2:3b",
        "CHAT": "ollama:qwen3:30b",
        "CODE": "ollama:qwen3-coder:30b",
        "REASON": "ollama:deepseek-r1:14b",
        "TOOLS": "ollama:qwen3:30b",
        "VISION": "ollama:qwen3-vl:30b",
        "ARCHITECT": "ollama:mistral-small:24b",
        "OPTIMIZER": "ollama:qwen3-coder:30b",
    },
    "stronger": {
        "FAST": "ollama:llama3.2:3b",
        "CHAT": "ollama:qwen3:30b",
        "CODE": "ollama:qwen3-coder:30b",
        "REASON": "ollama:qwen3:30b",
        "TOOLS": "ollama:qwen3:30b",
        "VISION": "ollama:qwen3-vl:30b",
        "ARCHITECT": "ollama:mistral-small:24b",
        "OPTIMIZER": "ollama:qwen3-coder:30b",
    },
}


@dataclass
class GuardSummary:
    profile: str
    installed_models: List[str]
    current_routes: Dict[str, str]
    target_routes: Dict[str, str]
    missing_target_models: List[str]
    degraded_current_routes: Dict[str, str]
    routes_needing_upgrade: Dict[str, Dict[str, str]]

    def as_dict(self) -> Dict[str, object]:
        return {
            "profile": self.profile,
            "installed_models": self.installed_models,
            "current_routes": self.current_routes,
            "target_routes": self.target_routes,
            "missing_target_models": self.missing_target_models,
            "degraded_current_routes": self.degraded_current_routes,
            "routes_needing_upgrade": self.routes_needing_upgrade,
        }


def _env_route(route: str) -> str:
    key = f"AI_{route}_MODEL"
    return (os.getenv(key) or "").strip() or CURRENT_ROUTE_DEFAULTS[route]


def _ollama_model_name(full_key: str) -> str:
    return full_key.split(":", 1)[1] if full_key.startswith("ollama:") else full_key


def _list_ollama_models() -> List[str]:
    proc = subprocess.run(
        ["ollama", "list"],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError((proc.stderr or proc.stdout or "ollama list failed").strip())
    lines = [ln.rstrip() for ln in proc.stdout.splitlines() if ln.strip()]
    if len(lines) <= 1:
        return []
    names: List[str] = []
    for line in lines[1:]:
        parts = line.split()
        if parts:
            names.append(parts[0].strip())
    return names


def _emit_bitacora(event: str, message: str, ok: bool) -> None:
    append_evolution_log(
        message=f"[LOCAL_AI:{event}] {message}",
        ok=ok,
        source="local_ai",
    )


async def _emit_telegram(message: str) -> bool:
    return await send_telegram(message)


def _maybe_emit_observability(event: str, message: str, ok: bool, severity: str) -> None:
    _emit_bitacora(event, message, ok)
    if severity.lower() not in {"warning", "critical"}:
        return
    try:
        asyncio.run(_emit_telegram(f"ATLAS LOCAL AI {severity.upper()}\n{message}"))
    except Exception:
        pass


def build_summary(profile: str, installed_models: Sequence[str]) -> GuardSummary:
    target_routes = TARGET_PROFILES[profile]
    current_routes = {route: _env_route(route) for route in CURRENT_ROUTE_DEFAULTS}
    installed_set = set(installed_models)

    target_models = sorted({_ollama_model_name(model) for model in target_routes.values()})
    missing_target_models = [name for name in target_models if name not in installed_set]

    degraded_current_routes: Dict[str, str] = {}
    routes_needing_upgrade: Dict[str, Dict[str, str]] = {}
    for route, current_model in current_routes.items():
        current_name = _ollama_model_name(current_model)
        if current_name not in installed_set:
            degraded_current_routes[route] = current_model
        target_model = target_routes[route]
        if current_model != target_model:
            routes_needing_upgrade[route] = {
                "current": current_model,
                "target": target_model,
            }

    return GuardSummary(
        profile=profile,
        installed_models=list(installed_models),
        current_routes=current_routes,
        target_routes=target_routes,
        missing_target_models=missing_target_models,
        degraded_current_routes=degraded_current_routes,
        routes_needing_upgrade=routes_needing_upgrade,
    )


def _print_human(summary: GuardSummary) -> None:
    print(f"ATLAS Local AI Guard — profile: {summary.profile}")
    print("=" * 72)
    print(f"Installed Ollama models: {len(summary.installed_models)}")
    print("Current routing:")
    for route, model in summary.current_routes.items():
        state = "OK" if route not in summary.degraded_current_routes else "MISSING"
        print(f"  - {route:<10} {model:<34} [{state}]")
    print("\nTarget routing:")
    for route, model in summary.target_routes.items():
        print(f"  - {route:<10} {model}")
    print("\nMissing target models:")
    if summary.missing_target_models:
        for model in summary.missing_target_models:
            print(f"  - {model}")
    else:
        print("  - none")
    print("\nRoutes needing upgrade:")
    if summary.routes_needing_upgrade:
        for route, data in summary.routes_needing_upgrade.items():
            print(f"  - {route}: {data['current']} -> {data['target']}")
    else:
        print("  - none")


def main() -> int:
    parser = argparse.ArgumentParser(description="ATLAS local AI stack guard")
    parser.add_argument(
        "--profile",
        choices=sorted(TARGET_PROFILES.keys()),
        default="stronger",
        help="Perfil objetivo de modelos locales",
    )
    parser.add_argument("--json", action="store_true", help="Salida JSON")
    parser.add_argument(
        "--no-emit",
        action="store_true",
        help="No emitir eventos a bitácora / Telegram",
    )
    args = parser.parse_args()

    installed = _list_ollama_models()
    summary = build_summary(args.profile, installed)

    if not args.no_emit:
        _maybe_emit_observability(
            "model_inventory_verified",
            f"Inventario local verificado: {len(summary.installed_models)} modelos en Ollama.",
            ok=True,
            severity="info",
        )
        if summary.missing_target_models:
            _maybe_emit_observability(
                "model_pull_failed",
                (
                    f"Perfil {summary.profile} incompleto. Faltan: "
                    + ", ".join(summary.missing_target_models)
                ),
                ok=False,
                severity="warning",
            )
        else:
            _maybe_emit_observability(
                "local_stack_ready",
                f"Perfil {summary.profile} completo y disponible en local.",
                ok=True,
                severity="info",
            )
        for route, model in summary.degraded_current_routes.items():
            _maybe_emit_observability(
                "role_degraded",
                f"Ruta {route} degradada: el modelo actual {model} no está instalado.",
                ok=False,
                severity="warning",
            )

    if args.json:
        print(json.dumps(summary.as_dict(), indent=2, ensure_ascii=False))
    else:
        _print_human(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
