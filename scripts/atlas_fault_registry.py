#!/usr/bin/env python3
"""
ATLAS Fault Registry Builder

Construye un inventario transversal del repo para el futuro ATLAS Fault Manager.
No mueve estructura ni modifica módulos existentes. Descubre dominios, carpetas,
archivos, artefactos de health/healing/supervisión y genera un registro en state/.
Además emite un resumen a bitácora y Telegram (warning/critical) cuando aplica.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, Iterable, List

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from modules.humanoid.ans.evolution_bitacora import append_evolution_log
from modules.humanoid.notify import send_telegram


STATE_DIR = REPO_ROOT / "state"
REGISTRY_PATH = STATE_DIR / "atlas_fault_registry.json"
SUMMARY_PATH = STATE_DIR / "atlas_fault_registry_summary.json"

KEYWORDS = (
    "health",
    "watchdog",
    "healing",
    "fault",
    "diagnostic",
    "bitacora",
    "supervisor",
    "autonomy",
    "recovery",
    "sentinel",
    "monitor",
    "rollback",
    "safe",
    "failsafe",
)

DOMAIN_HINTS = {
    "atlas_adapter": "gateway_core",
    "atlas_workspace": "workspace_core",
    "atlas_code_quant": "quant_core",
    "autonomous": "autonomy_core",
    "nexus": "cognitive_core",
    "atlas_nexus": "cognitive_core",
    "brain": "cognitive_core",
    "memory": "memory_core",
    "memory_engine": "memory_core",
    "modules": "shared_modules",
    "software_hub": "operations_core",
    "scripts": "operations_core",
    "ros2_ws": "robotics_core",
    "simulation": "simulation_core",
    "grafana": "integration_core",
    "cloudflare": "integration_core",
    "homeassistant_config": "integration_core",
    "bridge": "integration_core",
    "runtime": "runtime_core",
    "state": "runtime_core",
    "shared": "shared_core",
    "tools": "tooling_core",
    "training": "training_core",
}


@dataclass
class DomainEntry:
    domain_key: str
    role: str
    path: str
    file_count: int
    subdir_count: int
    signal_files: List[str]


def _iter_top_level_dirs() -> Iterable[Path]:
    for path in sorted(REPO_ROOT.iterdir(), key=lambda p: p.name.lower()):
        if path.is_dir():
            yield path


def _infer_role(name: str) -> str:
    for hint, role in DOMAIN_HINTS.items():
        if name.lower() == hint.lower():
            return role
    if name.startswith("."):
        return "hidden_support"
    if "venv" in name.lower():
        return "environment"
    if name.lower() in {"logs", "reports", "snapshots", "backups", "tmp", "temp_models_cache"}:
        return "artifacts"
    return "unclassified"


def _repo_files() -> List[str]:
    proc = __import__("subprocess").run(
        ["rg", "--files"],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError((proc.stderr or proc.stdout or "rg --files failed").strip())
    return [line.strip().replace("\\", "/") for line in proc.stdout.splitlines() if line.strip()]


def _scan_domain(path: Path, repo_files: List[str]) -> DomainEntry:
    rel_root = path.resolve().relative_to(REPO_ROOT.resolve())
    prefix = str(rel_root).replace("\\", "/")
    domain_files = [f for f in repo_files if f == prefix or f.startswith(prefix + "/")]
    subdirs = {
        remainder.split("/", 1)[0]
        for f in domain_files
        for remainder in [f[len(prefix) + 1 :]]
        if f.startswith(prefix + "/") and remainder and "/" in remainder
    }
    signal_files = [
        f for f in domain_files if any(keyword in f.lower() for keyword in KEYWORDS)
    ]
    signal_files = sorted(signal_files)[:200]
    return DomainEntry(
        domain_key=path.name,
        role=_infer_role(path.name),
        path=str(path.resolve()),
        file_count=len(domain_files),
        subdir_count=len(subdirs),
        signal_files=signal_files,
    )


def _build_registry() -> Dict[str, object]:
    repo_files = _repo_files()
    domains: List[DomainEntry] = []
    role_counter: Counter[str] = Counter()
    total_files = 0
    total_signal_files = 0
    for top in _iter_top_level_dirs():
        domain = _scan_domain(top, repo_files)
        domains.append(domain)
        role_counter[domain.role] += 1
        total_files += domain.file_count
        total_signal_files += len(domain.signal_files)

    by_role: Dict[str, List[str]] = defaultdict(list)
    for domain in domains:
        by_role[domain.role].append(domain.domain_key)

    return {
        "generated_at": __import__("datetime").datetime.utcnow().isoformat() + "Z",
        "repo_root": str(REPO_ROOT.resolve()),
        "domain_count": len(domains),
        "total_files": total_files,
        "total_signal_files": total_signal_files,
        "roles": dict(role_counter),
        "domains_by_role": {k: sorted(v) for k, v in by_role.items()},
        "domains": [asdict(domain) for domain in domains],
        "fault_manager_seed": {
            "detection": [
                "autonomous.health_monitor",
                "modules.humanoid.ans.checks",
                "scripts.atlas_autodiagnostic",
                "atlas_adapter.supervisor_daemon",
            ],
            "recovery": [
                "autonomous.self_healing",
                "modules.humanoid.healing",
                "modules.humanoid.supervisor.rollback_manager",
            ],
            "observability": [
                "modules.humanoid.ans.evolution_bitacora",
                "modules.humanoid.notify",
                "atlas_adapter.static.v4.modules.health",
                "atlas_adapter.static.v4.modules.healing",
            ],
        },
    }


def _write_json(path: Path, payload: Dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def _emit_bitacora(message: str, ok: bool) -> None:
    append_evolution_log(message=message, ok=ok, source="fault_registry")


async def _emit_telegram(message: str) -> bool:
    return await send_telegram(message)


def _notify_warning(message: str) -> None:
    try:
        asyncio.run(_emit_telegram(message))
    except Exception:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description="Build ATLAS fault registry")
    parser.add_argument("--json", action="store_true", help="Print JSON summary")
    parser.add_argument("--no-emit", action="store_true", help="Skip bitacora/telegram")
    args = parser.parse_args()

    registry = _build_registry()
    summary = {
        "generated_at": registry["generated_at"],
        "repo_root": registry["repo_root"],
        "domain_count": registry["domain_count"],
        "total_files": registry["total_files"],
        "total_signal_files": registry["total_signal_files"],
        "roles": registry["roles"],
    }
    _write_json(REGISTRY_PATH, registry)
    _write_json(SUMMARY_PATH, summary)

    if not args.no_emit:
        _emit_bitacora(
            (
                f"[FAULT_REGISTRY] Registro generado: {summary['domain_count']} dominios, "
                f"{summary['total_files']} archivos, {summary['total_signal_files']} señales de fault/health."
            ),
            ok=True,
        )
        if int(summary["domain_count"]) < 10:
            msg = (
                "ATLAS Fault Registry detectó una cobertura inesperadamente baja. "
                f"Dominios detectados: {summary['domain_count']}."
            )
            _emit_bitacora(f"[FAULT_REGISTRY] {msg}", ok=False)
            _notify_warning(f"ATLAS FAULT REGISTRY WARNING\n{msg}")

    if args.json:
        print(json.dumps(summary, indent=2, ensure_ascii=False))
    else:
        print("ATLAS Fault Registry generated")
        print(json.dumps(summary, indent=2, ensure_ascii=False))
        print(f"Registry: {REGISTRY_PATH}")
        print(f"Summary : {SUMMARY_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
