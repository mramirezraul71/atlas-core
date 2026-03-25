"""
ATLAS DOCTOR CLI — Interfaz de línea de comandos
================================================
Uso:
    python -m atlas_adapter.services.doctor_cli status
    python -m atlas_adapter.services.doctor_cli diagnose
    python -m atlas_adapter.services.doctor_cli retrospect [--limit N]
    python -m atlas_adapter.services.doctor_cli emergency
    python -m atlas_adapter.services.doctor_cli ports
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent.parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

# ── Helpers de presentación ────────────────────────────────────────────────────
TIER_ICONS = {0: "🔴", 1: "🟠", 2: "🟡", 3: "🔵", 4: "🟣"}
TIER_LABELS = {0: "CRASH", 1: "CRÍTICO", 2: "DEGRADADO", 3: "WARNING", 4: "EVOLUCIÓN"}


def _print_header(title: str) -> None:
    w = 64
    print("\n" + "═" * w)
    print(f"  {title}")
    print("═" * w)


def _print_status(data: dict) -> None:
    _print_header("ATLAS DOCTOR — Estado del Sistema Nervioso")
    ok = data.get("ok", False)
    print(f"  Estado global  : {'✓ OK' if ok else '✗ ANOMALÍAS DETECTADAS'}")
    print(f"  Ciclo          : #{data.get('cycle', '--')}")
    print(f"  Último ciclo   : {data.get('last_ts', '--')}")
    print(f"  Anomalías      : {data.get('anomalies_count', 0)}")
    print(f"  Sanadas        : {data.get('healed_count', 0)}")

    tiers = data.get("tiers", {})
    if tiers:
        print(f"\n  Por tier:")
        for label, count in tiers.items():
            tier_num = next((k for k, v in TIER_LABELS.items() if v == label), 3)
            icon = TIER_ICONS.get(tier_num, "⚪")
            print(f"    {icon} {label}: {count}")

    anomalies = data.get("anomalies", [])
    if anomalies:
        print(f"\n  Anomalías activas:")
        for a in anomalies:
            tier_num = a.get("tier", 3)
            icon = TIER_ICONS.get(tier_num, "⚪")
            label = TIER_LABELS.get(tier_num, a.get("tier"))
            print(f"    {icon} [{label}] {a['component']} ({a['layer']})")
            print(f"       {a['description']}")

    actions = data.get("actions", [])
    if actions:
        print(f"\n  Acciones tomadas:")
        for act in actions:
            healed = act.get("healed", False)
            mark = "✓" if healed else "→"
            print(f"    {mark} {act['component']}: {act['type']} — {act.get('outcome') or act.get('detail', '')}")


def _print_ports(data: dict) -> None:
    _print_header("ATLAS DOCTOR — Estado de Puertos (15)")
    by_layer: dict = {}
    for name, p in data.items():
        layer = p.get("layer", "other")
        by_layer.setdefault(layer, []).append((name, p))

    for layer, items in sorted(by_layer.items()):
        print(f"\n  [{layer.upper()}]")
        for name, p in items:
            up = p.get("up", False)
            mark = "UP  " if up else "DOWN"
            icon = "✓" if up else "✗"
            print(f"    [{mark}] {icon} :{p['port']:<5}  {name:<22}  {p.get('description', '')}")


def _print_history(items: list) -> None:
    _print_header(f"ATLAS DOCTOR — Historial de Eventos ({len(items)})")
    if not items:
        print("  Sin eventos registrados.")
        return
    for e in items[:30]:
        healed = e.get("healed") in (1, True)
        tier = e.get("tier", 3)
        icon = TIER_ICONS.get(tier, "⚪")
        mark = "✓ SANADO" if healed else e.get("action_type", "")
        print(f"  {icon} {e.get('ts', '')[:19]}  {e.get('component', ''):<22}  "
              f"[{e.get('tier_label', '')}]  {mark}")
        if e.get("outcome"):
            print(f"       → {e['outcome'][:80]}")


# ── Subcomandos ────────────────────────────────────────────────────────────────
def cmd_status(args) -> None:
    from atlas_adapter.services.doctor_nervous_system import get_doctor
    doctor = get_doctor()
    data = doctor.status_report()
    if args.json:
        print(json.dumps(data, indent=2, ensure_ascii=False))
    else:
        _print_status(data)


def cmd_diagnose(args) -> None:
    from atlas_adapter.services.doctor_nervous_system import get_doctor
    doctor = get_doctor()

    async def _run():
        return await doctor.run_once()

    data = asyncio.run(_run())
    if args.json:
        print(json.dumps(data, indent=2, ensure_ascii=False))
    else:
        print("\n⚡ Diagnóstico completo ejecutado:")
        _print_status(data)


def cmd_retrospect(args) -> None:
    from atlas_adapter.services.doctor_nervous_system import get_doctor
    doctor = get_doctor()
    items = doctor.history(args.limit)
    if args.json:
        print(json.dumps(items, indent=2, ensure_ascii=False))
    else:
        _print_history(items)


def cmd_emergency(args) -> None:
    _print_header("🚨 ATLAS DOCTOR — EMERGENCY STOP")
    if not args.force:
        resp = input("  ¿Confirmar emergency stop? [s/N]: ").strip().lower()
        if resp not in ("s", "si", "sí", "y", "yes"):
            print("  Cancelado.")
            return
    from atlas_adapter.services.doctor_nervous_system import get_doctor
    doctor = get_doctor()
    result = doctor.emergency_stop()
    print("\n  Resultado:")
    for k, v in result.items():
        print(f"    {k}: {v}")


def cmd_ports(args) -> None:
    import socket
    from modules.humanoid.healing.atlas_doctor_daemon import MONITORED_PORTS

    def _tcp(port: int) -> bool:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.8):
                return True
        except Exception:
            return False

    data = {}
    for port, (name, layer, desc) in MONITORED_PORTS.items():
        data[name] = {"port": port, "layer": layer, "description": desc, "up": _tcp(port)}

    if args.json:
        print(json.dumps(data, indent=2))
    else:
        _print_ports(data)


# ── Main ───────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        prog="python -m atlas_adapter.services.doctor_cli",
        description="ATLAS DOCTOR CLI — Sistema Nervioso Central"
    )
    parser.add_argument("--json", action="store_true", help="Salida en JSON")
    parser.add_argument("--dry-run", action="store_true", help="No ejecutar reparaciones")

    sub = parser.add_subparsers(dest="command")

    sub.add_parser("status",     help="Estado actual del doctor")
    sub.add_parser("diagnose",   help="Ciclo de diagnóstico completo ahora")

    retro = sub.add_parser("retrospect", help="Análisis histórico de eventos")
    retro.add_argument("--limit", type=int, default=50)

    em = sub.add_parser("emergency", help="Emergency stop — detiene subsistemas críticos")
    em.add_argument("--force", action="store_true", help="Sin confirmación interactiva")

    sub.add_parser("ports", help="Estado instantáneo de todos los puertos")

    args = parser.parse_args()

    if args.dry_run:
        os.environ["ATLAS_DOCTOR_DRY_RUN"] = "true"

    dispatch = {
        "status":     cmd_status,
        "diagnose":   cmd_diagnose,
        "retrospect": cmd_retrospect,
        "emergency":  cmd_emergency,
        "ports":      cmd_ports,
    }

    fn = dispatch.get(args.command)
    if fn is None:
        parser.print_help()
        sys.exit(0)

    fn(args)


if __name__ == "__main__":
    main()
