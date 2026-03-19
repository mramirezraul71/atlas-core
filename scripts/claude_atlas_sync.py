#!/usr/bin/env python3
"""
claude_atlas_sync.py — CLI de sincronización Claude-Atlas Memory Bridge

Uso:
    python scripts/claude_atlas_sync.py sync       # Bidireccional (default)
    python scripts/claude_atlas_sync.py push       # Claude → Atlas
    python scripts/claude_atlas_sync.py pull       # Atlas → Claude
    python scripts/claude_atlas_sync.py stats      # Ver estado del puente
    python scripts/claude_atlas_sync.py insight "texto" [--category cat] [--importance 0.8]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# Añadir raíz del proyecto al path
_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_ROOT))

from brain.claude_memory_bridge import ClaudeMemoryBridge


def _print_json(data: dict) -> None:
    print(json.dumps(data, ensure_ascii=False, indent=2))


def cmd_sync(args: argparse.Namespace) -> None:
    print("[claude_atlas_sync] Sincronización BIDIRECCIONAL...")
    bridge = ClaudeMemoryBridge(verbose=True)
    result = bridge.bidirectional_sync()
    _print_json(result)
    if result["ok"]:
        c2a = result["claude_to_atlas"]
        a2c = result["atlas_to_claude"]
        print(f"\n[OK] Claude->Atlas: {c2a['episodes_written']} episodios escritos, "
              f"{len(c2a['patterns_updated'])} patrones actualizados")
        print(f"[OK] Atlas->Claude: {a2c['episodes_exported']} episodios exportados -> {a2c['output_file']}")
    else:
        print("[ERROR] Error durante la sincronizacion", file=sys.stderr)
        sys.exit(1)


def cmd_push(args: argparse.Namespace) -> None:
    print("[claude_atlas_sync] Claude → Atlas brain...")
    bridge = ClaudeMemoryBridge(verbose=True)
    result = bridge.sync_claude_to_atlas()
    _print_json(result)
    if result["ok"]:
        print(f"\n[OK] {result['episodes_written']} episodios escritos al brain de Atlas")
        print(f"   Secciones: {', '.join(result['sections_processed'])}")
    else:
        print(f"[ERROR] {result.get('error', 'Error desconocido')}", file=sys.stderr)
        sys.exit(1)


def cmd_pull(args: argparse.Namespace) -> None:
    print("[claude_atlas_sync] Atlas brain → Claude context...")
    bridge = ClaudeMemoryBridge(verbose=True)
    result = bridge.sync_atlas_to_claude()
    _print_json(result)
    if result["ok"]:
        print(f"\n[OK] {result['episodes_exported']} episodios exportados")
        print(f"   Archivo: {result['output_file']}")
    else:
        print("[ERROR] Error durante la exportacion", file=sys.stderr)
        sys.exit(1)


def cmd_stats(args: argparse.Namespace) -> None:
    print("[claude_atlas_sync] Estado del puente:")
    bridge = ClaudeMemoryBridge()
    stats = bridge.get_bridge_stats()
    _print_json(stats)


def cmd_insight(args: argparse.Namespace) -> None:
    if not args.text:
        print("✘  Debes proporcionar el texto del insight", file=sys.stderr)
        sys.exit(1)
    bridge = ClaudeMemoryBridge()
    result = bridge.write_insight(
        insight=args.text,
        category=args.category,
        importance=args.importance,
        tags=args.tags or [],
    )
    _print_json(result)
    print(f"\n[OK] Insight registrado (episode_id={result['episode_id']})")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Claude-Atlas Memory Bridge — Sincronización bidireccional",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="command")

    sub.add_parser("sync",  help="Sincronización bidireccional (default)")
    sub.add_parser("push",  help="Claude MEMORY.md → Atlas brain")
    sub.add_parser("pull",  help="Atlas brain → atlas_brain_context.json para Claude")
    sub.add_parser("stats", help="Ver estado del puente")

    p_insight = sub.add_parser("insight", help="Escribir insight puntual al brain")
    p_insight.add_argument("text",       nargs="?", default="",  help="Texto del insight")
    p_insight.add_argument("--category", default="general",      help="Categoría (default: general)")
    p_insight.add_argument("--importance", type=float, default=0.8, help="Importancia 0.0-1.0")
    p_insight.add_argument("--tags",     nargs="*", default=[],   help="Tags adicionales")

    args = parser.parse_args()

    dispatch = {
        "sync":    cmd_sync,
        "push":    cmd_push,
        "pull":    cmd_pull,
        "stats":   cmd_stats,
        "insight": cmd_insight,
        None:      cmd_sync,  # default
    }
    fn = dispatch.get(args.command, cmd_sync)
    fn(args)


if __name__ == "__main__":
    main()
