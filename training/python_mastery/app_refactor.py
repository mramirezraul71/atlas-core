from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Config:
    root: Path
    dry_run: bool
    min_size_kb: int


def parse_args(argv: list[str]) -> Config:
    """PY007: parseo determinista (sin side-effects)."""
    p = argparse.ArgumentParser(prog="atlas_py_refactor", add_help=False)
    p.add_argument("--root", default=".")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--min-size-kb", type=int, default=0)
    ns = p.parse_args(argv)
    return Config(root=Path(ns.root), dry_run=bool(ns.dry_run), min_size_kb=int(ns.min_size_kb))


def build_inventory_payload(total_files: int, total_size_bytes: int, top_extensions: list[tuple[str, int]]) -> dict:
    """PY007: lógica pura, fácil de testear."""
    return {
        "total_files": int(total_files),
        "total_size_bytes": int(total_size_bytes),
        "top_extensions": [(str(ext), int(count)) for ext, count in top_extensions],
    }

