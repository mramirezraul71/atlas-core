from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

from .inventory import scan_inventory

LOG = logging.getLogger("atlas_py")


def _setup_logging() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s - %(message)s")


def main(argv: list[str] | None = None) -> int:
    """PY003: CLI minimal para inventory.

    Nota: los tests de PY003 hoy validan `scan_inventory` (core), no el CLI.
    """
    _setup_logging()
    p = argparse.ArgumentParser(prog="atlas_py", description="ATLAS Python Mastery tools")
    sub = p.add_subparsers(dest="cmd", required=True)

    inv = sub.add_parser("inventory", help="Inventario de archivos")
    inv.add_argument("--root", default=".", help="Directorio raíz")
    inv.add_argument("--min-size-kb", type=int, default=0, help="Filtrar por tamaño mínimo")
    inv.add_argument("--json", action="store_true", help="Salida JSON")

    ns = p.parse_args(sys.argv[1:] if argv is None else argv)
    if ns.cmd == "inventory":
        res = scan_inventory(Path(ns.root), min_size_kb=int(ns.min_size_kb))
        payload = {
            "total_files": res.total_files,
            "total_size_bytes": res.total_size_bytes,
            "top_extensions": res.top_extensions,
        }
        if ns.json:
            print(json.dumps(payload, ensure_ascii=False))
        else:
            print(payload)
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())

