from __future__ import annotations

import argparse
import sys


def main(argv: list[str] | None = None) -> int:
    """PY012: entrypoint de ejemplo (console_script)."""
    p = argparse.ArgumentParser(prog="atlas-py-tool")
    p.add_argument("--ping", action="store_true")
    ns = p.parse_args(sys.argv[1:] if argv is None else argv)
    if ns.ping:
        print("pong")
        return 0
    print("atlas-py-tool")
    return 0

