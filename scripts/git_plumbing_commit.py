from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


def _repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _run(repo: Path, *args: str, timeout_s: int = 20) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", *args],
        cwd=str(repo),
        capture_output=True,
        text=True,
        timeout=timeout_s,
        env={**os.environ, "LANG": "C"},
    )


def _must(repo: Path, *args: str, timeout_s: int = 20) -> str:
    r = _run(repo, *args, timeout_s=timeout_s)
    if r.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)} failed: {(r.stderr or r.stdout or '').strip()}")
    return (r.stdout or "").strip()


def main() -> int:
    ap = argparse.ArgumentParser(description="Crear commit usando plumbing (evita wrappers de PowerShell).")
    ap.add_argument("-m", "--message", required=True, help="Mensaje del commit (una sola linea).")
    args = ap.parse_args()

    repo = _repo_root()
    if not (repo / ".git").exists():
        print("No es repo git.", file=sys.stderr)
        return 2

    # Verifica que haya cambios staged
    staged = _must(repo, "diff", "--cached", "--name-only")
    if not staged.strip():
        print("Nada staged para commitear.")
        return 0

    # Crea commit desde el index
    tree = _must(repo, "write-tree")
    parent = _must(repo, "rev-parse", "HEAD")
    ref = _must(repo, "symbolic-ref", "-q", "HEAD")
    commit = _must(repo, "commit-tree", tree, "-p", parent, "-m", str(args.message))

    # Avanza el ref y limpia el index como lo haría `git commit`
    _must(repo, "update-ref", ref, commit, parent)
    _must(repo, "reset")

    print(commit.strip())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

