"""Comprueba que los archivos listados en docs/FASE3_IMPLEMENTACION_ESTADO.csv existan en el árbol atlas_code_quant."""
from __future__ import annotations

import csv
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "docs" / "FASE3_IMPLEMENTACION_ESTADO.csv"


def _paths_from_archivo_cell(cell: str) -> list[Path]:
    cell = cell.strip()
    if not cell or "*" in cell or " + " in cell:
        return []
    first = cell.split(",")[0].strip()
    if not first.endswith(".py"):
        return []
    return [ROOT / first]


def main() -> int:
    if not MANIFEST.is_file():
        print(f"Falta manifest: {MANIFEST}", file=sys.stderr)
        return 2
    missing: list[str] = []
    with MANIFEST.open(encoding="utf-8-sig", newline="") as f:
        for row in csv.DictReader(f):
            archivo = row.get("Archivo", "")
            for p in _paths_from_archivo_cell(archivo):
                if not p.is_file():
                    missing.append(str(p.relative_to(ROOT)))
    # Tests unitarios Fase 3 (wildcard en CSV)
    for rel in (
        "tests/test_learning/test_cnn_lstm.py",
        "tests/test_forecasting/test_prophet.py",
        "tests/test_market_context/test_onchain.py",
        "tests/test_market_context/test_fundamental.py",
    ):
        p = ROOT / rel
        if not p.is_file():
            missing.append(rel)
    if missing:
        print("Archivos Fase 3 no encontrados:", file=sys.stderr)
        for m in missing:
            print(f"  - {m}", file=sys.stderr)
        return 1
    # Mensaje alineado con guia de actualizacion del repo (ASCII para consolas Windows)
    print("[OK] FASE3 manifest OK: todos los artefactos core presentes.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
