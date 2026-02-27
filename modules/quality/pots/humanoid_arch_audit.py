from __future__ import annotations

from pathlib import Path
from typing import Dict, List

from ..models import POT, POTSeverity, POTStep


def _dir_exists(p: Path) -> bool:
    try:
        return p.exists() and p.is_dir()
    except Exception:
        return False


def get_pot() -> POT:
    def _run(ctx: Dict) -> str:
        root = Path(__file__).resolve().parents[3]
        humanoid = root / "modules" / "humanoid"

        # Mapa mínimo de subsistemas que deberían existir si el cerebro “completo” vive aquí.
        expected = [
            "ans",
            "quality",
            "comms",
            "scheduler",
            "vision",
            "voice",
            "memory_engine",
            "governance",
            "policy",
            "nexus",
        ]

        lines: List[str] = ["HUMANOID ARCH AUDIT (check-only)"]
        lines.append(f"- humanoid_dir_exists={_dir_exists(humanoid)}")

        if not _dir_exists(humanoid):
            lines.append("")
            lines.append("Hallazgo: este repo no contiene el cerebro humanoid completo en disco.")
            lines.append("Acción: definir repositorio fuente (atlas-core) o restaurar el árbol modules/humanoid.")
            return "\n".join(lines)

        missing = [n for n in expected if not _dir_exists(humanoid / n)]
        present = [n for n in expected if _dir_exists(humanoid / n)]

        lines.append(f"- expected_subsystems={len(expected)} present={len(present)} missing={len(missing)}")
        if present:
            lines.append("- present:")
            for n in present:
                lines.append(f"  - {n}")
        if missing:
            lines.append("- missing:")
            for n in missing:
                lines.append(f"  - {n}")

        lines.append("")
        lines.append("Guía de especialista:")
        lines.append("- Si falta un subsistema, NO ejecutar POTs que lo dependan.")
        lines.append("- Mantener POTs check-only hasta restaurar el árbol humanoid completo.")
        return "\n".join(lines)

    return POT(
        id="humanoid_arch_audit",
        name="Humanoid Architecture Audit (modules/humanoid)",
        description="Audita presencia mínima de subsistemas humanoid para evitar operar a ciegas.",
        severity=POTSeverity.HIGH,
        rules=[
            "Check-only: no intenta crear módulos faltantes automáticamente.",
            "Si faltan subsistemas, se considera estado 'parcial' y se opera en modo seguro.",
        ],
        tags=["humanoid", "architecture", "audit"],
        steps=[POTStep(id="audit_humanoid", name="Auditar árbol humanoid", run=_run, fatal=True)],
    )

