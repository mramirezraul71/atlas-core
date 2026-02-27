from __future__ import annotations

from pathlib import Path
from typing import Dict

from ..models import POT, POTSeverity, POTStep


def get_pot() -> POT:
    def _run(ctx: Dict) -> str:
        root = Path(__file__).resolve().parents[3]  # .../modules/quality/pots -> repo root
        tutorias = root / "modules" / "humanoid" / "quality" / "tutorias"
        visitas = tutorias / "visitas"
        index = tutorias / "index.md"
        opus = visitas / "2026-02-15__claude_opus_4_5__tutoria_tecnica_atlas.md"

        lines = ["QUALITY VISITS AUDIT (check-only)"]
        lines.append(f"- tutorias_dir_exists={tutorias.exists()}")
        lines.append(f"- visitas_dir_exists={visitas.exists()}")
        lines.append(f"- index_exists={index.exists()}")
        lines.append(f"- opus_4_5_visit_exists={opus.exists()}")

        if tutorias.exists():
            md = sorted([p.name for p in visitas.glob("*.md")] if visitas.exists() else [])
            lines.append(f"- visitas_md_count={len(md)}")
            for n in md[:20]:
                lines.append(f"  - {n}")
            if len(md) > 20:
                lines.append(f"  ... +{len(md) - 20} más")

        lines.append("")
        lines.append("Regla de especialista:")
        lines.append("- Si una visita/tutoría ocurre, debe quedar aquí como MD + index actualizado.")
        return "\n".join(lines)

    return POT(
        id="quality_visits_audit",
        name="Quality Visits Audit (Tutorías/Visitas)",
        description="Verifica que la carpeta de visitas tenga índice y evidencias (incluye Opus 4.5).",
        severity=POTSeverity.LOW,
        rules=[
            "Check-only: no crea ni modifica visitas automáticamente (evita falsos positivos).",
            "Si falta evidencia, la corrección es crear un nuevo MD de visita y actualizar index.md.",
        ],
        tags=["quality", "tutorias", "visitas", "audit"],
        steps=[POTStep(id="audit_visits", name="Auditar carpeta tutorías/visitas", run=_run, fatal=True)],
    )

