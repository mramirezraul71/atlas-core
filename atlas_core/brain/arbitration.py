"""Resolución simple de conflictos entre comandos."""
from __future__ import annotations

import logging
from .models import Command, SystemSnapshot

logger = logging.getLogger("atlas.brain.arbitration")


class ArbitrationEngine:
    """Prioriza safety; evita contradicciones obvias en el mismo target."""

    def resolve(self, commands: list[Command], _snapshot: SystemSnapshot) -> list[Command]:
        if not commands:
            return []
        by_target: dict[str, list[Command]] = {}
        for c in commands:
            by_target.setdefault(c.target, []).append(c)

        out: list[Command] = []
        for target, group in by_target.items():
            if len(group) == 1:
                out.append(group[0])
                continue
            # Si alguno pone safe, gana
            safe_cmd = next(
                (c for c in group if c.action == "set_mode" and str(c.params.get("mode", "")).lower() == "safe"),
                None,
            )
            if safe_cmd:
                out.append(safe_cmd)
                logger.debug("arbitration: safe wins for target=%s", target)
                continue
            # Eliminar duplicados idénticos
            seen: set[tuple[str, str, str]] = set()
            for c in group:
                key = (c.target, c.action, str(sorted(c.params.items())))
                if key in seen:
                    continue
                seen.add(key)
                out.append(c)
        return out
