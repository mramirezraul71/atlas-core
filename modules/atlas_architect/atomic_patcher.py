from __future__ import annotations

import difflib
import re
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple


class AtomicPatchError(RuntimeError):
    pass


def _udiff(before: str, after: str, file_path: str) -> str:
    return "\n".join(
        difflib.unified_diff(
            before.splitlines(),
            after.splitlines(),
            fromfile=file_path + " (before)",
            tofile=file_path + " (after)",
            lineterm="",
        )
    )


@dataclass(frozen=True)
class ReplaceBlock:
    """Reemplaza el bloque entre anclas (incluyéndolas o excluyéndolas)."""

    anchor_start: str
    anchor_end: str
    new_block: str
    include_anchors: bool = True
    require_unique: bool = True


@dataclass(frozen=True)
class RegexReplace:
    pattern: str
    repl: str
    count: int = 1
    flags: int = 0


@dataclass(frozen=True)
class InsertAfter:
    anchor: str
    text: str
    require_unique: bool = True


PatchOp = ReplaceBlock | RegexReplace | InsertAfter


class AtomicPatcher:
    """Aplica parches atómicos en texto (sin sobreescritura ciega).

    Estrategia:
    - Carga archivo.
    - Aplica operaciones deterministas (anclas/regex) con verificaciones de unicidad.
    - Devuelve (after, diff) sin escribir.
    """

    def apply(self, before: str, ops: List[PatchOp], *, file_path: str = "") -> Tuple[str, str]:
        after = before
        for op in ops:
            if isinstance(op, ReplaceBlock):
                after = self._replace_block(after, op)
            elif isinstance(op, RegexReplace):
                after = self._regex_replace(after, op)
            elif isinstance(op, InsertAfter):
                after = self._insert_after(after, op)
            else:
                raise AtomicPatchError(f"Operación no soportada: {type(op)}")
        diff = _udiff(before, after, file_path or "file")
        return after, diff

    def patch_file_preview(self, path: Path, ops: List[PatchOp]) -> Tuple[str, str, str]:
        p = Path(path)
        before = p.read_text(encoding="utf-8", errors="ignore") if p.exists() else ""
        after, diff = self.apply(before, ops, file_path=str(p).replace("\\", "/"))
        return before, after, diff

    def _replace_block(self, text: str, op: ReplaceBlock) -> str:
        a = op.anchor_start
        b = op.anchor_end
        if not a or not b:
            raise AtomicPatchError("anchor_start/anchor_end requeridos")

        start_idxs = [m.start() for m in re.finditer(re.escape(a), text)]
        end_idxs = [m.start() for m in re.finditer(re.escape(b), text)]
        if not start_idxs or not end_idxs:
            raise AtomicPatchError("No se encontraron anclas en el archivo")
        if op.require_unique and (len(start_idxs) != 1 or len(end_idxs) != 1):
            raise AtomicPatchError("Anclas no únicas (require_unique=True)")

        s = start_idxs[0]
        e = end_idxs[0]
        if e < s:
            raise AtomicPatchError("anchor_end aparece antes de anchor_start")

        if op.include_anchors:
            # reemplaza desde inicio de anchor_start hasta fin de anchor_end
            e_end = e + len(b)
            return text[:s] + op.new_block + text[e_end:]

        # excluye anclas: conserva anclas y reemplaza lo intermedio
        s_end = s + len(a)
        e_start = e
        return text[:s_end] + op.new_block + text[e_start:]

    def _regex_replace(self, text: str, op: RegexReplace) -> str:
        try:
            rx = re.compile(op.pattern, op.flags)
        except re.error as e:
            raise AtomicPatchError(f"Regex inválida: {e}")
        new, n = rx.subn(op.repl, text, count=int(op.count or 0))
        if n == 0:
            raise AtomicPatchError("RegexReplace no aplicó cambios (0 matches)")
        return new

    def _insert_after(self, text: str, op: InsertAfter) -> str:
        if not op.anchor:
            raise AtomicPatchError("anchor requerido")
        idxs = [m.start() for m in re.finditer(re.escape(op.anchor), text)]
        if not idxs:
            raise AtomicPatchError("Anchor no encontrado para InsertAfter")
        if op.require_unique and len(idxs) != 1:
            raise AtomicPatchError("Anchor no único (require_unique=True)")
        i = idxs[0] + len(op.anchor)
        return text[:i] + op.text + text[i:]

