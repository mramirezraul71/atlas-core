from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

from .atomic_patcher import InsertAfter, RegexReplace, ReplaceBlock


def serialize_patch_ops(ops: List[Any]) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for op in ops or []:
        if isinstance(op, ReplaceBlock):
            out.append({"type": "replace_block", **op.__dict__})
        elif isinstance(op, RegexReplace):
            out.append({"type": "regex_replace", **op.__dict__})
        elif isinstance(op, InsertAfter):
            out.append({"type": "insert_after", **op.__dict__})
        elif isinstance(op, dict) and "type" in op:
            out.append(op)
        else:
            raise ValueError(f"PatchOp no serializable: {type(op)}")
    return out


def deserialize_patch_ops(data: List[Dict[str, Any]]) -> List[Any]:
    ops: List[Any] = []
    for d in data or []:
        t = (d.get("type") or "").strip()
        if t == "replace_block":
            ops.append(
                ReplaceBlock(
                    anchor_start=d.get("anchor_start") or "",
                    anchor_end=d.get("anchor_end") or "",
                    new_block=d.get("new_block") or "",
                    include_anchors=bool(d.get("include_anchors", True)),
                    require_unique=bool(d.get("require_unique", True)),
                )
            )
        elif t == "regex_replace":
            ops.append(
                RegexReplace(
                    pattern=d.get("pattern") or "",
                    repl=d.get("repl") or "",
                    count=int(d.get("count") or 1),
                    flags=int(d.get("flags") or 0),
                )
            )
        elif t == "insert_after":
            ops.append(
                InsertAfter(
                    anchor=d.get("anchor") or "",
                    text=d.get("text") or "",
                    require_unique=bool(d.get("require_unique", True)),
                )
            )
        else:
            raise ValueError(f"Tipo de PatchOp desconocido: {t}")
    return ops


@dataclass
class PlannedChange:
    kind: str  # write_file | atomic_patch
    path: str
    justification: str
    content: Optional[str] = None
    ops: Optional[List[Dict[str, Any]]] = None  # serialized ops
    diff_preview: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "kind": self.kind,
            "path": self.path,
            "justification": self.justification,
            "content": self.content,
            "ops": self.ops,
            "diff_preview": self.diff_preview,
        }


@dataclass
class ChangePlan:
    goal: str
    mode: str  # governed | growth
    changes: List[PlannedChange]

    def to_payload(self) -> Dict[str, Any]:
        return {"goal": self.goal, "mode": self.mode, "changes": [c.to_dict() for c in self.changes]}

    def to_json(self) -> str:
        return json.dumps(self.to_payload(), ensure_ascii=False, indent=2)


def apply_change_plan(plan: ChangePlan, fs: "FilesystemTools") -> List[Dict[str, Any]]:  # noqa: F821
    """Aplica cambios; devuelve records (file_path, diff_path, justification)."""
    out: List[Dict[str, Any]] = []
    for ch in plan.changes:
        if ch.kind == "write_file":
            rec = fs.write_file(ch.path, ch.content or "", justification=ch.justification)
            out.append({"kind": ch.kind, "file_path": rec.file_path, "diff_path": rec.diff_path})
        elif ch.kind == "atomic_patch":
            ops = deserialize_patch_ops(ch.ops or [])
            rec = fs.atomic_patch(ch.path, ops, justification=ch.justification)
            out.append({"kind": ch.kind, "file_path": rec.file_path, "diff_path": rec.diff_path})
        else:
            out.append({"kind": ch.kind, "file_path": ch.path, "error": "unknown_kind"})
    return out

