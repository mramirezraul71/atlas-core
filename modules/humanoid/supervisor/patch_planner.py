"""Planificador mínimo de propuestas de patch para supervisor."""
from __future__ import annotations

import uuid
from typing import Any, Dict, List, Optional

from .models import ChangeProposal, PatchBlock


def _normalize_source(payload: Dict[str, Any]) -> str:
    return str(payload.get("source") or "unknown")


class PatchPlanner:
    """Convierte señales/eventos en propuestas estructuradas (si hay datos suficientes)."""

    def build_review(
        self,
        topic: str,
        payload: Dict[str, Any],
        recommended_action: str,
    ) -> Dict[str, Any]:
        proposal = self.proposal_from_event(topic, payload, recommended_action)
        return {
            "review_id": f"spr_{uuid.uuid4().hex[:10]}",
            "topic": topic,
            "source": _normalize_source(payload),
            "recommended_action": recommended_action,
            "target_file": str(payload.get("target_file") or payload.get("path") or ""),
            "proposal_ready": proposal is not None,
            "patch_count": len(proposal.patches) if proposal else 0,
        }

    def proposal_from_event(
        self,
        topic: str,
        payload: Dict[str, Any],
        recommended_action: str,
    ) -> Optional[ChangeProposal]:
        target_file = str(payload.get("target_file") or payload.get("path") or "").strip()
        patch_blocks: List[PatchBlock] = []

        payload_patches = payload.get("patches")
        if isinstance(payload_patches, list):
            for item in payload_patches:
                if not isinstance(item, dict):
                    continue
                old_text = str(item.get("old_text") or "")
                new_text = str(item.get("new_text") or "")
                if not old_text and not new_text:
                    continue
                patch_blocks.append(
                    PatchBlock(
                        old_text=old_text,
                        new_text=new_text,
                        replace_all=bool(item.get("replace_all", False)),
                    )
                )
        else:
            old_text = str(payload.get("old_text") or "")
            new_text = str(payload.get("new_text") or "")
            if old_text or new_text:
                patch_blocks.append(
                    PatchBlock(
                        old_text=old_text,
                        new_text=new_text,
                        replace_all=bool(payload.get("replace_all", False)),
                    )
                )

        if not target_file or not patch_blocks:
            return None

        raw_test_paths = payload.get("test_paths")
        test_paths = [str(p) for p in raw_test_paths] if isinstance(raw_test_paths, list) else []

        return ChangeProposal(
            topic=topic,
            source=_normalize_source(payload),
            action=recommended_action,
            target_file=target_file,
            patches=patch_blocks,
            allow_new_file=bool(payload.get("allow_new_file", False)),
            metadata={
                "rule": payload.get("rule"),
                "component": payload.get("component"),
                "change_type": payload.get("change_type"),
            },
            test_paths=test_paths,
            proposal_id=f"prop_{uuid.uuid4().hex[:10]}",
        )

    def proposal_from_dict(self, raw: Dict[str, Any]) -> Optional[ChangeProposal]:
        if not isinstance(raw, dict):
            return None
        return self.proposal_from_event(
            topic=str(raw.get("topic") or "manual.proposal"),
            payload=raw,
            recommended_action=str(raw.get("action") or "apply_patch"),
        )

