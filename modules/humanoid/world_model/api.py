"""API REST para World Model, Autobiographical Memory, Lifelog, Experience Planner y Unified Memory."""
from __future__ import annotations

from fastapi import APIRouter, Query
from pydantic import BaseModel
from typing import Any, Dict, List, Optional

router = APIRouter(prefix="/api/cognitive-memory", tags=["Cognitive Memory"])


# ═══════════════════════════════════════════════════════════
# WORLD MODEL
# ═══════════════════════════════════════════════════════════

@router.get("/world-model/status")
def world_model_status():
    try:
        from .engine import get_world_model
        wm = get_world_model()
        return {"ok": True, **wm.get_state_summary()}
    except Exception as e:
        return {"ok": False, "error": str(e)}


class EntityBody(BaseModel):
    name: str
    entity_type: str
    state: Dict[str, Any] = {}
    properties: Dict[str, Any] = {}
    confidence: float = 1.0


@router.post("/world-model/entity")
def upsert_entity(body: EntityBody):
    try:
        from .engine import get_world_model
        wm = get_world_model()
        ent = wm.upsert_entity(body.name, body.entity_type, body.state,
                                body.properties, body.confidence)
        return {"ok": True, "entity_id": ent.id}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/world-model/entities")
def list_entities(entity_type: str = None):
    try:
        from .engine import get_world_model
        wm = get_world_model()
        entities = wm.query_entities(entity_type=entity_type)
        return {"ok": True, "entities": [
            {"id": e.id, "name": e.name, "type": e.entity_type,
             "state": e.state, "confidence": e.confidence}
            for e in entities
        ], "count": len(entities)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/world-model/transitions/{entity_id}")
def entity_transitions(entity_id: str, limit: int = 20):
    try:
        from .engine import get_world_model
        wm = get_world_model()
        return {"ok": True, "transitions": wm.get_transitions(entity_id, limit)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/world-model/predict/{action_type}")
def predict_outcome(action_type: str):
    try:
        from .predictor import OutcomePredictor
        pred = OutcomePredictor()
        return {"ok": True, **pred.predict(action_type)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.post("/world-model/snapshot")
def take_snapshot(trigger: str = "manual"):
    try:
        from .engine import get_world_model
        wm = get_world_model()
        sid = wm.take_snapshot(trigger)
        return {"ok": True, "snapshot_id": sid}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/world-model/action-stats/{action_type}")
def action_stats(action_type: str):
    try:
        from .engine import get_world_model
        wm = get_world_model()
        return {"ok": True, **wm.get_action_stats(action_type)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


# ═══════════════════════════════════════════════════════════
# AUTOBIOGRAPHICAL MEMORY
# ═══════════════════════════════════════════════════════════

@router.get("/autobiographical/status")
def autobio_status():
    try:
        from modules.humanoid.memory_engine.autobiographical import get_autobiographical_memory
        am = get_autobiographical_memory()
        return {"ok": True, **am.get_stats()}
    except Exception as e:
        return {"ok": False, "error": str(e)}


class PeriodBody(BaseModel):
    name: str
    description: str = ""


@router.post("/autobiographical/period")
def start_period(body: PeriodBody):
    try:
        from modules.humanoid.memory_engine.autobiographical import get_autobiographical_memory
        am = get_autobiographical_memory()
        pid = am.start_period(body.name, body.description)
        return {"ok": True, "period_id": pid}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/autobiographical/periods")
def list_periods():
    try:
        from modules.humanoid.memory_engine.autobiographical import get_autobiographical_memory
        am = get_autobiographical_memory()
        return {"ok": True, "periods": am.get_all_periods()}
    except Exception as e:
        return {"ok": False, "error": str(e)}


class MilestoneBody(BaseModel):
    title: str
    description: str
    category: str
    importance: float = 0.7


@router.post("/autobiographical/milestone")
def record_milestone(body: MilestoneBody):
    try:
        from modules.humanoid.memory_engine.autobiographical import get_autobiographical_memory
        am = get_autobiographical_memory()
        mid = am.record_milestone(body.title, body.description,
                                   body.category, body.importance)
        return {"ok": True, "milestone_id": mid}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/autobiographical/milestones")
def list_milestones(category: str = None, limit: int = 50):
    try:
        from modules.humanoid.memory_engine.autobiographical import get_autobiographical_memory
        am = get_autobiographical_memory()
        return {"ok": True, "milestones": am.get_milestones(category, limit)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/autobiographical/identity")
def get_identity():
    try:
        from modules.humanoid.memory_engine.autobiographical import get_autobiographical_memory
        am = get_autobiographical_memory()
        return {"ok": True, "identity": am.get_identity()}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/autobiographical/relationships")
def get_relationships(entity_type: str = None):
    try:
        from modules.humanoid.memory_engine.autobiographical import get_autobiographical_memory
        am = get_autobiographical_memory()
        return {"ok": True, "relationships": am.get_relationships(entity_type)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/autobiographical/narrative")
def get_narrative():
    try:
        from modules.humanoid.memory_engine.autobiographical import get_autobiographical_memory
        am = get_autobiographical_memory()
        narrative = am.generate_narrative()
        return {"ok": True, "narrative": narrative}
    except Exception as e:
        return {"ok": False, "error": str(e)}


# ═══════════════════════════════════════════════════════════
# LIFELOG
# ═══════════════════════════════════════════════════════════

@router.get("/lifelog/status")
def lifelog_status():
    try:
        from modules.humanoid.memory_engine.lifelog import get_lifelog
        ll = get_lifelog()
        return {"ok": True, **ll.get_stats()}
    except Exception as e:
        return {"ok": False, "error": str(e)}


class LifelogEntry(BaseModel):
    event_type: str
    source: str
    perception: str = ""
    action: str = ""
    outcome: str = ""
    success: Optional[bool] = None
    reward: float = 0.0
    human_feedback: str = ""
    importance: float = 0.5
    tags: List[str] = []


@router.post("/lifelog/log")
def lifelog_log(body: LifelogEntry):
    try:
        from modules.humanoid.memory_engine.lifelog import get_lifelog
        ll = get_lifelog()
        lid = ll.log(
            event_type=body.event_type, source=body.source,
            perception=body.perception, action=body.action,
            outcome=body.outcome, success=body.success,
            reward=body.reward, human_feedback=body.human_feedback,
            importance=body.importance, tags=body.tags,
        )
        return {"ok": True, "entry_id": lid}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/lifelog/timeline")
def lifelog_timeline(hours: float = 24, limit: int = 100):
    try:
        from modules.humanoid.memory_engine.lifelog import get_lifelog
        ll = get_lifelog()
        return {"ok": True, "entries": ll.get_timeline(hours, limit)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/lifelog/search")
def lifelog_search(q: str, limit: int = 30):
    try:
        from modules.humanoid.memory_engine.lifelog import get_lifelog
        ll = get_lifelog()
        return {"ok": True, "entries": ll.search(q, limit)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/lifelog/failures")
def lifelog_failures(limit: int = 30):
    try:
        from modules.humanoid.memory_engine.lifelog import get_lifelog
        ll = get_lifelog()
        return {"ok": True, "entries": ll.get_failures(limit)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/lifelog/feedback-history")
def lifelog_feedback(limit: int = 30):
    try:
        from modules.humanoid.memory_engine.lifelog import get_lifelog
        ll = get_lifelog()
        return {"ok": True, "entries": ll.get_human_feedback_history(limit)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/lifelog/sessions")
def lifelog_sessions(limit: int = 20):
    try:
        from modules.humanoid.memory_engine.lifelog import get_lifelog
        ll = get_lifelog()
        return {"ok": True, "sessions": ll.get_session_history(limit)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


# ═══════════════════════════════════════════════════════════
# EXPERIENCE PLANNER
# ═══════════════════════════════════════════════════════════

class PlanRequest(BaseModel):
    goal: str
    context: Dict[str, Any] = {}
    actions: List[str] = []


@router.post("/planner/plan")
def create_plan(body: PlanRequest):
    try:
        from modules.humanoid.brain.experience_planner import get_experience_planner
        planner = get_experience_planner()
        plan = planner.plan(body.goal, body.context,
                           body.actions if body.actions else None)
        return {"ok": True, **plan.to_dict()}
    except Exception as e:
        return {"ok": False, "error": str(e)}


# ═══════════════════════════════════════════════════════════
# UNIFIED MEMORY CORTEX
# ═══════════════════════════════════════════════════════════

@router.get("/unified/recall")
def unified_recall(q: str, limit: int = 20, min_relevance: float = 0.0):
    try:
        from modules.humanoid.cortex.unified_memory import get_unified_memory
        um = get_unified_memory()
        results = um.recall(q, limit=limit, min_relevance=min_relevance)
        return {"ok": True, "results": [r.to_dict() for r in results],
                "count": len(results)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/unified/context/{action}")
def unified_context(action: str, goal: str = ""):
    try:
        from modules.humanoid.cortex.unified_memory import get_unified_memory
        um = get_unified_memory()
        return {"ok": True, **um.get_context_for_action(action, goal)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/unified/stats")
def unified_stats():
    try:
        from modules.humanoid.cortex.unified_memory import get_unified_memory
        um = get_unified_memory()
        return {"ok": True, **um.get_all_stats()}
    except Exception as e:
        return {"ok": False, "error": str(e)}
