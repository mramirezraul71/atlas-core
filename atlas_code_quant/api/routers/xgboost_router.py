"""Router REST XGBoost Signal — prefijo /api/v2/quant/xgboost."""
from __future__ import annotations

import sqlite3
from pathlib import Path
from typing import Any

from fastapi import APIRouter, Body, HTTPException
from pydantic import BaseModel, Field

from config.settings import settings
from learning.xgboost_signal import (
    XGBoostExitAdvisor,
    XGBoostSignalScorer,
    generate_audit_report,
)
from learning.xgboost_signal.model_loader import XGBoostModelLoader
from learning.xgboost_signal.model_trainer import (
    _count_real_trades,
    ensure_xgboost_feature_log_table,
    get_training_phase,
    train_if_ready,
)

router = APIRouter(prefix="/api/v2/quant/xgboost", tags=["xgboost"])


class ExitAdviceBody(BaseModel):
    position: dict[str, Any] = Field(default_factory=dict)
    market_snapshot: dict[str, Any] = Field(default_factory=dict)


@router.post("/score")
async def score_candidate(candidate: dict[str, Any] = Body(default_factory=dict)) -> dict[str, Any]:
    scorer = XGBoostSignalScorer.get_instance()
    return scorer.score(candidate)


@router.post("/exit-advice")
async def advise_exit(body: ExitAdviceBody) -> dict[str, Any]:
    advisor = XGBoostExitAdvisor.get_instance()
    return advisor.advise_exit(body.position, body.market_snapshot)


@router.get("/status")
async def get_status() -> dict[str, Any]:
    loader = XGBoostModelLoader.get_instance()
    loader.load()
    p = Path(settings.journal_db_path)
    n_real = 0
    phase = 0
    if p.is_file():
        conn = sqlite3.connect(str(p))
        try:
            ensure_xgboost_feature_log_table(conn)
            n_real = _count_real_trades(conn)
            phase = get_training_phase(n_real)
        finally:
            conn.close()
    meta = loader.meta if loader.is_loaded() else {}
    slim = {k: meta[k] for k in ("test_auc", "baseline_auc", "beats_baseline", "phase") if k in meta}
    return {
        "enabled": settings.xgboost_enabled,
        "phase": phase,
        "n_real_trades": n_real,
        "model_loaded": loader.is_loaded(),
        "model_path": str(loader.model_path()) if settings.xgboost_enabled else None,
        "meta": slim,
    }


@router.post("/train")
async def trigger_training() -> dict[str, Any]:
    if not settings.xgboost_enabled:
        return {"ok": False, "message": "QUANT_XGBOOST_ENABLED=false — no se entrena."}
    res = train_if_ready()
    if res is None:
        return {"ok": False, "message": "train_if_ready retornó None."}
    return {
        "ok": True,
        "phase": res.phase,
        "n_real_trades": res.n_real_trades,
        "model_path": res.model_path,
        "metrics": res.metrics,
        "message": res.message,
    }


@router.post("/audit-report")
async def build_audit_report() -> dict[str, Any]:
    if not settings.xgboost_enabled:
        raise HTTPException(status_code=400, detail="Módulo deshabilitado")
    jp, mp = generate_audit_report()
    return {"json": str(jp), "markdown": str(mp)}
