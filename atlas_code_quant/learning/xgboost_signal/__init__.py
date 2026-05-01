"""Módulo XGBoost Signal — scoring pre-trade y exit advisor (opt-in via QUANT_XGBOOST_ENABLED)."""

from .audit_report import generate_audit_report
from .exit_advisor import XGBoostExitAdvisor
from .model_loader import XGBoostModelLoader
from .signal_scorer import XGBoostSignalScorer

__all__ = [
    "XGBoostSignalScorer",
    "XGBoostExitAdvisor",
    "XGBoostModelLoader",
    "generate_audit_report",
]
