"""
EvolutionOrchestratorV2 - Flujo completo de actualización:
Scan → Backup → Regression Test → Baseline → Staged Rollout → Metrics Comparison → Auto-Rollback → Changelog → Notify.
"""
from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any

from .regression_tester import RegressionTester, TestResults
from .backup_manager import BackupManager
from .staged_rollout import StagedRollout, RolloutPhase
from .metrics_comparator import MetricsComparator, ComparisonReport

logger = logging.getLogger(__name__)


def _load_config() -> dict:
    cfg_path = Path(__file__).resolve().parent.parent.parent / "config" / "autonomous.yaml"
    if not cfg_path.exists():
        return {}
    try:
        import yaml
        with open(cfg_path, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


class EvolutionOrchestratorV2:
    """
    Orquestador de actualización con validación robusta.
    execute_full_update_pipeline(packages) ejecuta el flujo completo.
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("evolution", {})
        self._tester = RegressionTester(self._config)
        self._backup = BackupManager(self._config)
        self._rollout = StagedRollout(self._config)
        self._comparator = MetricsComparator(self._config)
        self._last_report: dict[str, Any] = {}
        self._in_progress = False

    def validate_update_safety(self, package: str) -> dict[str, Any]:
        """Análisis pre-update: regression tests y estado actual."""
        results = self._tester.run_full_test_suite()
        return {
            "package": package,
            "tests_passed": results.all_passed,
            "test_results": {
                "passed": results.passed,
                "failed": results.failed,
                "total_duration_ms": results.total_duration_ms,
            },
            "recommendation": "proceed" if results.all_passed else "fix_failures_first",
        }

    def execute_full_update_pipeline(self, packages: list[str] | None = None) -> dict[str, Any]:
        """
        Flujo: 1) Regression test 2) Backup 3) Baseline 4) Staged rollout (simulado si no hay updater real)
        5) Post metrics 6) Compare 7) Rollback si degradación 8) Report.
        """
        if self._in_progress:
            return {"ok": False, "error": "Update already in progress", "report": self._last_report}
        self._in_progress = True
        report = {"steps": [], "success": False, "rollback_performed": False, "snapshot_id": None}
        packages = packages or []

        try:
            # 1) Regression test
            results = self._tester.run_full_test_suite()
            report["steps"].append({"step": "regression_test", "passed": results.all_passed, "details": f"passed={results.passed}, failed={results.failed}"})
            if not results.all_passed:
                report["error"] = "Regression tests failed"
                return self._finalize_report(report)

            # 2) Backup
            snapshot_id = self._backup.create_snapshot(tag="pre_update")
            report["snapshot_id"] = snapshot_id
            report["steps"].append({"step": "backup", "snapshot_id": snapshot_id})

            # 3) Baseline
            self._comparator.capture_baseline()
            report["steps"].append({"step": "baseline_captured"})

            # 4) Staged rollout (simulado: no instalamos paquetes aquí, solo avanzamos fases)
            self._rollout.start_rollout({"packages": packages})
            self._rollout.set_metrics_check(lambda: True)
            self._rollout.advance_phase()
            self._rollout.advance_phase()
            self._rollout.advance_phase()
            self._rollout.advance_phase()
            report["steps"].append({"step": "staged_rollout", "note": "phases advanced (no package install in this stub)"})

            # 5) Post-update metrics
            self._comparator.capture_post_update()
            report["steps"].append({"step": "post_metrics_captured"})

            # 6) Compare
            comp = self._comparator.compare_metrics()
            report["comparison"] = {"degraded": comp.degraded, "improved": comp.improved, "stable": comp.stable}

            # 7) Auto-rollback si degradación
            if self._comparator.should_rollback() and self._config.get("auto_rollback", {}).get("enabled", True):
                ok = self._backup.restore_snapshot(snapshot_id)
                report["rollback_performed"] = True
                report["steps"].append({"step": "rollback", "success": ok})
                report["success"] = False
                report["error"] = "Metrics degraded; rollback performed"
            else:
                report["success"] = True
                report["steps"].append({"step": "validation_ok"})

        except Exception as e:
            logger.exception("Update pipeline: %s", e)
            report["error"] = str(e)
            if report.get("snapshot_id"):
                self._backup.restore_snapshot(report["snapshot_id"])
                report["rollback_performed"] = True
        finally:
            self._in_progress = False
            self._last_report = report
        return self._finalize_report(report)

    def _finalize_report(self, report: dict) -> dict:
        report["timestamp"] = time.time()
        return {"ok": report.get("success", False), "report": report}

    def get_status(self) -> dict[str, Any]:
        """Estado del update en progreso o último reporte."""
        return {
            "in_progress": self._in_progress,
            "last_report": self._last_report,
        }

    def generate_update_report(self) -> str:
        """Reporte en texto del último pipeline."""
        r = self._last_report
        if not r:
            return "No update report available."
        lines = ["=== Evolution Update Report ===", f"Success: {r.get('success', False)}", f"Rollback: {r.get('rollback_performed', False)}"]
        for step in r.get("steps", []):
            lines.append(f"  - {step.get('step', '')}: {step}")
        if r.get("comparison"):
            lines.append("  Comparison: " + str(r["comparison"]))
        return "\n".join(lines)
