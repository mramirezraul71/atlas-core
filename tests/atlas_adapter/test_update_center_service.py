from __future__ import annotations

from atlas_adapter.services.update_center import UpdateCenterService


def test_policy_blocks_runtime_sensitive_item(tmp_path):
    svc = UpdateCenterService(tmp_path)
    item = {"id": "trading-runtime-core", "name": "Trading Runtime Core", "category": "runtime"}
    out = svc.classify_item(item, "tools")
    assert out["policy_class"] == "never_auto_update"
    assert out["policy_status_label"] == "BLOQUEADO"
    assert out["auto_update_allowed"] is False


def test_policy_summary_counts_by_class(tmp_path):
    svc = UpdateCenterService(tmp_path)
    rows = [
        {"id": "git", "name": "Git", "category": "devops"},
        {"id": "trading-core", "name": "Trading", "category": "runtime"},
        {"id": "gpu-driver", "name": "NVIDIA Driver", "category": "driver", "critical": True},
    ]
    annotated = svc.annotate_rows(rows, "software")
    summary = svc.policy_summary(annotated)
    assert summary["manual_review_required"] == 1
    assert summary["never_auto_update"] == 1
    assert summary["security_critical"] == 1


def test_runtime_and_datascience_tools_are_manual_not_blocked(tmp_path):
    svc = UpdateCenterService(tmp_path)
    node = svc.classify_item({"id": "node", "name": "Node Runtime", "category": "dependency"}, "tools")
    pandas = svc.classify_item({"id": "pandas", "name": "Pandas", "category": "dependency"}, "tools")
    assert node["policy_class"] == "manual_review_required"
    assert node["policy_status_label"] == "MANUAL"
    assert pandas["policy_class"] == "manual_review_required"
    assert pandas["policy_status_label"] == "MANUAL"


def test_ccxt_stays_blocked_for_live_sensitivity(tmp_path):
    svc = UpdateCenterService(tmp_path)
    ccxt = svc.classify_item({"id": "ccxt", "name": "CCXT Trading API", "category": "dependency"}, "tools")
    assert ccxt["policy_class"] == "never_auto_update"
    assert ccxt["policy_status_label"] == "BLOQUEADO"


def test_auto_cycle_due_after_interval(tmp_path):
    svc = UpdateCenterService(tmp_path)
    svc.save_config({"enabled": True, "scan_interval_sec": 60})
    assert svc.should_run_cycle() is True
    svc.set_auto_cycle(True, reason="test")
    assert svc.should_run_cycle() is False
    svc.set_auto_cycle(False, reason="test")
    assert svc.should_run_cycle() is False


def test_register_and_sync_job(tmp_path):
    svc = UpdateCenterService(tmp_path)
    svc.register_job("job123", "tools", "update_all", "manual", {"count": 2})
    synced = svc.sync_job("job123", {"status": "running", "done": 1, "total": 2})
    assert synced["status"] == "running"
    assert synced["done"] == 1
    state = svc.get_state(events_limit=5)
    assert "job123" in state.get("jobs", {})
