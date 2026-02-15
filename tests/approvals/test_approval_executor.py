from __future__ import annotations


def test_execute_approved_screen_action(monkeypatch):
    from modules.humanoid.approvals.executor import execute_approved

    called = {"action": None, "payload": None}

    def fake_execute_action(action, payload):
        called["action"] = action
        called["payload"] = payload
        return {"ok": True, "error": None, "evidence": {"before": None, "after": None}}

    monkeypatch.setattr("modules.humanoid.screen.actions.execute_action", fake_execute_action)

    item = {
        "action": "screen_act_destructive",
        "payload": {
            "confirm_text": "Eliminar",
            "expected_window": "Dashboard",
            "expected_process": "chrome.exe",
            "requested_action": "click",
            "requested_payload": {"x": 10, "y": 20, "confirm_text": "Eliminar", "expected_window": "Dashboard"},
        },
    }
    out = execute_approved(item, approval_id="abc123", resolved_by="test")
    assert out["ok"] is True
    assert called["action"] == "click"
    assert called["payload"]["approval_granted"] is True
    assert called["payload"]["record_evidence"] is True

