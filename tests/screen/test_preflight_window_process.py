from __future__ import annotations


def test_click_blocks_on_wrong_active_process(monkeypatch):
    import modules.humanoid.screen.actions as actions

    # Avoid needing GUI deps in unit test
    monkeypatch.setattr(actions, "_screen_deps_ok", lambda: True)
    monkeypatch.setattr(actions, "active_window_matches", lambda exp: True)
    monkeypatch.setattr(actions, "active_window_process_matches", lambda exp: False)
    monkeypatch.setattr(actions, "get_active_window_title", lambda: "Some Window")
    monkeypatch.setattr(actions, "get_active_window_process_path", lambda: r"C:\Windows\System32\notepad.exe")

    out = actions.execute_action("click", {"x": 10, "y": 10, "expected_process": "chrome.exe"})
    assert out["ok"] is False
    assert out["error"] == "wrong_active_process"
    assert "active_process_path" in out

