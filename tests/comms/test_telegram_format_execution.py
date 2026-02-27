from __future__ import annotations


def test_format_status_message_includes_execution_and_evidence():
    from modules.humanoid.comms.telegram_poller import _format_status_message

    msg = _format_status_message(
        "approve",
        "abc123",
        "approved",
        "",
        {
            "ok": True,
            "ms": 123,
            "result": {"evidence": {"before": r"C:\ATLAS_PUSH\snapshots\hands_eyes\actions\before.png", "after": r"C:\ATLAS_PUSH\snapshots\hands_eyes\actions\after.png"}},
        },
    )
    assert "Ejecución" in msg
    assert "before" in msg
    assert "after" in msg

