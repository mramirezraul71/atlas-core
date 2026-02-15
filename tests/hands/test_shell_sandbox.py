from __future__ import annotations

from pathlib import Path


def test_safe_shell_blocks_disallowed_cwd(monkeypatch, tmp_path: Path):
    from modules.humanoid.hands.safe_shell import SafeShellExecutor

    allowed = tmp_path / "allowed"
    disallowed = tmp_path / "disallowed"
    allowed.mkdir()
    disallowed.mkdir()

    monkeypatch.setenv("HANDS_ALLOWED_CWD_PREFIXES", str(allowed))
    sh = SafeShellExecutor(allowed_cmds=["python"])
    r = sh.run("python -c \"print('ok')\"", cwd=str(disallowed), timeout_sec=5)
    assert r["ok"] is False
    assert r["error"] == "cwd_not_allowed"

