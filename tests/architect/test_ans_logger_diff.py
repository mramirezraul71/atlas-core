from __future__ import annotations

from pathlib import Path


def test_ans_logger_writes_diff_file(tmp_path: Path):
    from modules.atlas_architect.ans_logger import ANSBitacoraLogger

    logger = ANSBitacoraLogger(repo_root=tmp_path)
    p = tmp_path / "x.txt"
    before = "a\nb\n"
    after = "a\nc\n"
    rec = logger.log_change(p, before, after, justification="test", ok=True)
    assert rec.ok is True
    diff_path = Path(rec.diff_path)
    assert diff_path.exists()
    txt = diff_path.read_text(encoding="utf-8", errors="ignore")
    assert "-b" in txt or "+c" in txt

