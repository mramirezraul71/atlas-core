from __future__ import annotations


def test_mttr_start_and_resolve(monkeypatch, tmp_path):
    monkeypatch.setenv("ATLAS_MTTR_DB_PATH", str(tmp_path / "mttr.sqlite"))
    from modules.humanoid.ans import mttr

    eid = mttr.start("test", "sig123", {"x": 1})
    assert isinstance(eid, int) and eid > 0
    mttr.resolve(eid, ok=True)
    s = mttr.stats(hours=24)
    assert s["count"] >= 1

