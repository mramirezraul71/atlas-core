from __future__ import annotations


def test_atomic_patcher_replace_block_includes_anchors():
    from modules.atlas_architect.atomic_patcher import AtomicPatcher, ReplaceBlock

    before = "a\n# START\nx\n# END\nb\n"
    ops = [ReplaceBlock(anchor_start="# START", anchor_end="# END", new_block="# START\nY\n# END", include_anchors=True)]
    after, diff = AtomicPatcher().apply(before, ops, file_path="x.txt")
    assert "Y" in after
    assert diff


def test_atomic_patcher_regex_replace():
    from modules.atlas_architect.atomic_patcher import AtomicPatcher, RegexReplace

    before = "port = 8000\n"
    ops = [RegexReplace(pattern=r"8000", repl="8001", count=1)]
    after, _ = AtomicPatcher().apply(before, ops, file_path="x.txt")
    assert "8001" in after

