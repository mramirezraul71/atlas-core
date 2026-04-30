"""
Tests de OptionsStatePublisher (JSON latest + historial rotativo).
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_BRAIN_ROOT = _REPO_ROOT / "atlas_options_brain_fase1"
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
if str(_BRAIN_ROOT) not in sys.path:
    sys.path.insert(0, str(_BRAIN_ROOT))

from atlas.core.options_publisher import OptionsStatePublisher  # noqa: E402


class _FakeReporter:
    """Mismo contrato que OptionsReporter.snapshot_state()."""

    def __init__(self, payload: dict) -> None:
        self._payload = payload

    def snapshot_state(self) -> dict:
        return dict(self._payload)


class TestOptionsStatePublisher(unittest.TestCase):
    def test_publish_writes_output_path(self) -> None:
        payload = {"timestamp": "t0", "portfolio": {"total_open_positions": 0}}
        with self._tmpdir() as td:
            latest = Path(td) / "nested" / "latest.json"
            pub = OptionsStatePublisher(
                _FakeReporter(payload),  # type: ignore[arg-type]
                output_path=str(latest),
            )
            out = pub.publish_once()
            self.assertEqual(out, payload)
            self.assertTrue(latest.is_file())
            with open(latest, encoding="utf-8") as f:
                disk = json.load(f)
            self.assertEqual(disk, payload)

    def test_history_creates_timestamped_file(self) -> None:
        payload = {"ok": True}
        with self._tmpdir() as td:
            td_path = Path(td)
            latest = td_path / "latest.json"
            hist = td_path / "history"
            pub = OptionsStatePublisher(
                _FakeReporter(payload),  # type: ignore[arg-type]
                output_path=str(latest),
                history_dir=str(hist),
            )
            pub.publish_once()
            files = list(hist.glob("options-state-*.json"))
            self.assertEqual(len(files), 1)
            with open(files[0], encoding="utf-8") as f:
                self.assertEqual(json.load(f), payload)

    def test_keep_last_prunes_old_files(self) -> None:
        payload = {"v": 1}
        with self._tmpdir() as td:
            td_path = Path(td)
            latest = td_path / "latest.json"
            hist = td_path / "history"
            hist.mkdir(parents=True, exist_ok=True)
            base_t = 1_700_000_000
            for i in range(4):
                p = hist / f"options-state-2020-01-0{i + 1}T12-00-00Z.json"
                p.write_text("{}", encoding="utf-8")
                os_m = base_t + i
                os.utime(p, (os_m, os_m))

            pub = OptionsStatePublisher(
                _FakeReporter(payload),  # type: ignore[arg-type]
                output_path=str(latest),
                history_dir=str(hist),
                keep_last=2,
            )
            time.sleep(0.05)
            pub.publish_once()
            remaining = sorted(hist.glob("options-state-*.json"))
            self.assertEqual(
                len(remaining),
                2,
                msg="deben quedar solo los 2 históricos más recientes",
            )
            by_mtime = sorted(remaining, key=lambda p: p.stat().st_mtime, reverse=True)
            with open(by_mtime[0], encoding="utf-8") as f:
                self.assertEqual(json.load(f), payload)

    def test_returned_dict_matches_disk(self) -> None:
        payload = {"a": 1, "b": [2, 3]}
        with self._tmpdir() as td:
            latest = Path(td) / "out.json"
            ret = OptionsStatePublisher(
                _FakeReporter(payload),  # type: ignore[arg-type]
                output_path=str(latest),
            ).publish_once()
            with open(latest, encoding="utf-8") as f:
                disk = json.load(f)
            self.assertEqual(ret, disk)
            self.assertEqual(ret, payload)

    def _tmpdir(self):
        return tempfile.TemporaryDirectory()


if __name__ == "__main__":
    unittest.main()
