from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from modules.nexus_core import (
    ScreenCalib,
    ScreenCalibSample,
    get_default_screen_calib_path,
    save_calib,
)


def _parse_sample(text: str) -> ScreenCalibSample:
    """Parse a CLI sample in the form x,y,yaw,pitch."""
    parts = [part.strip() for part in str(text or "").split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(
            "Sample must have the form x,y,yaw,pitch"
        )
    try:
        x, y, yaw, pitch = [float(part) for part in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc
    return ScreenCalibSample(x=x, y=y, yaw=yaw, pitch=pitch)


def build_parser() -> argparse.ArgumentParser:
    """Create the CLI parser for screen gaze calibration."""
    parser = argparse.ArgumentParser(
        description="Build and save a screen-to-neck calibration."
    )
    parser.add_argument("--width", type=int, required=True, help="Monitor width in pixels.")
    parser.add_argument(
        "--height", type=int, required=True, help="Monitor height in pixels."
    )
    parser.add_argument(
        "--zoom-center",
        type=float,
        default=1.0,
        help="Default zoom used when look_at_screen does not receive one.",
    )
    parser.add_argument(
        "--sample",
        action="append",
        default=[],
        type=_parse_sample,
        help="Calibration sample as x,y,yaw,pitch. Repeat this flag per sample.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=get_default_screen_calib_path(),
        help="Target JSON path for the calibration.",
    )
    return parser


def main() -> int:
    """Build a calibration file from manual samples."""
    parser = build_parser()
    args = parser.parse_args()

    if len(args.sample) < 2:
        parser.error("At least 2 --sample values are required.")

    # Manual operator flow:
    # 1. Pick reference pixels on the target monitor, typically center and corners.
    # 2. Use the current neck/PTZ control path to align the camera with each point.
    # 3. Record x,y,yaw,pitch for every pose and pass them through repeated --sample flags.
    calib = ScreenCalib.from_samples(
        W=args.width,
        H=args.height,
        samples=list(args.sample),
        zoom_center=args.zoom_center,
    )
    out_path = save_calib(calib, args.output)
    print(f"Saved calibration to {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
