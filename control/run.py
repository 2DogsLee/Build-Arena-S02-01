"""One-click reproduction of the S02 mission (payload + reusable booster).

Usage (from this directory, with the project venv python):

    python run.py [--run-dir <dir>] [--no-gates]

It runs the official BuildArena CLI end-to-end:
    besiege_cli run --bsg machine.bsg --controller controller_mission.py ...
then converts the recorded telemetry into trajectory.csv and evaluates the
acceptance gates G1-G5 with offline/gates.py. No human input is required.
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent          # .../entry_s02_.../control
PRODUCT = HERE.parent                           # .../entry_s02_...
REPO = Path(r"D:\projects\build_arena\BuildArena-2.0-s02")
CONTROL = REPO / "control"
BSG = PRODUCT / "machine.bsg"
CONTROLLER = HERE / "controller_mission.py"
GATES = PRODUCT / "offline" / "gates.py"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path,
                        default=PRODUCT / "final_run",
                        help="output directory for this run (must be new/empty)")
    parser.add_argument("--bsg", type=Path, default=BSG)
    parser.add_argument("--no-gates", action="store_true")
    parser.add_argument("--timeout", type=float, default=3000.0,
                        help="controller wall-clock timeout (s)")
    parser.add_argument("--post-hold", type=float, default=3.0)
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    if not args.bsg.is_file():
        print(f"machine bsg not found: {args.bsg}", file=sys.stderr)
        return 2
    if run_dir.exists() and any(run_dir.iterdir()):
        print(f"run dir must be new/empty: {run_dir}", file=sys.stderr)
        return 2
    run_dir.mkdir(parents=True, exist_ok=True)

    cmd = [sys.executable, "-m", "besiege_cli", "run",
           "--bsg", str(args.bsg),
           "--controller", str(CONTROLLER),
           "--sandbox", "LONE ORB",
           "--run-dir", str(run_dir),
           "--telemetry-hz", "25",
           "--telemetry-profile", "full",
           "--recorder-hz", "25",
           "--pre-controller-hold", "1",
           "--post-completion-hold", str(args.post_hold),
           "--controller-timeout", str(args.timeout),
           "--launch-timeout", "180",
           "--timeout", "120"]
    print("running:", " ".join(cmd), flush=True)
    result = subprocess.run(cmd, cwd=str(CONTROL))
    print(f"besiege_cli exit code: {result.returncode}", flush=True)

    # archive the controller / outputs for provenance
    for name in ("controller_snapshot.py", "input_manifest.json", "summary.json",
                 "mission_summary.json", "mission.json", "frames.json"):
        src = run_dir / name
        if src.is_file():
            print(f"  output: {src.name} ({src.stat().st_size} bytes)")

    if args.no_gates:
        return 0
    if not GATES.is_file():
        print("gate checker not found", file=sys.stderr)
        return 2
    gate = subprocess.run([sys.executable, str(GATES), str(run_dir)])
    return gate.returncode


if __name__ == "__main__":
    raise SystemExit(main())
