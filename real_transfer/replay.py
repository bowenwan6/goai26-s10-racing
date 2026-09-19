"""Offline JSONL replay. No ROS, robot connection or command publication."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from real_transfer.shadow import ShadowSession


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    session = ShadowSession(config)
    # Exclusive creation prevents overwriting collected evidence.
    with args.input.open() as source, args.output.open("x") as target:
        for number, line in enumerate(source, 1):
            try:
                result = session.step(json.loads(line))
            except (ValueError, TypeError, KeyError, IndexError) as exc:
                session.guard.faults.add("malformed_replay_record")
                result = {
                    "mode": "shadow_only",
                    "motion_enabled": False,
                    "transport_command": [0, 0, 0],
                    "candidate": [0, 0, 0],
                    "candidate_computed": False,
                    "reasons": [f"malformed_record:{exc}"],
                }
            result["record"] = number
            target.write(json.dumps(result, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
