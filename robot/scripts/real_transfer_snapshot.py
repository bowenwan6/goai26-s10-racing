"""Freeze/compare locally materialized source files; never fetch missing assets."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = "defbb719ac3822941479caccdee2f35ef0547cf4"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path)
    args = parser.parse_args()
    commit = subprocess.check_output(
        ["git", "-C", str(args.baseline), "rev-parse", "HEAD"],
        text=True,
    ).strip()
    if commit != EXPECTED:
        raise SystemExit("unexpected baseline commit")
    names = subprocess.check_output(
        ["git", "-C", str(args.baseline), "ls-files", "-z"],
        text=True,
    ).split("\0")
    hashes = {}
    for name in names:
        source, copy = args.baseline / name, ROOT / name
        if name and source.is_file() and copy.is_file():
            a = hashlib.sha256(source.read_bytes()).hexdigest()
            b = hashlib.sha256(copy.read_bytes()).hexdigest()
            if a != b:
                raise SystemExit(f"baseline changed: {name}")
            hashes[name] = a
    record = {
        "source_url": "https://github.com/bowenwan6/goai26-s10-racing",
        "source_commit": commit,
        "materialized_files": hashes,
        "note": "Materialized subset only; SDK/model bundles/full-course assets not copied.",
    }
    output = ROOT / "real_transfer/source_snapshot.json"
    with output.open("x") as stream:
        json.dump(record, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(f"Frozen {len(hashes)} unchanged baseline files: {output}")


if __name__ == "__main__":
    main()
