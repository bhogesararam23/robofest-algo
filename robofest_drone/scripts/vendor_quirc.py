#!/usr/bin/env python3
"""Vendors the quirc QR-decoder library into robofest_drone/lib/quirc.

REQ-DER-110 (item 10): the firmware's QR path compiles against quirc
(dlbeer/quirc, ISC license) when present and degrades to "QR unsupported"
when absent, so builds never break on a missing dependency.

Usage:
    python scripts/vendor_quirc.py [--url URL]

Default source: https://github.com/dlbeer/quirc (master HEAD; the project
does not cut release tags). Only lib/ headers + sources are copied;
tests/demo files are dropped.
"""

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_FILES = ["lib/quirc.h", "lib/quirc_internal.h", "lib/decode.c",
              "lib/identify.c", "lib/quirc.c", "lib/version_db.c"]
DEST = Path(__file__).resolve().parent.parent / "lib" / "quirc"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="https://github.com/dlbeer/quirc.git")
    args = ap.parse_args()

    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td) / "quirc"
        print(f"[vendor_quirc] cloning {args.url} (master HEAD) ...")
        subprocess.run(
            ["git", "clone", "--depth", "1", args.url, str(tmp)],
            check=True,
        )
        DEST.mkdir(parents=True, exist_ok=True)
        for rel in REPO_FILES:
            src = tmp / rel
            if not src.exists():
                print(f"[vendor_quirc] missing upstream file: {rel}", file=sys.stderr)
                return 1
            shutil.copy2(src, DEST / Path(rel).name)
    print(f"[vendor_quirc] installed -> {DEST}")
    print("[vendor_quirc] re-run platformio build; code_reader_read_qr is now active.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
