#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
# Run: python3 sim/scripts/hydrate-school-assets.py [--check]
"""Restore only pinned school CV assets; never overwrite custom non-pointer files."""

import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import TypedDict


class Asset(TypedDict):
    path: str
    sha256: str
    size: int
    url: str


def matches(path: Path, asset: Asset) -> bool:
    """Check both LFS identity fields before accepting bytes."""
    return path.stat().st_size == asset["size"] and hashlib.sha256(path.read_bytes()).hexdigest() == asset["sha256"]


def hydrate(asset: Asset, *, check: bool) -> None:
    """Atomically replace a matching pointer, preserving all other local files."""
    target = Path(__file__).resolve().parents[1] / "models/props" / asset["path"]
    if matches(target, asset):
        return
    pointer = (f"version https://git-lfs.github.com/spec/v1\noid sha256:{asset['sha256']}\nsize {asset['size']}\n").encode()
    if target.read_bytes() != pointer:
        raise SystemExit(f"Refusing to overwrite modified asset: {target}")
    if check:
        raise SystemExit(f"Asset is still an LFS pointer: {target}; run {Path(__file__).name}")
    with tempfile.TemporaryDirectory(dir=target.parent, prefix=".school-asset-") as directory:
        candidate = Path(directory) / "download"
        subprocess.run(["curl", "--fail", "--location", "--retry", "2", "--max-time", "120", "--silent", "--show-error", "--output", str(candidate), asset["url"]], check=True)
        if not matches(candidate, asset):
            raise SystemExit(f"Fuel asset hash/size mismatch: {asset['url']}; original pointer preserved")
        candidate.chmod(0o644)
        candidate.replace(target)
    print(f"hydrated {asset['path']}")


def main() -> None:
    """Check or hydrate the exact CC0 versions recorded beside this script."""
    assets: list[Asset] = json.loads(Path(__file__).with_name("school-assets.json").read_text())
    for asset in assets:
        hydrate(asset, check="--check" in sys.argv[1:])
    print("school CV asset hashes and sizes: OK")


if __name__ == "__main__":
    main()
