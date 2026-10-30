#!/usr/bin/env python3
"""Check that school STL assets contain usable finite triangle geometry."""

import math
import re
import struct
import sys
from pathlib import Path


class AssetError(Exception):
    """An asset cannot be safely loaded as an STL mesh."""


def check_stl(path: Path) -> None:
    try:
        data = path.read_bytes()
    except OSError as error:
        raise AssetError(f"{path}: cannot read: {error}") from error
    if data.startswith(b"version https://git-lfs.github.com/spec/v1"):
        raise AssetError(f"{path}: Git LFS pointer, not a mesh; restore the LFS object")
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if count > 0 and len(data) == 84 + 50 * count:
            triangles = (struct.unpack_from("<12fH", data, 84 + 50 * i)[3:12]
                         for i in range(count))
            _check_triangles(path, triangles)
            return
    try:
        text = data.decode("ascii")
    except UnicodeDecodeError as error:
        raise AssetError(f"{path}: invalid binary STL length or triangle count") from error
    number = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
    point = rf"({number})\s+({number})\s+({number})"
    facet = re.compile(rf"facet\s+normal\s+{point}\s+outer\s+loop\s+vertex\s+{point}\s+vertex\s+{point}\s+vertex\s+{point}\s+endloop\s+endfacet", re.IGNORECASE)
    header = re.match(r"\s*solid[^\r\n]*(?:\r?\n)", text, re.IGNORECASE)
    if not header:
        raise AssetError(f"{path}: malformed or empty ASCII STL")
    position = header.end()
    triangles = []
    while True:
        whitespace = re.match(r"\s*", text[position:])
        position += whitespace.end()
        match = facet.match(text, position)
        if match is None:
            break
        values = tuple(map(float, match.groups()))
        if not all(math.isfinite(value) for value in values):
            raise AssetError(f"{path}: non-finite STL coordinate")
        triangles.append(values[3:])
        position = match.end()
    if not triangles or not re.fullmatch(r"endsolid[^\r\n]*\s*", text[position:], re.IGNORECASE):
        raise AssetError(f"{path}: malformed or empty ASCII STL")
    _check_triangles(path, triangles)


def _check_triangles(path: Path, triangles) -> None:
    for triangle in triangles:
        if not all(math.isfinite(value) for value in triangle):
            raise AssetError(f"{path}: non-finite STL vertex")
        a, b, c = triangle[:3], triangle[3:6], triangle[6:9]
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        cross = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        if not all(math.isfinite(value) for value in cross) or not any(cross):
            raise AssetError(f"{path}: degenerate or oversized STL triangle")


def main() -> int:
    try:
        for argument in sys.argv[1:]:
            check_stl(Path(argument))
    except AssetError as error:
        print(f"school runtime asset preflight failed: {error}", file=sys.stderr)
        return 1
    if len(sys.argv) < 2:
        print("school runtime asset preflight requires STL paths", file=sys.stderr)
        return 1
    print("school runtime STL assets: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
