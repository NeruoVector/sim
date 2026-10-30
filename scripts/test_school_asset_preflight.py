"""Exercise STL readiness without requiring the missing school assets."""
import struct
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "docker"))
from school_asset_preflight import AssetError, check_stl


def expect_failure(path: Path) -> None:
    try:
        check_stl(path)
    except AssetError:
        return
    raise AssertionError(f"invalid asset accepted: {path}")


def main() -> None:
    with tempfile.TemporaryDirectory() as temp:
        mesh = Path(temp) / "mesh.stl"
        triangle = (0., 0., 1., 0., 0., 0., 1., 0., 0., 0., 1., 0., 0)
        mesh.write_bytes(b"binary".ljust(80, b"\0") + struct.pack("<I12fH", 1, *triangle))
        check_stl(mesh)
        mesh.write_text("solid s\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid s\n", encoding="ascii")
        check_stl(mesh)
        for data in (b"version https://git-lfs.github.com/spec/v1\n", b"", b"solid s\nendsolid s\n", b"binary".ljust(80, b"\0") + struct.pack("<I", 1)):
            mesh.write_bytes(data)
            expect_failure(mesh)
        degenerate = (0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0)
        mesh.write_bytes(b"binary".ljust(80, b"\0") + struct.pack("<I12fH", 1, *degenerate))
        expect_failure(mesh)
        expect_failure(Path(temp) / "missing.stl")
    print("school asset preflight tests passed")


if __name__ == "__main__":
    main()
