"""Asset hydration must preserve local edits and fail clearly on pointers."""

import json
import shutil
import subprocess
import sys
from pathlib import Path


def test_check_rejects_unhydrated_pointer_without_overwriting(tmp_path: Path):
    # Given an untouched LFS checkout in an isolated scene directory.
    source = Path(__file__).parents[1] / "scripts"
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    script = scripts / "hydrate-school-assets.py"
    shutil.copyfile(source / script.name, script)
    asset = json.loads((source / "school-assets.json").read_text())[0]
    (scripts / "school-assets.json").write_text(json.dumps([asset]))
    target = tmp_path / "models/props" / asset["path"]
    target.parent.mkdir(parents=True)
    pointer = f"version https://git-lfs.github.com/spec/v1\noid sha256:{asset['sha256']}\nsize {asset['size']}\n"
    target.write_text(pointer)
    # When checking offline, then the pointer remains intact and fails clearly.
    result = subprocess.run([sys.executable, str(script), "--check"], capture_output=True, text=True)
    assert result.returncode != 0
    assert "LFS pointer" in result.stderr
    assert target.read_text() == pointer


def test_hydration_preserves_modified_local_file(tmp_path: Path):
    # Given a local customized file that does not match its pinned Fuel bytes.
    source = Path(__file__).parents[1] / "scripts"
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    script = scripts / "hydrate-school-assets.py"
    shutil.copyfile(source / script.name, script)
    asset = json.loads((source / "school-assets.json").read_text())[0]
    (scripts / "school-assets.json").write_text(json.dumps([asset]))
    target = tmp_path / "models/props" / asset["path"]
    target.parent.mkdir(parents=True)
    target.write_bytes(b"custom local mesh")
    # When hydration is requested, then it refuses before downloading anything.
    result = subprocess.run([sys.executable, str(script)], capture_output=True, text=True)
    assert result.returncode != 0
    assert "Refusing to overwrite" in result.stderr
    assert target.read_bytes() == b"custom local mesh"
