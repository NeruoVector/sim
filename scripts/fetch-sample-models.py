#!/usr/bin/env python3
"""Gazebo Fuel 에서 시뮬레이션용 샘플(더미) 모델을 내려받는다.

모든 모델은 OpenRobotics 가 공개한 CC0 모델만 사용한다.
실제 모델을 저장소에 커밋한 뒤에도, 새 환경에서 같은 에셋을 복원하거나
다른 샘플을 추가할 때 이 스크립트를 사용할 수 있다.

사용:
    python3 scripts/fetch-sample-models.py
    python3 scripts/fetch-sample-models.py --force
"""

from __future__ import annotations

import argparse
import json
import shutil
import urllib.parse
import urllib.request
from pathlib import Path

FUEL_BASE = "https://fuel.gazebosim.org/1.0"

# 저장 디렉터리 이름 -> (Gazebo Fuel 소유자, 모델 이름)
# 모두 CC0 모델만 사용한다.
SAMPLE_MODELS: dict[str, tuple[str, str]] = {
    "construction-barrel": ("OpenRobotics", "Construction Barrel"),
    "construction-cone": ("OpenRobotics", "Construction Cone"),
    "jersey-barrier": ("OpenRobotics", "Jersey Barrier"),
    "fire-hydrant": ("OpenRobotics", "Fire hydrant"),
    "oak-tree": ("OpenRobotics", "Oak tree"),
    "pine-tree": ("OpenRobotics", "Pine Tree"),
    "prius-hybrid": ("OpenRobotics", "Prius Hybrid"),
    "office-building": ("OpenRobotics", "Office Building"),
    "dumpster": ("OpenRobotics", "Dumpster"),
    "walking-actor": ("chapulina", "Walking actor"),
}


def get_json(url: str) -> dict:
    with urllib.request.urlopen(url, timeout=60) as response:
        return json.load(response)


def download(url: str, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(url, timeout=120) as response:
        with destination.open("wb") as output:
            shutil.copyfileobj(response, output)


def collect_files(node: dict) -> list[str]:
    """Fuel file_tree 를 순회해 다운로드할 파일 경로 목록을 만든다."""
    children = node.get("children", [])
    if not children:
        path = node.get("path")
        return [path] if path else []

    files: list[str] = []
    for child in children:
        files.extend(collect_files(child))
    return files


def fetch_model(slug: str, owner: str, name: str, models_dir: Path, force: bool) -> None:
    quoted_name = urllib.parse.quote(name, safe="")
    metadata = get_json(f"{FUEL_BASE}/{owner}/models/{quoted_name}")
    version = metadata["version"]
    license_name = metadata.get("license_name", "unknown")
    license_url = metadata.get("license_url", "")

    tree = get_json(f"{FUEL_BASE}/{owner}/models/{quoted_name}/{version}/files")
    target = models_dir / "props" / slug

    if target.exists() and not force:
        print(f"skip  {slug} (already exists)")
        return

    if force and target.exists():
        shutil.rmtree(target)

    files: list[str] = []
    for node in tree["file_tree"]:
        files.extend(collect_files(node))
    for path in files:
        # 썸네일은 Gazebo 실행에 필요 없으므로 제외한다.
        if path.startswith("/thumbnails/"):
            continue
        relative = path.lstrip("/")
        encoded_path = urllib.parse.quote(path, safe="/")
        url = f"{FUEL_BASE}/{owner}/models/{quoted_name}/{version}/files{encoded_path}"
        download(url, target / relative)

    rewrite_model_uris(target, name, slug)

    attribution = (
        f"# {name}\n\n"
        f"- Source: https://fuel.gazebosim.org/1.0/{owner}/models/{quoted_name}\n"
        f"- Version: {version}\n"
        f"- License: {license_name}\n"
        f"- License URL: {license_url}\n"
    )
    (target / "ATTRIBUTION.md").write_text(attribution, encoding="utf-8")
    print(f"fetch {slug}: {len(files)} files ({license_name})")


def rewrite_model_uris(target: Path, name: str, slug: str) -> None:
    """원본 모델 이름을 저장소 디렉터리 이름(slug)으로 맞춘다.

    Fuel 모델은 ``model://Oak tree/...`` 처럼 원본 이름을 참조하는 경우가 많다.
    디렉터리 이름을 바꾸면 그대로는 리소스를 찾지 못하므로 텍스트 파일의 URI 만
    ``model://oak-tree/...`` 로 바꾼다.
    """
    aliases = {
        name,
        name.replace(" ", "_"),
        name.replace(" ", "-").lower(),
        name.lower().replace(" ", "_"),
    }
    text_suffixes = {".sdf", ".config", ".xml", ".txt"}
    for path in target.rglob("*"):
        if not path.is_file() or path.suffix.lower() not in text_suffixes:
            continue
        text = path.read_text(encoding="utf-8")
        rewritten = text
        for alias in sorted(aliases, key=len, reverse=True):
            rewritten = rewritten.replace(f"model://{alias}/", f"model://{slug}/")
        if rewritten != text:
            path.write_text(rewritten, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--models-dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "models",
    )
    parser.add_argument("--force", action="store_true", help="기존 모델을 덮어쓴다")
    args = parser.parse_args()

    for slug, (owner, name) in SAMPLE_MODELS.items():
        fetch_model(slug, owner, name, args.models_dir, args.force)


if __name__ == "__main__":
    main()
