"""Bind offline-verified routes to the exact collision mesh and fixture inputs."""

from __future__ import annotations

import hashlib
from pathlib import Path


def route_signature(mesh: Path, configuration: str) -> str:
    """Fail closed after geometry, placement, pads, limits or route points change."""
    digest = hashlib.sha256(mesh.read_bytes())
    digest.update(configuration.encode("utf-8"))
    return digest.hexdigest()
