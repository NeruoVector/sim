"""A replacement cannot omit, reorder, or move an outstanding stop."""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sitl"))
from coordinates import Enu
from replacement import verify_replacement


def test_replacement_requires_held_start_and_exact_remaining_stops() -> None:
    old = [Enu(0, 0, 8), Enu(0, 30, 8), Enu(0, 45, 8), Enu(0, 0, 8)]
    new = [Enu(0, 15, 8), Enu(0, 30, 8), Enu(0, 45, 8), Enu(0, 0, 8)]
    indices = (0, 1, 2, 3)
    assert verify_replacement(old, indices, new, indices, Enu(0, 15, 8)) == indices
    with_via = [new[0], Enu(-5, 20, 8), *new[1:]]
    assert verify_replacement(old, indices, with_via, (0, 1, 2, 3, 4), new[0]) == (0, 2, 3, 4)
    with pytest.raises(ValueError):
        verify_replacement(old, indices, [new[0], new[2], new[1], new[3]], indices, new[0])
    with pytest.raises(ValueError):
        verify_replacement(old, indices, new, indices, Enu(0, 5, 8))
