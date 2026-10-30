"""Validate a replacement against the aircraft hold and remaining stops."""
from __future__ import annotations

import math

from coordinates import Enu


def verify_replacement(old_route: list[Enu], old_indices: tuple[int, ...],
                       new_route: list[Enu], new_indices: tuple[int, ...], hold: Enu) -> tuple[int, ...]:
    if len(old_indices) < 3 or len(new_indices) < len(old_indices):
        raise ValueError("replacement omitted remaining mandatory stops")
    if math.dist((new_route[0].east_m, new_route[0].north_m, new_route[0].up_m),
                 (hold.east_m, hold.north_m, hold.up_m)) > 2.0:
        raise ValueError("replacement does not start at the held aircraft")
    matched = [new_indices[0]]
    remaining = iter(new_indices[1:])
    for old_index in old_indices[1:]:
        old = old_route[old_index]
        for new_index in remaining:
            new = new_route[new_index]
            if math.dist((old.east_m, old.north_m, old.up_m),
                         (new.east_m, new.north_m, new.up_m)) <= 0.1:
                matched.append(new_index)
                break
        else:
            raise ValueError("replacement changed or reordered a mandatory stop")
    if matched[-1] != new_indices[-1]:
        raise ValueError("replacement must end at the original home")
    return tuple(matched)
