"""Bind ordered backend delivery stops to required mission waypoints."""
from __future__ import annotations

from dataclasses import dataclass
from typing import TypedDict


class StopRecord(TypedDict):
    order_id: str
    stop_sequence: int
    status: str


@dataclass(frozen=True, slots=True)
class Stop:
    order_id: str
    sequence: int
    waypoint_index: int


def bind_stops(records: list[StopRecord], indices: tuple[int, ...]) -> tuple[Stop, ...]:
    if len(indices) < 3 or len(records) != len(indices) - 2:
        raise ValueError("backend stop count does not match required mission stops")
    if any(record["status"] not in ("scheduled", "arrived") for record in records):
        raise ValueError("a backend stop has already completed or is invalid")
    sequences = [record["stop_sequence"] for record in records]
    orders = [record["order_id"] for record in records]
    if (any(type(seq) is not int or seq < 1 for seq in sequences)
            or sequences != sorted(sequences) or len(set(sequences)) != len(sequences)
            or any(not order for order in orders) or len(set(orders)) != len(orders)):
        raise ValueError("invalid ordered backend stop identity")
    return tuple(Stop(record["order_id"], record["stop_sequence"], index)
                 for record, index in zip(records, indices[1:-1], strict=True))


def stop_is_complete(records: list[StopRecord], stops: tuple[Stop, ...], current: int) -> bool:
    if len(records) != len(stops):
        raise ValueError("backend stops changed during mission")
    for index, (record, stop) in enumerate(zip(records, stops, strict=True)):
        if record["order_id"] != stop.order_id or record["stop_sequence"] != stop.sequence:
            raise ValueError("backend stop identity or ordering changed")
        status = record["status"]
        if status not in ("scheduled", "arrived", "completed"):
            raise ValueError("backend stop is cancelled or invalid")
        if index < current and status != "completed":
            raise ValueError("previous stop is no longer completed")
        if index > current and status == "completed":
            raise ValueError("a later stop completed before its turn")
    return records[current]["status"] == "completed"
