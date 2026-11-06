"""Persist a cursor over real backend orders and assign one bounded SITL flight at a time."""

from __future__ import annotations

import logging
import threading
import time
from datetime import UTC, datetime
from pathlib import Path
from types import TracebackType
from typing import Self
from uuid import UUID

import httpx2
from pydantic import BaseModel, ConfigDict, TypeAdapter
from school_relay_runtime import atomic_write, http_client

LOG = logging.getLogger("school-orders")


class DispatchOrder(BaseModel):
    model_config = ConfigDict(frozen=True)
    id: UUID
    status: str
    created_at: datetime
    destination_id: str | None = None
    received: bool = False


class DispatchCursor(BaseModel):
    model_config = ConfigDict(frozen=True)
    after: datetime
    after_id: UUID


class ControllerState(BaseModel):
    timestamp: float
    completed_order_id: UUID | None = None


class Assignment(BaseModel):
    timestamp: float
    order_id: UUID | None
    error: str | None = None
    destination_id: str | None = None
    received: bool = False


class OrderRelay:
    def __init__(self, root: Path, backend: str) -> None:
        self.root, self.backend = root, backend
        cursor_path = root / "order-cursor.json"
        if cursor_path.exists():
            self.cursor = DispatchCursor.model_validate_json(cursor_path.read_bytes())
        else:
            self.cursor = DispatchCursor(after=datetime.now(UTC), after_id=UUID(int=0))
            self._save_cursor()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="school-orders", daemon=True
        )

    def _save_cursor(self) -> None:
        atomic_write(
            self.root, "order-cursor.json", self.cursor.model_dump_json().encode()
        )

    def _assign(
        self,
        order_id: UUID | None,
        error: str | None = None,
        destination_id: str | None = None,
        received: bool = False,
    ) -> None:
        assignment = Assignment(
            timestamp=time.time(),
            order_id=order_id,
            error=error,
            destination_id=destination_id,
            received=received,
        )
        atomic_write(
            self.root, "order-assignment.json", assignment.model_dump_json().encode()
        )

    def poll_once(self, client: httpx2.Client) -> None:
        response = client.get(
            self.backend + "/api/v1/simulation/orders",
            params={
                "after": self.cursor.after.isoformat(),
                "after_id": str(self.cursor.after_id),
            },
        )
        response.raise_for_status()
        orders = TypeAdapter(list[DispatchOrder]).validate_python(response.json())
        state = ControllerState.model_validate_json(
            (self.root / "state.json").read_bytes()
        )
        if not 0 <= time.time() - state.timestamp <= 5:
            raise TimeoutError("Order controller state is stale")
        if state.completed_order_id is not None:
            (self.root / "orders").mkdir(exist_ok=True)
            name = f"orders/{state.completed_order_id}.json"
            if not (self.root / name).exists():
                completed = Assignment(
                    timestamp=time.time(), order_id=state.completed_order_id
                )
                atomic_write(self.root, name, completed.model_dump_json().encode())
        for order in orders:
            if (
                order.status in {"cancelled", "failed", "delivered", "refunded"}
                or order.id == state.completed_order_id
            ):
                self.cursor = DispatchCursor(after=order.created_at, after_id=order.id)
                self._save_cursor()
                continue
            self._assign(order.id, destination_id=order.destination_id, received=order.received)
            return
        self._assign(None)

    def __enter__(self) -> Self:
        self._thread.start()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self._stop.set()
        self._thread.join(timeout=5)
        self._assign(None)
        if self._thread.is_alive():
            raise RuntimeError("Order reporter did not stop within its HTTP timeout")

    def _run(self) -> None:
        with http_client(telemetry=True) as client:
            while not self._stop.is_set():
                started = time.monotonic()
                try:
                    self.poll_once(client)
                except (OSError, ValueError, RuntimeError, httpx2.HTTPError) as error:
                    self._assign(None, str(error))
                    LOG.warning("Order feed unavailable: %s", error)
                self._stop.wait(max(0, 1 - (time.monotonic() - started)))
