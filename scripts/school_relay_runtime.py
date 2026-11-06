"""Independent telemetry ownership and serialized, fail-closed inference publication."""

from __future__ import annotations

import json
import logging
import math
import socket
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import httpx2
from pydantic import BaseModel, ConfigDict

from neurovector_vision.models import DetectResponse
from school_mission import Phase

LOG = logging.getLogger("school-relay")


class MissionContext(BaseModel):
    phase: Phase
    destination_id: str | None = None
    route_enu: list[tuple[float, float, float]] | None = None


class SimState(BaseModel):
    model_config = ConfigDict(frozen=True)
    timestamp: float
    camera_timestamp: float
    position_enu: tuple[float, float, float] | None
    mode: str
    armed: bool
    drone_id: str
    mission: MissionContext | None = None
    waypoint: int = 0

    def require_fresh(self) -> None:
        now = time.time()
        if not 0 <= now - self.timestamp <= 5:
            raise TimeoutError("Simulator data is stale")
        if not 0 <= now - self.camera_timestamp <= 5:
            raise TimeoutError("Simulator/camera data is stale")
        if self.position_enu is None:
            raise RuntimeError("Simulator position is unavailable")


def atomic_write(root: Path, name: str, content: bytes) -> None:
    target = root / name
    temp = target.with_suffix(".relay.tmp")
    temp.write_bytes(content)
    temp.replace(target)


def http_client(*, telemetry: bool = False) -> httpx2.Client:
    limits = httpx2.Limits(max_connections=2, max_keepalive_connections=2, keepalive_expiry=30)
    transport = httpx2.HTTPTransport(
        http2=True,
        limits=limits,
        retries=0 if telemetry else 3,
        socket_options=[(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)],
    )
    timeout = (
        httpx2.Timeout(1) if telemetry else httpx2.Timeout(connect=5, read=30, write=10, pool=10)
    )
    return httpx2.Client(transport=transport, timeout=timeout, follow_redirects=True)


@dataclass(frozen=True, slots=True)
class TelemetryHealth:
    generation: int = 0
    acknowledged: float = 0
    source_timestamp: float = 0
    drone_id: str = ""
    error: str | None = "Telemetry has not acknowledged simulator data"


class InferencePublisher:
    def __init__(self, root: Path) -> None:
        self.root = root
        self._lock = threading.Lock()
        self._health = TelemetryHealth()

    def read_state(self) -> SimState:
        state = SimState.model_validate_json((self.root / "state.json").read_bytes())
        state.require_fresh()
        return state

    def _unavailable(self, error: str) -> None:
        atomic_write(
            self.root,
            "inference.json",
            json.dumps(dict(timestamp=time.time(), ok=False, blocked=True, error=error)).encode(),
        )

    def telemetry_failed(self, error: str) -> None:
        with self._lock:
            self._health = TelemetryHealth(generation=self._health.generation + 1, error=error)
            self._unavailable(error)

    def telemetry_succeeded(self, state: SimState) -> None:
        with self._lock:
            state.require_fresh()
            self._health = TelemetryHealth(
                generation=self._health.generation,
                acknowledged=time.monotonic(),
                source_timestamp=state.timestamp,
                drone_id=state.drone_id,
                error=None,
            )

    def inference_failed(self, error: str) -> None:
        with self._lock:
            self._unavailable(error)

    def _require_healthy(self, state: SimState) -> int:
        state.require_fresh()
        health = self._health
        if health.error is not None:
            raise RuntimeError(health.error)
        if time.monotonic() - health.acknowledged > 3 or time.time() - health.source_timestamp > 5:
            raise TimeoutError("Telemetry acknowledgement is stale")
        if health.drone_id != state.drone_id:
            raise RuntimeError("Telemetry drone identity changed")
        return health.generation

    def begin(self, state: SimState) -> int:
        with self._lock:
            return self._require_healthy(state)

    def publish(
        self,
        result: DetectResponse,
        state: SimState,
        generation: int,
        started: float,
        blocked: bool,
    ) -> None:
        with self._lock:
            current = self.read_state()
            if current.drone_id != state.drone_id or result.drone_id != state.drone_id:
                raise RuntimeError("Inference drone identity changed")
            if self._require_healthy(state) != generation:
                raise RuntimeError("Telemetry failed while inference was running")
            atomic_write(
                self.root,
                "inference.json",
                json.dumps(
                    dict(
                        timestamp=started,
                        ok=True,
                        blocked=blocked,
                        model="school_obstacles_best.pt",
                        **result.model_dump(mode="json"),
                    )
                ).encode(),
            )


class TelemetryReporter:
    def __init__(
        self,
        publisher: InferencePublisher,
        backend: str,
        client_factory: Callable[[], httpx2.Client] = lambda: http_client(telemetry=True),
    ) -> None:
        self.publisher = publisher
        self.backend = backend
        self._client_factory = client_factory
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="school-telemetry", daemon=True)

    def __enter__(self) -> TelemetryReporter:
        self._thread.start()
        return self

    def __exit__(self, *_exc: object) -> None:
        self._stop.set()
        self._thread.join(timeout=5)
        if self._thread.is_alive():
            self.publisher.telemetry_failed("Telemetry worker did not stop")
            raise RuntimeError("Telemetry worker did not stop within its HTTP timeout")

    def report_once(self, client: httpx2.Client) -> None:
        try:
            state = self.publisher.read_state()
            assert state.position_enu is not None  # Validated at the simulator boundary.
            x, y, z = state.position_enu
            response = client.post(
                self.backend + "/api/v1/flights/telemetry/" + state.drone_id,
                json=dict(
                    latitude=37 + y / 111320,
                    longitude=127 + x / (111320 * math.cos(math.radians(37))),
                    altitude_m=z,
                    status="flying" if state.armed else "disarmed",
                ),
            )
            response.raise_for_status()
            self.publisher.telemetry_succeeded(state)
        except (OSError, ValueError, httpx2.HTTPError, RuntimeError) as error:
            self.publisher.telemetry_failed(str(error))
            LOG.warning("telemetry unavailable: %s", error)

    def _run(self) -> None:
        try:
            with self._client_factory() as client:
                while not self._stop.is_set():
                    started = time.monotonic()
                    self.report_once(client)
                    self._stop.wait(max(0, 1 - (time.monotonic() - started)))
        except (OSError, ValueError, httpx2.HTTPError, RuntimeError) as error:
            self.publisher.telemetry_failed(str(error))
            LOG.exception("Telemetry worker stopped")
