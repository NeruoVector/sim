"""Disposable PostgreSQL + actual backend API checks for the synthetic school runner."""

import json
import os
import socket
import subprocess
import time
import urllib.error
import urllib.request
import uuid
from datetime import UTC, datetime
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[2]


def check_backend(work: Path, fixture: dict, checker_args: list[str]) -> dict:
    """Create only disposable test data; remove the owned DB container on every exit."""
    if fixture.get("synthetic") is not True:
        raise RuntimeError("backend school checks require a synthetic fixture")
    work = work.resolve()
    name = f"nv-school-{uuid.uuid4().hex}"
    container = None
    cidfile = work / "school-backend.cid"
    backend = None
    log = None

    def run(command, *, text=None, timeout=60):
        result = subprocess.run(
            command,
            input=text,
            text=True,
            capture_output=True,
            cwd=PROJECT,
            timeout=timeout,
            check=False,
        )
        if result.returncode:
            raise RuntimeError(
                f"school command failed ({command[0]}): "
                f"{result.stderr[-3000:]}{result.stdout[-1000:]}"
            )
        return result.stdout.strip()

    try:
        container = run(
            [
                "docker",
                "run",
                "--detach",
                "--pull",
                "never",
                "--name",
                name,
                "--cidfile",
                str(cidfile),
                "--tmpfs",
                "/var/lib/postgresql/data:rw",
                "--publish",
                "127.0.0.1::5432",
                "--env",
                "POSTGRES_USER=school",
                "--env",
                "POSTGRES_DB=school_demo",
                "--env",
                "POSTGRES_PASSWORD=school-dummy-only",
                "pgvector/pgvector:pg16",
            ]
        )
        # Cleanup always targets the ID returned by this invocation, never an existing name.
        if len(container) != 64 or any(c not in "0123456789abcdef" for c in container):
            raise RuntimeError("Docker did not return a container ID")
        deadline = time.monotonic() + 60
        while True:
            ready = subprocess.run(
                [
                    "docker",
                    "exec",
                    container,
                    "pg_isready",
                    "-h",
                    "127.0.0.1",
                    "-U",
                    "school",
                    "-d",
                    "school_demo",
                ],
                capture_output=True,
                timeout=5,
                check=False,
            )
            if ready.returncode == 0:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "disposable school database did not start within 60 seconds"
                )
            time.sleep(0.25)
        mapping = run(["docker", "port", container, "5432/tcp"])
        if not mapping.startswith("127.0.0.1:") or "\n" in mapping:
            raise RuntimeError("school database must bind only to 127.0.0.1")
        db_port = int(mapping.rsplit(":", 1)[1])
        psql = [
            "docker",
            "exec",
            "-i",
            "--env",
            "PGPASSWORD=school-dummy-only",
            container,
            "psql",
            "-h",
            "127.0.0.1",
            "-U",
            "school",
            "-d",
            "school_demo",
            "--no-psqlrc",
            "--set",
            "ON_ERROR_STOP=1",
        ]
        run(
            psql,
            text='CREATE SCHEMA neurovector; CREATE EXTENSION IF NOT EXISTS "uuid-ossp";',
        )
        migrations = sorted((PROJECT / "backend/migrations").glob("*.sql"))
        for migration in migrations:
            run(psql, text=migration.read_text())

        manifest = str(PROJECT / "backend/Cargo.toml")
        run(["cargo", "build", "--offline", "--manifest-path", manifest], timeout=600)
        metadata = json.loads(
            run(
                [
                    "cargo",
                    "metadata",
                    "--offline",
                    "--no-deps",
                    "--format-version",
                    "1",
                    "--manifest-path",
                    manifest,
                ]
            )
        )
        binary = Path(metadata["target_directory"]) / "debug/neurovector-backend"
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            http_port = reservation.getsockname()[1]
        base = f"http://127.0.0.1:{http_port}"
        drone = fixture["drone"]
        # Explicit values prevent inherited deployment settings or a parent .env from leaking in.
        env = {
            "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
            "RUST_LOG": "warn",
            "LISTEN_ADDR": f"127.0.0.1:{http_port}",
            "PUBLIC_BASE_URL": base,
            "DATABASE_URL": f"postgres://school:school-dummy-only@127.0.0.1:{db_port}/school_demo",
            "DATABASE_MAX_CONNECTIONS": "4",
            "REDIS_URL": "redis://127.0.0.1:1/",
            "REDIS_TIMEOUT_MS": "50",
            "CACHE_ENABLED": "false",
            "CACHE_TTL_SECONDS": "1",
            "KAFKA_BROKERS": "127.0.0.1:1",
            "KAFKA_TOPIC_EVENTS": "school-dummy-events",
            "KAFKA_TOPIC_ORDERS": "school-dummy-orders",
            "ADMIN_USERNAME": "school-dummy",
            "ADMIN_PASSWORD": "school-dummy-only",
            "ADMIN_SESSION_TTL_SECONDS": "60",
            "ADMIN_COOKIE_SECURE": "false",
            "CORS_ALLOWED_ORIGINS": base,
            "MQTT_HOST": "127.0.0.1",
            "MQTT_PORT": "1",
            "MQTT_USERNAME": "school-dummy",
            "MQTT_PASSWORD": "school-dummy-only",
            "MQTT_CLIENT_ID": name,
            "MQTT_TOPIC_PREFIX": "school-dummy",
            "MQTT_QOS": "0",
            "MEDIA_API_BASE_URL": "http://127.0.0.1:1",
            "MEDIA_WHEP_BASE_URL": base,
            "MEDIA_SRT_PUBLIC_HOST": "127.0.0.1",
            "MEDIA_SRT_PORT": "1",
            "MEDIA_RECORDINGS_DIR": str(work / "school-recordings"),
            "MAX_ALTITUDE_M": str(drone["max_altitude_m"]),
            "ROUTE_MAX_LEG_M": "5000",
            "ROUTE_MAX_DISTANCE_M": "20000",
            "ROUTE_BATTERY_WH": str(drone["battery_wh"]),
            "ROUTE_POWER_W": str(drone["modeled_power_w"]),
            "RISK_LOOKBACK_DAYS": "90",
            "RISK_CORRIDOR_M": "150",
            "TELEMETRY_ONLINE_TTL_SECONDS": "120",
        }
        log = (work / "school-backend.log").open("w")
        backend = subprocess.Popen(
            [str(binary)], cwd=work, env=env, stdout=log, stderr=subprocess.STDOUT
        )
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

        def api(method, path, body=None, accepted=(200,)):
            request = urllib.request.Request(
                base + path,
                method=method,
                data=json.dumps(body).encode() if body is not None else None,
                headers={"Content-Type": "application/json"},
            )
            try:
                response = opener.open(request, timeout=10)
            except urllib.error.HTTPError as error:
                response = error
            with response:
                status = response.code
                raw = response.read().decode()
            if status not in accepted:
                raise RuntimeError(f"{method} {path}: HTTP {status}: {raw[:3000]}")
            return status, json.loads(raw) if raw else None

        deadline = time.monotonic() + 60
        while True:
            if backend.poll() is not None:
                raise RuntimeError(
                    "isolated backend exited before health; see school-backend.log"
                )
            try:
                api("GET", "/health")
                break
            except (OSError, urllib.error.URLError):
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        "isolated backend did not start within 60 seconds"
                    ) from None
                time.sleep(0.25)

        detour = json.loads((work / "school-detour.json").read_text())["payload"]
        for zone in detour["zones"]:
            api(
                "POST",
                "/api/v1/routes/no-fly-zones",
                {
                    "name": zone["name"],
                    "polygon": zone["polygon"],
                    "max_altitude_m": zone.get("max_altitude_m"),
                    "active": True,
                },
                accepted=(201,),
            )
        outdoor = {
            **detour["candidates"][0],
            "expires_in_seconds": 600,
            "alternatives": [c["waypoints"] for c in detour["candidates"][1:]],
        }
        window = json.loads((work / "window-request.json").read_text())
        reports = []
        denied_status = None
        delivery_id = None
        for label, body in (("outdoor", outdoor), ("window", window)):
            _, plan = api("POST", "/api/v1/routes/plan", body, accepted=(201,))
            (work / f"school-api-{label}.json").write_text(
                json.dumps(plan, indent=2) + "\n"
            )
            if label == "window":
                _, delivery = api(
                    "POST",
                    "/api/v1/deliveries",
                    {
                        "route_id": plan["route_id"],
                        "drone_id": drone["id"],
                        "cargo": "dummy-school",
                        "scheduled_at": datetime.now(UTC).isoformat(),
                        "notes": "Synthetic API test; no motor or physical flight",
                    },
                    accepted=(201,),
                )
                delivery_id = delivery["id"]
                denied_status, _ = api(
                    "PATCH",
                    f"/api/v1/deliveries/{delivery_id}/status",
                    {"status": "dispatched"},
                    accepted=(400, 409),
                )
            checked = json.loads(
                run(
                    checker_args + ["--fingerprint", plan["route_fingerprint"]],
                    text=plan["plan_text"],
                )
            )
            if not checked.get("valid"):
                raise RuntimeError(
                    f"drone checker rejected actual {label} API plan: {checked}"
                )
            _, verified = api(
                "POST", f"/api/v1/routes/plans/{plan['id']}/cross-check", checked
            )
            if verified["status"] != "verified":
                raise RuntimeError(
                    "backend did not persist successful drone verification"
                )
            reports.append(
                {
                    "name": label,
                    "plan_id": plan["id"],
                    "route_id": plan["route_id"],
                    **checked,
                }
            )
        for status in ("dispatched", "in_flight"):
            _, delivery = api(
                "PATCH", f"/api/v1/deliveries/{delivery_id}/status", {"status": status}
            )
            if delivery["status"] != status:
                raise RuntimeError(f"delivery did not transition to {status}")
        return {
            "isolated_database": True,
            "temporary_database_writes": True,
            "production_database_writes": False,
            "schema_migrations": len(migrations),
            "cross_checks": reports,
            "unverified_dispatch_http_status": denied_status,
            "verified_delivery_status": "in_flight",
            "physical_flight": False,
            "window_handoff": "API validation only; no motor commands",
        }
    finally:
        if backend is not None and backend.poll() is None:
            backend.terminate()
            try:
                backend.wait(timeout=10)
            except subprocess.TimeoutExpired:
                backend.kill()
                backend.wait(timeout=5)
        if log is not None:
            log.close()
        if container is None and cidfile.is_file():
            container = cidfile.read_text().strip()
        if (
            container
            and len(container) == 64
            and all(c in "0123456789abcdef" for c in container)
        ):
            subprocess.run(
                ["docker", "rm", "--force", container],
                capture_output=True,
                timeout=30,
                check=True,
            )
            cidfile.unlink(missing_ok=True)
