"""Disposable PostgreSQL + actual backend API checks for the synthetic school runner."""

import hashlib
import json
import os
import socket
import subprocess
import time
import urllib.error
import urllib.request
import uuid
from datetime import UTC, datetime, timedelta
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
                "postgres:16",
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
            if migration.name == "0009_spots_dispatch.sql":
                # Model a deployment that already has the original indoor-only
                # constraints; the upgrade must replace them, not just skip CREATE.
                run(
                    psql,
                    text="""
                        CREATE TABLE neurovector.delivery_spots (
                            id uuid PRIMARY KEY DEFAULT uuid_generate_v4(),
                            customer_id uuid NOT NULL REFERENCES neurovector.customers(id) ON DELETE CASCADE,
                            label text NOT NULL,
                            delivery_address text NOT NULL,
                            delivery_detail text,
                            latitude double precision NOT NULL,
                            longitude double precision NOT NULL,
                            room_number text,
                            grade integer,
                            classroom text,
                            created_at timestamptz NOT NULL DEFAULT now(),
                            updated_at timestamptz NOT NULL DEFAULT now(),
                            CONSTRAINT delivery_spots_legacy_indoor_check CHECK (
                                (room_number IS NOT NULL AND length(btrim(room_number)) BETWEEN 1 AND 40
                                 AND grade IS NULL AND classroom IS NULL)
                                OR (room_number IS NULL AND grade BETWEEN 1 AND 12
                                    AND classroom IS NOT NULL AND length(btrim(classroom)) BETWEEN 1 AND 40)
                            )
                        );
                        ALTER TABLE neurovector.orders
                            ADD COLUMN room_number text,
                            ADD COLUMN grade integer,
                            ADD COLUMN classroom text,
                            ADD CONSTRAINT orders_legacy_indoor_check CHECK (
                                (room_number IS NOT NULL AND length(btrim(room_number)) BETWEEN 1 AND 40
                                 AND grade IS NULL AND classroom IS NULL)
                                OR (room_number IS NULL AND grade BETWEEN 1 AND 12
                                    AND classroom IS NOT NULL AND length(btrim(classroom)) BETWEEN 1 AND 40)
                            );
                    """,
                )
            run(psql, text=migration.read_text())

        drone = fixture["drone"]
        origin = fixture["origin"]
        run(
            psql,
            text=f"""
                INSERT INTO neurovector.delivery_fleet
                    (drone_id, capacity_grams, home_latitude, home_longitude,
                     cruise_speed_mps, max_altitude_m)
                VALUES ('{drone['id']}', 100000, {origin['lat']}, {origin['lon']},
                        {drone['cruise_speed_mps']}, {drone['max_altitude_m']});
            """,
        )

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
        # Explicit values prevent inherited deployment settings or a parent .env from leaking in.
        env = {
            "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
            "RUST_LOG": os.environ.get("SCHOOL_BACKEND_RUST_LOG", "warn"),
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
            # This disposable scenario uses the declared synthetic fixture home as its altitude origin.
            "AUTOMATIC_PLAN_ALTITUDE_REFERENCE": "home_relative",
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

        def api(method, path, body=None, accepted=(200,), headers=None):
            request_headers = {"Content-Type": "application/json"}
            if headers is not None:
                request_headers.update(headers)
            request = urllib.request.Request(
                base + path,
                method=method,
                data=json.dumps(body).encode() if body is not None else None,
                headers=request_headers,
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

        def schedule_weight_scenarios():
            prefix = f"SCH-{uuid.uuid4().hex[:12]}"
            owner_id = str(uuid.uuid4())
            merchant_id = str(uuid.uuid4())
            category_id = str(uuid.uuid4())
            regular_item_id = str(uuid.uuid4())
            overweight_item_id = str(uuid.uuid4())
            unweighed_item_id = str(uuid.uuid4())
            ready_order_ids = [str(uuid.uuid4()) for _ in range(4)]
            overweight_order_id = str(uuid.uuid4())
            unweighed_order_id = str(uuid.uuid4())

            login_request = urllib.request.Request(
                base + "/api/v1/admin/auth/login",
                method="POST",
                data=json.dumps(
                    {"username": "school-dummy", "password": "school-dummy-only"}
                ).encode(),
                headers={"Content-Type": "application/json"},
            )
            with opener.open(login_request, timeout=10) as response:
                admin_cookie = response.headers.get("Set-Cookie", "").split(";", 1)[0]
            if not admin_cookie.startswith("nv_admin_session="):
                raise RuntimeError("admin login did not return an authenticated session cookie")
            admin_headers = {"Cookie": admin_cookie}

            api(
                "POST",
                "/api/v1/admin/fleet",
                {
                    "drone_id": f"{prefix}-drone-a",
                    "capacity_grams": 300,
                    "home_latitude": 37.5,
                    "home_longitude": 127.0,
                    "cruise_speed_mps": 8.0,
                    "max_altitude_m": 30.0,
                    "active": True,
                },
                accepted=(200,),
                headers=admin_headers,
            )
            api(
                "POST",
                "/api/v1/admin/fleet",
                {
                    "drone_id": f"{prefix}-drone-b",
                    "capacity_grams": 300,
                    "home_latitude": 37.5,
                    "home_longitude": 127.0,
                    "cruise_speed_mps": 8.0,
                    "max_altitude_m": 30.0,
                    "active": True,
                },
                accepted=(200,),
                headers=admin_headers,
            )
            api(
                "POST",
                "/api/v1/admin/fleet",
                {
                    "drone_id": f"{prefix}-unauthenticated-check",
                    "capacity_grams": 300,
                    "home_latitude": 37.5,
                    "home_longitude": 127.0,
                    "cruise_speed_mps": 8.0,
                    "max_altitude_m": 30.0,
                    "active": True,
                },
                accepted=(401,),
            )

            statements = [
                f"INSERT INTO neurovector.customers (id, provider, subject, name) VALUES ('{owner_id}', 'google', '{prefix}', 'Synthetic Scheduler Owner');",
                f"INSERT INTO neurovector.merchants (id, owner_id, name) VALUES ('{merchant_id}', '{owner_id}', 'Synthetic Scheduler Merchant');",
                f"INSERT INTO neurovector.product_categories (id, merchant_id, name) VALUES ('{category_id}', '{merchant_id}', 'Synthetic Delivery');",
                f"INSERT INTO neurovector.inventory_items (id, sku, name, unit_price, stock_quantity) VALUES ('{regular_item_id}', '{prefix}-REGULAR', 'Synthetic 150g product', 1000, 100), ('{overweight_item_id}', '{prefix}-HEAVY', 'Synthetic 400g product', 1000, 100), ('{unweighed_item_id}', '{prefix}-UNKNOWN', 'Synthetic unweighed product', 1000, 100);",
                f"INSERT INTO neurovector.product_details (item_id, merchant_id, category_id, weight_grams) VALUES ('{regular_item_id}', '{merchant_id}', '{category_id}', 150), ('{overweight_item_id}', '{merchant_id}', '{category_id}', 400);",
            ]
            all_order_ids = ready_order_ids + [overweight_order_id, unweighed_order_id]
            for index, order_id in enumerate(ready_order_ids):
                statements.append(
                    f"INSERT INTO neurovector.orders (id, order_number, customer_name, customer_phone, status, total_amount, currency, delivery_address, latitude, longitude) VALUES ('{order_id}', '{prefix}-READY-{index}', 'Synthetic customer', '010-0000-0000', 'ready', 1000, 'KRW', 'Synthetic school', 37.501{index}, 127.001{index});"
                )
                statements.append(
                    f"INSERT INTO neurovector.order_items (order_id, item_id, quantity, unit_price, line_total, weight_grams) VALUES ('{order_id}', '{regular_item_id}', 1, 1000, 1000, 150);"
                )
            statements.extend(
                [
                    f"INSERT INTO neurovector.orders (id, order_number, customer_name, customer_phone, status, total_amount, currency, delivery_address, latitude, longitude) VALUES ('{overweight_order_id}', '{prefix}-OVERWEIGHT', 'Synthetic customer', '010-0000-0000', 'ready', 1000, 'KRW', 'Synthetic school', 37.505, 127.005);",
                    f"INSERT INTO neurovector.order_items (order_id, item_id, quantity, unit_price, line_total, weight_grams) VALUES ('{overweight_order_id}', '{overweight_item_id}', 1, 1000, 1000, 400);",
                    f"INSERT INTO neurovector.orders (id, order_number, customer_name, customer_phone, status, total_amount, currency, delivery_address, latitude, longitude) VALUES ('{unweighed_order_id}', '{prefix}-UNWEIGHED', 'Synthetic customer', '010-0000-0000', 'ready', 1000, 'KRW', 'Synthetic school', 37.506, 127.006);",
                    f"INSERT INTO neurovector.order_items (order_id, item_id, quantity, unit_price, line_total, weight_grams) VALUES ('{unweighed_order_id}', '{unweighed_item_id}', 1, 1000, 1000, NULL);",
                    f"UPDATE neurovector.product_details SET weight_grams = 999 WHERE item_id = '{regular_item_id}';",
                    f"UPDATE neurovector.product_details SET weight_grams = 100 WHERE item_id = '{overweight_item_id}';",
                    f"INSERT INTO neurovector.delivery_spots (customer_id, label, delivery_address, latitude, longitude) VALUES ('{owner_id}', '{prefix}-OUTDOOR', 'Synthetic outdoor location', 37.5, 127.0);",
                ]
            )
            run(psql, text="\n".join(statements))
            for fleet_drone in (f"{prefix}-drone-a", f"{prefix}-drone-b"):
                api("POST", f"/api/v1/flights/telemetry/{fleet_drone}", {"status": "idle"}, accepted=(201,))
            try:
                run(
                    psql,
                    text=f"""
                        INSERT INTO neurovector.delivery_spots
                            (customer_id, label, delivery_address, latitude, longitude,
                             room_number, grade, classroom)
                        VALUES ('{owner_id}', '{prefix}-AMBIGUOUS', 'Invalid indoor location',
                                37.5, 127.0, '101', 4, '4-2');
                    """,
                )
            except RuntimeError as error:
                if "delivery_spots" not in str(error):
                    raise
            else:
                raise RuntimeError("Spot schema accepted ambiguous indoor fields")

            deadline = time.monotonic() + 25
            scenario = None
            while time.monotonic() < deadline:
                result = run(
                    [*psql, "--tuples-only", "--no-align"],
                    text=f"""
                        SELECT json_build_object(
                            'orders', (
                                SELECT json_agg(json_build_object(
                                    'id', id,
                                    'order_number', order_number,
                                    'status', status,
                                    'delivery_id', delivery_id
                                ) ORDER BY order_number)
                                FROM neurovector.orders WHERE order_number LIKE '{prefix}%'
                            ),
                            'deliveries', (
                                SELECT json_agg(json_build_object(
                                    'id', d.id,
                                    'drone_id', d.drone_id,
                                    'status', d.status,
                                    'estimated_weight_grams', d.estimated_weight_grams,
                                    'stop_count', (SELECT count(*) FROM neurovector.delivery_stops s WHERE s.delivery_id=d.id)
                                ) ORDER BY d.drone_id)
                                FROM neurovector.deliveries d
                                WHERE d.schedule_source='automatic'
                                  AND d.drone_id LIKE '{prefix}-drone-%'
                            ),
                            'blocked_reasons', (
                                SELECT json_agg(e.payload->>'reason' ORDER BY o.order_number)
                                FROM neurovector.order_tracking_events e
                                JOIN neurovector.orders o ON o.id=e.order_id
                                WHERE o.order_number LIKE '{prefix}%'
                                  AND e.event_type='schedule_blocked'
                            )
                        )::text;
                    """,
                )
                if result:
                    scenario = json.loads(result)
                    orders = scenario.get("orders") or []
                    if len(orders) == 6 and sum(order["delivery_id"] is not None for order in orders) == 4:
                        break
                time.sleep(0.25)
            if scenario is None:
                raise RuntimeError("automatic delivery scheduler did not return scenario data")
            orders = scenario.get("orders") or []
            if len(orders) != 6 or sum(order["delivery_id"] is not None for order in orders) != 4:
                raise RuntimeError(f"scheduler failed to assign eligible weights: {scenario}")
            if any(order["status"] != "assigned" for order in orders if order["delivery_id"]):
                raise RuntimeError(f"ready orders did not transition to assigned: {scenario}")
            if any(order["delivery_id"] is not None for order in orders if "READY" not in order["order_number"]):
                raise RuntimeError(f"overweight or unweighed order was assigned: {scenario}")
            deliveries = scenario.get("deliveries") or []
            if len(deliveries) != 2 or any(
                delivery["status"] != "scheduled"
                or delivery["estimated_weight_grams"] != 300
                or delivery["stop_count"] != 2
                for delivery in deliveries
            ):
                raise RuntimeError(f"scheduler did not pack two concurrent multi-stop batches: {scenario}")
            if set(scenario.get("blocked_reasons") or []) != {
                "payload_exceeds_fleet_capacity",
                "product_weight_missing",
            }:
                raise RuntimeError(f"scheduler did not report weight blockers: {scenario}")

            stale_drone_id = f"{prefix}-stale-only"
            stale_order_id = str(uuid.uuid4())
            stale_order_number = f"{prefix}-STALE-FLEET"
            api(
                "POST",
                "/api/v1/admin/fleet",
                {
                    "drone_id": stale_drone_id,
                    "capacity_grams": 1000,
                    "home_latitude": 37.5,
                    "home_longitude": 127.0,
                    "cruise_speed_mps": 8.0,
                    "max_altitude_m": 30.0,
                    "active": True,
                },
                accepted=(200,),
                headers=admin_headers,
            )
            run(
                psql,
                text=f"""
                    UPDATE neurovector.delivery_fleet
                    SET active=false
                    WHERE drone_id IN ('{prefix}-drone-a', '{prefix}-drone-b');
                    INSERT INTO neurovector.orders
                        (id, order_number, customer_name, customer_phone, status,
                         total_amount, currency, delivery_address, latitude, longitude)
                    VALUES ('{stale_order_id}', '{stale_order_number}', 'Synthetic customer',
                            '010-0000-0000', 'ready', 1000, 'KRW', 'Synthetic school',
                            37.507, 127.007);
                    INSERT INTO neurovector.order_items
                        (order_id, item_id, quantity, unit_price, line_total, weight_grams)
                    VALUES ('{stale_order_id}', '{regular_item_id}', 1, 1000, 1000, 100);
                """,
            )
            api("POST", f"/api/v1/flights/telemetry/{stale_drone_id}", {"ts": (datetime.now(UTC) - timedelta(hours=1)).isoformat(), "status": "idle"}, accepted=(201,))
            all_order_ids.append(stale_order_id)

            def stale_order_state() -> dict:
                result = run(
                    [*psql, "--tuples-only", "--no-align"],
                    text=f"""
                        SELECT json_build_object(
                            'delivery_id', (SELECT delivery_id FROM neurovector.orders WHERE id='{stale_order_id}'),
                            'status', (SELECT status FROM neurovector.orders WHERE id='{stale_order_id}'),
                            'stop_count', (SELECT count(*) FROM neurovector.delivery_stops WHERE order_id='{stale_order_id}'),
                            'blocked_offline', EXISTS (
                                SELECT 1 FROM neurovector.order_tracking_events
                                WHERE order_id='{stale_order_id}' AND event_type='schedule_blocked'
                                  AND payload->>'reason'='no_active_drone'
                            )
                        )::text;
                    """,
                )
                return json.loads(result)

            stale_deadline = time.monotonic() + 15
            offline_state = stale_order_state()
            while time.monotonic() < stale_deadline and not offline_state["blocked_offline"]:
                time.sleep(0.25)
                offline_state = stale_order_state()
            if offline_state != {
                "delivery_id": None,
                "status": "ready",
                "stop_count": 0,
                "blocked_offline": True,
            }:
                raise RuntimeError(f"stale-only fleet drone was selected for a valid-weight order: {offline_state}")

            future_device_ts = datetime.now(UTC) + timedelta(hours=1)
            api(
                "POST",
                f"/api/v1/flights/telemetry/{stale_drone_id}",
                {
                    "ts": future_device_ts.isoformat(),
                    "latitude": 37.5,
                    "longitude": 127.0,
                    "status": "idle",
                },
                accepted=(201,),
            )
            _, future_status = api(
                "GET", f"/api/v1/flights/status/{stale_drone_id}", accepted=(200,)
            )
            if future_status["online"] is not False:
                raise RuntimeError(
                    f"future device timestamp was reported online: {future_status}"
                )
            if stale_order_state() != offline_state:
                raise RuntimeError("future device telemetry changed stale-only order assignment")

            api(
                "POST",
                f"/api/v1/flights/telemetry/{stale_drone_id}",
                {
                    "ts": datetime.now(UTC).isoformat(),
                    "latitude": 37.5,
                    "longitude": 127.0,
                    "status": "idle",
                },
                accepted=(201,),
            )
            fresh_deadline = time.monotonic() + 15
            fresh_state = stale_order_state()
            while time.monotonic() < fresh_deadline and fresh_state["delivery_id"] is None:
                time.sleep(0.25)
                fresh_state = stale_order_state()
            if (
                fresh_state["delivery_id"] is None
                or fresh_state["status"] != "assigned"
                or fresh_state["stop_count"] != 1
            ):
                raise RuntimeError(f"fresh telemetry did not restore fleet scheduling eligibility: {fresh_state}")
            run(
                psql,
                text=f"UPDATE neurovector.delivery_fleet SET active=true WHERE drone_id IN ('{prefix}-drone-a', '{prefix}-drone-b');",
            )

            # Keep the simulator's fleet online with trusted, current synthetic samples.
            # These samples exercise the dispatch gate; they do not command a physical drone.
            for scheduled in deliveries:
                api("POST", f"/api/v1/flights/telemetry/{scheduled['drone_id']}", {"status": "idle"}, accepted=(201,))

            delivery = deliveries[0]
            drone_id = delivery["drone_id"]
            def expect_dispatch_blocked(expected: str) -> None:
                try:
                    run(
                        [*psql, "--tuples-only", "--no-align"],
                        text=f"UPDATE neurovector.deliveries SET status='dispatched' WHERE id='{delivery['id']}';",
                    )
                except RuntimeError as error:
                    if expected not in str(error):
                        raise
                else:
                    raise RuntimeError(
                        f"database allowed dispatch without expected gate: {expected}"
                    )

            api("POST", f"/api/v1/flights/telemetry/{drone_id}", {"ts": (datetime.now(UTC) + timedelta(seconds=1)).isoformat(), "status": "failed"}, accepted=(201,))
            expect_dispatch_blocked("requires fresh live telemetry")
            api("POST", f"/api/v1/flights/telemetry/{drone_id}", {"ts": (datetime.now(UTC) + timedelta(seconds=2)).isoformat(), "status": "idle"}, accepted=(201,))
            expect_dispatch_blocked("requires a recent positive measured payload")
            run(
                psql,
                text=f"INSERT INTO neurovector.delivery_weight_measurements (delivery_id, drone_id, weight_grams, capacity_exceeded, measured_by, measured_at) VALUES ('{delivery['id']}', '{drone_id}', 250, false, 'school-test', now() - interval '10 minutes');",
            )
            expect_dispatch_blocked("requires a recent positive measured payload")
            _, zero_measurement = api(
                "POST",
                f"/api/v1/admin/fleet/{drone_id}/payload-measurements",
                {"delivery_id": delivery["id"], "weight_grams": 0},
                accepted=(201,),
                headers=admin_headers,
            )
            expect_dispatch_blocked("requires a recent positive measured payload")
            measured_status, measurement = api(
                "POST",
                f"/api/v1/admin/fleet/{drone_id}/payload-measurements",
                {"delivery_id": delivery["id"], "weight_grams": 400},
                accepted=(201,),
                headers=admin_headers,
            )
            if (
                measured_status != 201
                or measurement["estimated_delivery_weight_grams"] != 300
                or measurement["weight_grams"] != 400
                or measurement["capacity_exceeded"] is not True
            ):
                raise RuntimeError(f"actual payload measurement was not kept distinct: {measurement}")
            expect_dispatch_blocked("latest measured payload exceeds current drone capacity")
            _, safe_reweigh = api(
                "POST",
                f"/api/v1/admin/fleet/{drone_id}/payload-measurements",
                {"delivery_id": delivery["id"], "weight_grams": 250},
                accepted=(201,),
                headers=admin_headers,
            )
            run(
                psql,
                text=f"UPDATE neurovector.delivery_fleet SET capacity_grams=200 WHERE drone_id='{drone_id}';",
            )
            expect_dispatch_blocked("latest measured payload exceeds current drone capacity")
            run(
                psql,
                text=f"UPDATE neurovector.delivery_fleet SET capacity_grams=300 WHERE drone_id='{drone_id}'; UPDATE neurovector.deliveries SET status='dispatched' WHERE id='{delivery['id']}';",
            )

            rollback_order_id = str(uuid.uuid4())
            rollback_order_number = f"{prefix}-ROLLBACK"
            run(
                psql,
                text=f"""
                    CREATE FUNCTION neurovector.school_fail_assignment_event() RETURNS trigger
                    LANGUAGE plpgsql AS $$
                    BEGIN
                        IF NEW.order_id = '{rollback_order_id}'::uuid
                           AND NEW.event_type = 'delivery_assigned' THEN
                            RAISE EXCEPTION 'school-injected-assignment-event-failure';
                        END IF;
                        RETURN NEW;
                    END $$;
                    CREATE TRIGGER school_fail_assignment_event
                        BEFORE INSERT ON neurovector.order_tracking_events
                        FOR EACH ROW EXECUTE FUNCTION neurovector.school_fail_assignment_event();
                    INSERT INTO neurovector.orders
                        (id, order_number, customer_name, customer_phone, status,
                         total_amount, currency, delivery_address, latitude, longitude)
                    VALUES ('{rollback_order_id}', '{rollback_order_number}',
                            'Synthetic retry customer', '010-0000-0000', 'ready',
                            1000, 'KRW', 'Synthetic school', 37.507, 127.007);
                    INSERT INTO neurovector.order_items
                        (order_id, item_id, quantity, unit_price, line_total, weight_grams)
                    VALUES ('{rollback_order_id}', '{regular_item_id}', 1, 1000, 1000, 100);
                """,
            )

            failure_deadline = time.monotonic() + 18
            failure_seen = False
            backend_log = work / "school-backend.log"
            while time.monotonic() < failure_deadline:
                if backend_log.is_file() and "school-injected-assignment-event-failure" in backend_log.read_text():
                    failure_seen = True
                    break
                time.sleep(0.25)
            if not failure_seen:
                raise RuntimeError("scheduler did not exercise the injected assignment-event failure")

            def rollback_state() -> dict:
                result = run(
                    [*psql, "--tuples-only", "--no-align"],
                    text=f"""
                        SELECT json_build_object(
                            'delivery_id', (SELECT delivery_id FROM neurovector.orders WHERE id='{rollback_order_id}'),
                            'status', (SELECT status FROM neurovector.orders WHERE id='{rollback_order_id}'),
                            'stops', (SELECT count(*) FROM neurovector.delivery_stops WHERE order_id='{rollback_order_id}'),
                            'assignment_events', (SELECT count(*) FROM neurovector.order_tracking_events WHERE order_id='{rollback_order_id}' AND event_type='delivery_assigned')
                        )::text;
                    """,
                )
                return json.loads(result)

            failed_state = rollback_state()
            if failed_state != {
                "delivery_id": None,
                "status": "ready",
                "stops": 0,
                "assignment_events": 0,
            }:
                raise RuntimeError(f"failed assignment left partial transaction state: {failed_state}")
            run(
                psql,
                text="DROP TRIGGER school_fail_assignment_event ON neurovector.order_tracking_events; DROP FUNCTION neurovector.school_fail_assignment_event();",
            )

            retry_deadline = time.monotonic() + 15
            retried_state = failed_state
            while time.monotonic() < retry_deadline:
                retried_state = rollback_state()
                if (
                    retried_state["delivery_id"] is not None
                    and retried_state["status"] == "assigned"
                    and retried_state["stops"] == 1
                    and retried_state["assignment_events"] == 1
                ):
                    break
                time.sleep(0.25)
            if not (
                retried_state["delivery_id"] is not None
                and retried_state["status"] == "assigned"
                and retried_state["stops"] == 1
                and retried_state["assignment_events"] == 1
            ):
                raise RuntimeError(f"scheduler retry did not assign exactly once: {retried_state}")

            released_delivery = next(
                item for item in deliveries if item["id"] != delivery["id"]
            )
            released_orders = [
                order
                for order in orders
                if order["delivery_id"] == released_delivery["id"]
            ]
            cancelled_order = released_orders[0]
            _, cancelled = api(
                "PATCH",
                f"/api/v1/orders/{cancelled_order['id']}/status",
                {"status": "cancelled"},
                accepted=(200,),
            )
            if cancelled["status"] != "cancelled":
                raise RuntimeError(f"assigned order did not cancel: {cancelled}")
            sibling = next(
                order
                for order in released_orders
                if order["id"] != cancelled_order["id"]
            )
            deadline = time.monotonic() + 12
            released = None
            while time.monotonic() < deadline:
                result = run(
                    [*psql, "--tuples-only", "--no-align"],
                    text=f"""
                        SELECT json_build_object(
                            'old_delivery_status', (SELECT status FROM neurovector.deliveries WHERE id='{released_delivery['id']}'),
                            'released_event', EXISTS (
                                SELECT 1 FROM neurovector.order_tracking_events
                                WHERE order_id='{sibling['id']}' AND event_type='delivery_released'
                            ),
                            'sibling_status', (SELECT status FROM neurovector.orders WHERE id='{sibling['id']}'),
                            'sibling_delivery_id', (SELECT delivery_id FROM neurovector.orders WHERE id='{sibling['id']}')
                        )::text;
                    """,
                )
                if result:
                    released = json.loads(result)
                    if (
                        released["old_delivery_status"] == "cancelled"
                        and released["released_event"]
                        and released["sibling_delivery_id"] not in (None, released_delivery["id"])
                    ):
                        break
                time.sleep(0.25)
            if released is None or released["sibling_delivery_id"] in (
                None,
                released_delivery["id"],
            ):
                raise RuntimeError(f"cancelled schedule capacity was not released: {released}")

            # Exercise recipient PIN handoff through the actual authenticated APIs. The
            # isolated database session is a synthetic customer cookie session; checkout,
            # payment, scheduling, telemetry and handoff use the application APIs.
            pin_prefix = f"PIN-{uuid.uuid4().hex[:12]}"
            pin_customer_id = str(uuid.uuid4())
            pin_session = uuid.uuid4().hex + uuid.uuid4().hex
            pin_session_hash = hashlib.sha256(pin_session.encode()).hexdigest()
            pin_drone_id = f"{pin_prefix}-drone"
            origin = fixture["origin"]
            selected_candidate = detour["candidates"][detour["selected_index"]]
            destination = selected_candidate["waypoints"][-1]
            run(
                psql,
                text=f"""
                    UPDATE neurovector.delivery_fleet SET active=false
                    WHERE drone_id IN ('{prefix}-drone-a', '{prefix}-drone-b', '{stale_drone_id}');
                    UPDATE neurovector.product_details SET weight_grams=100
                    WHERE item_id='{regular_item_id}';
                    INSERT INTO neurovector.customers (id, provider, subject, name)
                    VALUES ('{pin_customer_id}', 'google', '{pin_prefix}', 'Synthetic PIN Recipient');
                    INSERT INTO neurovector.customer_sessions (token_hash, customer_id, expires_at)
                    VALUES ('{pin_session_hash}', '{pin_customer_id}', now() + interval '1 hour');
                """,
            )
            pin_cookie = f"nv_customer={pin_session}"
            api(
                "POST",
                "/api/v1/admin/fleet",
                {
                    "drone_id": pin_drone_id,
                    "capacity_grams": 500,
                    "home_latitude": origin["lat"],
                    "home_longitude": origin["lon"],
                    "cruise_speed_mps": 3.0,
                    "max_altitude_m": 120.0,
                    "active": True,
                },
                accepted=(200,),
                headers=admin_headers,
            )
            api(
                "POST",
                f"/api/v1/flights/telemetry/{pin_drone_id}",
                {
                    "ts": datetime.now(UTC).isoformat(),
                    "latitude": origin["lat"],
                    "longitude": origin["lon"],
                    "speed_mps": 0.0,
                    "status": "idle",
                },
                accepted=(201,),
            )
            pin_order_ids = []
            pin_spot_by_order = {}
            for index in range(3):
                _, pin_spot = api(
                    "POST",
                    "/api/v1/account/spots",
                    {
                        "label": f"{pin_prefix}-outdoor-{index}",
                        "delivery_address": "Synthetic outdoor pickup point",
                        "latitude": destination["lat"],
                        "longitude": destination["lon"],
                    },
                    accepted=(201,),
                    headers={"Cookie": pin_cookie},
                )
                _, pin_order = api(
                    "POST",
                    "/api/v1/orders",
                    {
                        "customer_name": "Synthetic PIN Recipient",
                        "customer_phone": f"010-9000-{index:04d}",
                        "items": [{"item_id": regular_item_id, "quantity": 2 if index == 1 else 1}],
                        "delivery_address": pin_spot["delivery_address"],
                        "latitude": pin_spot["latitude"],
                        "longitude": pin_spot["longitude"],
                        "spot_id": pin_spot["id"],
                        "handoff_pin": "042731",
                    },
                    accepted=(201,),
                    headers={"Cookie": pin_cookie},
                )
                pin_order_ids.append(pin_order["id"])
                pin_spot_by_order[pin_order["id"]] = pin_spot
            run(
                psql,
                text=f"UPDATE neurovector.product_details SET weight_grams=999 WHERE item_id='{regular_item_id}';",
            )
            snapshot_weights = run(
                [*psql, "--tuples-only", "--no-align"],
                text=f"""
                    SELECT string_agg(weight_grams::text, ',' ORDER BY order_id)
                    FROM neurovector.order_items
                    WHERE order_id IN ('{pin_order_ids[0]}', '{pin_order_ids[1]}', '{pin_order_ids[2]}');
                """,
            )
            if snapshot_weights != "100,100,100":
                raise RuntimeError(
                    f"PIN checkout did not snapshot product weights: {snapshot_weights}"
                )
            pin_snapshot_total = run(
                [*psql, "--tuples-only", "--no-align"],
                text=f"""
                    SELECT sum(weight_grams::bigint * quantity::bigint)::text
                    FROM neurovector.order_items
                    WHERE order_id IN ('{pin_order_ids[0]}', '{pin_order_ids[1]}', '{pin_order_ids[2]}');
                """,
            )
            if pin_snapshot_total != "400":
                raise RuntimeError(
                    f"checkout snapshot weights were not multiplied by quantity: {pin_snapshot_total}"
                )
            for index, order_id in enumerate(pin_order_ids):
                api(
                    "POST", f"/api/v1/orders/{order_id}/payments",
                    {"simulate": "success"}, accepted=(201,), headers={"Cookie": pin_cookie},
                )
                if index != 1:
                    for status in ("preparing", "ready"):
                        api("PATCH", f"/api/v1/orders/{order_id}/status", {"status": status}, accepted=(200,), headers=admin_headers)


            assignment = None
            assignment_deadline = time.monotonic() + 25
            while time.monotonic() < assignment_deadline:
                result = run(
                    [*psql, "--tuples-only", "--no-align"],
                    text=f"""
                        SELECT json_build_object(
                            'delivery_count', count(DISTINCT delivery_id),
                            'assigned_count', count(*) FILTER (WHERE delivery_id IS NOT NULL),
                            'delivery_id', min(o.delivery_id::text),
                            'drone_id', min(d.drone_id),
                            'stop_count', (SELECT count(*) FROM neurovector.delivery_stops s
                                           WHERE s.delivery_id IN (
                                               SELECT assigned_order.delivery_id
                                               FROM neurovector.orders assigned_order
                                               WHERE assigned_order.id IN ('{pin_order_ids[0]}', '{pin_order_ids[1]}', '{pin_order_ids[2]}')
                                           ))
                        )::text
                        FROM neurovector.orders o
                        LEFT JOIN neurovector.deliveries d ON d.id=o.delivery_id
                        WHERE o.id IN ('{pin_order_ids[0]}', '{pin_order_ids[1]}', '{pin_order_ids[2]}');
                    """,
                )
                if result:
                    assignment = json.loads(result)
                    if assignment["assigned_count"] == 3:
                        break
                time.sleep(0.25)
            if (
                not assignment
                or assignment["delivery_count"] != 1
                or assignment["assigned_count"] != 3
                or assignment["drone_id"] != pin_drone_id
                or assignment["stop_count"] != 3
            ):
                raise RuntimeError(
                    f"PIN orders were not planned as one three-stop delivery: {assignment}"
                )
            pin_delivery_id = assignment["delivery_id"]
            pin_planned_weight = run(
                [*psql, "--tuples-only", "--no-align"],
                text=f"SELECT estimated_weight_grams FROM neurovector.deliveries WHERE id='{pin_delivery_id}';",
            )
            if pin_planned_weight != "400":
                raise RuntimeError(
                    f"automatic scheduler did not use immutable quantity-weight snapshots: {pin_planned_weight}"
                )
            first_pin_order_id, second_pin_order_id, third_pin_order_id = run(
                [*psql, "--tuples-only", "--no-align"],
                text=f"""
                    SELECT string_agg(order_id::text, ',' ORDER BY stop_sequence)
                    FROM neurovector.delivery_stops WHERE delivery_id='{pin_delivery_id}';
                """,
            ).split(",")

            _, pin_delivery = api("GET", f"/api/v1/deliveries/{pin_delivery_id}", accepted=(200,))
            _, pin_plan = api("GET", f"/api/v1/routes/plans/by-route/{pin_delivery['route_id']}", accepted=(200,))
            if pin_plan["route_id"] != pin_delivery["route_id"] or pin_plan["status"] != "planned":
                raise RuntimeError("automatic delivery did not retain its own pending mission")
            pin_checker_args = list(checker_args)
            pin_checker_args[pin_checker_args.index("--drone-id") + 1] = pin_drone_id
            checked_pin_plan = json.loads(
                run(
                    pin_checker_args + ["--fingerprint", pin_plan["route_fingerprint"]],
                    text=pin_plan["plan_text"],
                )
            )
            if not checked_pin_plan.get("valid"):
                raise RuntimeError(f"synthetic PIN mission did not validate: {checked_pin_plan}")
            _, verified_pin_plan = api(
                "POST",
                f"/api/v1/routes/plans/{pin_plan['id']}/cross-check",
                checked_pin_plan,
                accepted=(200,),
            )
            if verified_pin_plan.get("status") != "verified":
                raise RuntimeError("PIN scenario mission did not pass the backend cross-check")
            pin_required = verified_pin_plan["payload"]["required_stops"][1:-1]
            if (verified_pin_plan["payload"].get("altitude_reference") != "home_relative"
                    or len(pin_required) != 3):
                raise RuntimeError("synthetic PIN stops differ from the verified home-relative plan")
            pin_target_by_order = dict(zip(
                (first_pin_order_id, second_pin_order_id, third_pin_order_id), pin_required, strict=True
            ))
            if any(abs(point["lat"] - pin_spot_by_order[order_id]["latitude"]) > 1e-8
                   or abs(point["lon"] - pin_spot_by_order[order_id]["longitude"]) > 1e-8
                   for order_id, point in pin_target_by_order.items()):
                raise RuntimeError("synthetic PIN order location differs from the verified stop")
            _, pin_measurement = api(
                "POST",
                f"/api/v1/admin/fleet/{pin_drone_id}/payload-measurements",
                {"delivery_id": pin_delivery_id, "weight_grams": 400},
                accepted=(201,),
                headers=admin_headers,
            )
            if pin_measurement["weight_grams"] != 400 or pin_measurement["capacity_exceeded"]:
                raise RuntimeError(f"PIN scenario payload measurement was invalid: {pin_measurement}")
            pending_order_status = run(
                [*psql, "--tuples-only", "--no-align"],
                text=f"SELECT status FROM neurovector.orders WHERE id='{pin_order_ids[1]}';",
            )
            if pending_order_status != "paid":
                raise RuntimeError(
                    f"paid order was unexpectedly promoted during scheduling: {pending_order_status}"
                )
            early_dispatch_status, _ = api(
                "PATCH",
                f"/api/v1/deliveries/{pin_delivery_id}/status",
                {"status": "dispatched"},
                accepted=(409,),
            )
            if early_dispatch_status != 409:
                raise RuntimeError(
                    f"dispatch with a paid scheduled stop returned HTTP {early_dispatch_status}, expected 409"
                )
            api(
                "PATCH",
                f"/api/v1/orders/{pin_order_ids[1]}/status",
                {"status": "preparing"},
                accepted=(200,),
                headers=admin_headers,
            )
            api(
                "PATCH",
                f"/api/v1/orders/{pin_order_ids[1]}/status",
                {"status": "ready"},
                accepted=(200,),
                headers=admin_headers,
            )
            ready_deadline = time.monotonic() + 15
            ready_order_status = "ready"
            while time.monotonic() < ready_deadline and ready_order_status != "assigned":
                ready_order_status = run(
                    [*psql, "--tuples-only", "--no-align"],
                    text=f"SELECT status FROM neurovector.orders WHERE id='{pin_order_ids[1]}';",
                )
                if ready_order_status != "assigned":
                    time.sleep(0.25)
            if ready_order_status != "assigned":
                raise RuntimeError(
                    f"scheduler did not sync merchant-ready order to assigned: {ready_order_status}"
                )
            api(
                "PATCH",
                f"/api/v1/deliveries/{pin_delivery_id}/status",
                {"status": "dispatched"},
                accepted=(200,),
            )
            api(
                "PATCH",
                f"/api/v1/deliveries/{pin_delivery_id}/status",
                {"status": "in_flight"},
                accepted=(200,),
            )
            for order_id in (first_pin_order_id, third_pin_order_id):
                api("PATCH", f"/api/v1/orders/{order_id}/status", {"status": "in_transit"}, accepted=(200,), headers=admin_headers)


            _, before_arrival = api(
                "GET",
                f"/api/v1/account/orders/{first_pin_order_id}/tracking",
                accepted=(200,),
                headers={"Cookie": pin_cookie},
            )
            before_arrival_stop = run(
                [*psql, "--tuples-only", "--no-align"],
                text=f"SELECT status FROM neurovector.delivery_stops WHERE order_id='{first_pin_order_id}';",
            )
            if (
                before_arrival.get("order_status") != "in_transit"
                or before_arrival.get("delivery_status") != "in_flight"
                or not before_arrival.get("eta_at")
                or before_arrival_stop != "scheduled"
            ):
                raise RuntimeError(f"PIN tracking did not show scheduled stop ETA: {before_arrival}")
            early_handoff_status, _ = api(
                "POST",
                f"/api/v1/orders/{first_pin_order_id}/handoff",
                {"pin": "042731"},
                accepted=(409,),
                headers={"Cookie": pin_cookie},
            )
            if early_handoff_status != 409:
                raise RuntimeError("scheduled PIN stop accepted handoff before arrival")

            def wait_for_arrival(order_id: str) -> None:
                deadline = time.monotonic() + 30
                target = pin_target_by_order[order_id]
                while time.monotonic() < deadline:
                    api(
                        "POST",
                        f"/api/v1/flights/telemetry/{pin_drone_id}",
                        {
                            "ts": datetime.now(UTC).isoformat(),
                            "latitude": target["lat"],
                            "longitude": target["lon"],
                            "altitude_m": target["altitude_m"],
                            "altitude_reference": "home_relative",
                            "speed_mps": 0.0,
                            "vertical_speed_mps": 0.0,
                            "battery_pct": 95.0,
                            "status": "in_flight",
                        },
                        accepted=(201,),
                    )
                    stop_status = run(
                        [*psql, "--tuples-only", "--no-align"],
                        text=f"SELECT status FROM neurovector.delivery_stops WHERE order_id='{order_id}';",
                    )
                    if stop_status == "arrived":
                        return
                    time.sleep(2.0)
                raise RuntimeError(f"fresh synthetic telemetry did not mark stop arrived: {order_id}")

            def assert_stable_arrival_notification(order_id: str) -> str:
                _, first_tracking = api(
                    "GET",
                    f"/api/v1/account/orders/{order_id}/tracking",
                    accepted=(200,),
                    headers={"Cookie": pin_cookie},
                )
                _, repeated_tracking = api(
                    "GET",
                    f"/api/v1/account/orders/{order_id}/tracking",
                    accepted=(200,),
                    headers={"Cookie": pin_cookie},
                )
                notification_id = first_tracking.get("arrival_notification_id")
                if (
                    not notification_id
                    or repeated_tracking.get("arrival_notification_id") != notification_id
                    or not first_tracking.get("arrived_at")
                ):
                    raise RuntimeError(
                        f"arrival tracking did not return a stable notification ID: {first_tracking}"
                    )
                return notification_id

            wait_for_arrival(first_pin_order_id)
            first_notification_id = assert_stable_arrival_notification(first_pin_order_id)
            wrong_pin_status, _ = api(
                "POST",
                f"/api/v1/orders/{first_pin_order_id}/handoff",
                {"pin": "999999"},
                accepted=(401,),
                headers={"Cookie": pin_cookie},
            )
            if wrong_pin_status != 401:
                raise RuntimeError("wrong recipient PIN was not rejected")
            _, first_handoff = api(
                "POST",
                f"/api/v1/orders/{first_pin_order_id}/handoff",
                {"pin": "042731"},
                accepted=(200,),
                headers={"Cookie": pin_cookie},
            )
            if first_handoff.get("status") != "delivered":
                raise RuntimeError(f"correct recipient PIN did not complete first stop: {first_handoff}")
            replay_status, _ = api(
                "POST",
                f"/api/v1/orders/{first_pin_order_id}/handoff",
                {"pin": "042731"},
                accepted=(409,),
                headers={"Cookie": pin_cookie},
            )
            if replay_status != 409:
                raise RuntimeError("recipient PIN replay was not rejected")

            second_stop_status = run(
                [*psql, "--tuples-only", "--no-align"],
                text=f"SELECT status FROM neurovector.delivery_stops WHERE order_id='{second_pin_order_id}';",
            )
            if second_stop_status != "scheduled":
                raise RuntimeError(f"second PIN stop advanced before previous receipt: {second_stop_status}")
            next_early_status, _ = api(
                "POST",
                f"/api/v1/orders/{second_pin_order_id}/handoff",
                {"pin": "042731"},
                accepted=(409,),
                headers={"Cookie": pin_cookie},
            )
            if next_early_status != 409:
                raise RuntimeError("next scheduled stop accepted PIN before its arrival")

            api("PATCH", f"/api/v1/orders/{second_pin_order_id}/status", {"status": "cancelled"}, accepted=(200,), headers=admin_headers)
            third_stop_status = run(
                [*psql, "--tuples-only", "--no-align"],
                text=f"SELECT status FROM neurovector.delivery_stops WHERE order_id='{third_pin_order_id}';",
            )
            if third_stop_status != "scheduled":
                raise RuntimeError(f"third stop advanced before its turn: {third_stop_status}")
            third_early_status, _ = api(
                "POST",
                f"/api/v1/orders/{third_pin_order_id}/handoff",
                {"pin": "042731"},
                accepted=(409,),
                headers={"Cookie": pin_cookie},
            )
            if third_early_status != 409:
                raise RuntimeError("third scheduled stop accepted PIN before arrival")

            wait_for_arrival(third_pin_order_id)
            third_notification_id = assert_stable_arrival_notification(third_pin_order_id)
            _, second_handoff = api(
                "POST",
                f"/api/v1/orders/{third_pin_order_id}/handoff",
                {"pin": "042731"},
                accepted=(200,),
                headers={"Cookie": pin_cookie},
            )
            if second_handoff.get("status") != "delivered":
                raise RuntimeError(f"correct recipient PIN did not complete second stop: {second_handoff}")
            second_replay_status, _ = api(
                "POST",
                f"/api/v1/orders/{third_pin_order_id}/handoff",
                {"pin": "042731"},
                accepted=(409,),
                headers={"Cookie": pin_cookie},
            )
            if second_replay_status != 409:
                raise RuntimeError("second recipient PIN replay was not rejected")
            arrival_count = run(
                [*psql, "--tuples-only", "--no-align"],
                text=f"""
                    SELECT count(*) FROM neurovector.order_tracking_events
                    WHERE order_id IN ('{first_pin_order_id}', '{second_pin_order_id}', '{third_pin_order_id}')
                      AND event_type='arrived' AND dedupe_key='arrival';
                """,
            )
            consumed_count = run(
                [*psql, "--tuples-only", "--no-align"],
                text=f"""
                    SELECT count(*) FROM neurovector.order_handoffs
                    WHERE order_id IN ('{first_pin_order_id}', '{second_pin_order_id}', '{third_pin_order_id}')
                      AND consumed_at IS NOT NULL;
                """,
            )
            if arrival_count != "2" or consumed_count != "2":
                raise RuntimeError(
                    f"arrival notification or PIN consumption was not exactly once: arrivals={arrival_count}, consumed={consumed_count}"
                )
            completed_status = run(
                [*psql, "--tuples-only", "--no-align"],
                text=f"SELECT status FROM neurovector.deliveries WHERE id='{pin_delivery_id}';",
            )
            if completed_status != "completed":
                raise RuntimeError(
                    f"cancelled sibling blocked completed recipient handoffs: {completed_status}"
                )
            all_order_ids.extend(pin_order_ids)

            return {
                "eligible_orders_assigned": 4,
                "multi_stop_deliveries": len(deliveries),
                "orders_per_delivery": 2,
                "overweight_and_unweighed_orders_queued": 2,
                "scheduled_weight_grams": 300,
                "measured_weight_grams": measurement["weight_grams"],
                "capacity_exceeded": measurement["capacity_exceeded"],
                "overweight_dispatch_blocked": True,
                "missing_zero_and_stale_measurements_blocked": True,
                "current_capacity_reweigh_check": True,
                "latest_safe_reweigh_cleared_prior_overweight": True,
                "latest_failed_telemetry_cannot_be_masked": True,
                "not_ready_stop_dispatch_returns_conflict": True,
                "merchant_ready_path_syncs_assigned_before_dispatch": True,
                "stale_only_fleet_drone_not_selected": True,
                "fresh_telemetry_restores_scheduler_eligibility": True,
                "future_device_timestamp_reported_offline": True,
                "automatic_delivery_id": pin_delivery_id,
                "automatic_route_id": pin_delivery["route_id"],
                "automatic_mission_plan_id": pin_plan["id"],
                "automatic_native_report": checked_pin_plan,
                "automatic_stop_order_ids": [first_pin_order_id, second_pin_order_id, third_pin_order_id],
                "recipient_pin_wrong_correct_replay": True,
                "scheduled_stop_arrives_before_pin_handoff": True,
                "next_stop_waits_for_previous_receipt": True,
                "arrival_events_deduplicated": True,
                "arrival_notification_ids_stable": bool(first_notification_id and third_notification_id),
                "cancelled_sibling_does_not_block_delivery_completion": True,
                "assignment_failure_rolled_back": True,
                "assignment_retry_idempotent": True,
                "zero_measurement_grams": zero_measurement["weight_grams"],
                "safe_reweigh_grams": safe_reweigh["weight_grams"],
                "cancelled_batch_capacity_released": True,
                "remaining_stop_rescheduled": True,
                "physical_flight": False,
                "database_ids": all_order_ids,
            }

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
        # A fresh receive time cannot disguise a delayed device timestamp. Mission
        # verification remains necessary, and live telemetry is an independent gate.
        stale_sample = {
            "ts": (datetime.now(UTC) - timedelta(hours=1)).isoformat(),
            "latitude": fixture["origin"]["lat"],
            "longitude": fixture["origin"]["lon"],
            "status": "idle",
        }
        api("POST", f"/api/v1/flights/telemetry/{drone['id']}", stale_sample, accepted=(201,))
        _, stale_status = api("GET", f"/api/v1/flights/status/{drone['id']}")
        if stale_status["online"]:
            raise RuntimeError("stale device telemetry was marked online")
        stale_dispatch_status, _ = api(
            "PATCH",
            f"/api/v1/deliveries/{delivery_id}/status",
            {"status": "dispatched"},
            accepted=(409,),
        )
        if stale_dispatch_status != 409:
            raise RuntimeError("verified mission dispatched with stale device telemetry")
        api(
            "POST",
            f"/api/v1/flights/telemetry/{drone['id']}",
            {
                "ts": datetime.now(UTC).isoformat(),
                "latitude": fixture["origin"]["lat"],
                "longitude": fixture["origin"]["lon"],
                "status": "idle",
            },
            accepted=(201,),
        )
        _, live_status = api("GET", f"/api/v1/flights/status/{drone['id']}")
        if not live_status["online"]:
            raise RuntimeError("fresh device telemetry was not marked online")
        for status in ("dispatched", "in_flight"):
            _, delivery = api(
                "PATCH", f"/api/v1/deliveries/{delivery_id}/status", {"status": status}
            )
            if delivery["status"] != status:
                raise RuntimeError(f"delivery did not transition to {status}")
        run(
            psql,
            text=f"UPDATE neurovector.delivery_fleet SET active=false WHERE drone_id='{drone['id']}';",
        )
        scheduling_report = schedule_weight_scenarios()
        return {
            "isolated_database": True,
            "temporary_database_writes": True,
            "production_database_writes": False,
            "schema_migrations": len(migrations),
            "cross_checks": reports,
            "unverified_dispatch_http_status": denied_status,
            "verified_mission_stale_telemetry_dispatch_status": stale_dispatch_status,
            "registered_fleet_fresh_telemetry_required": True,
            "verified_delivery_status": "in_flight",
            "physical_flight": False,
            "window_handoff": "API validation only; no motor commands",
            "scheduling_and_weight_scenarios": scheduling_report,
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
