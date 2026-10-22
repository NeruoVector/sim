#!/usr/bin/env python3
"""Run synthetic school geometry and real Rust/C++ decision code without a DB."""

import argparse
import json
import math
import os
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

SIM = Path(__file__).resolve().parents[1]
PROJECT = SIM.parent


def run(command, *, text=None, env=None, accepted=(0,)):
    result = subprocess.run(
        command,
        check=False,
        input=text,
        text=True,
        capture_output=True,
        env=env,
        timeout=600,
        cwd=PROJECT,
    )
    if result.returncode not in accepted:
        raise RuntimeError(
            f"{' '.join(map(str, command))}\n{result.stdout[-4000:]}\n{result.stderr[-4000:]}"
        )
    return result


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def fnv(text):
    value = 0xCBF29CE484222325
    for byte in text.encode():
        value = ((value ^ byte) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return f"{value:016x}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--with-backend",
        action="store_true",
        help="also test actual APIs using a disposable Docker PostgreSQL database",
    )
    options = parser.parse_args()
    fixture = json.loads((SIM / "fixtures/school.json").read_text())
    require(fixture["synthetic"] is True, "school fixture must be explicitly synthetic")
    for tool in ("cargo", "g++"):
        require(shutil.which(tool), f"{tool} is required")
    run(["python3", str(SIM / "scripts/generate-school-world.py"), "--check"])
    with tempfile.TemporaryDirectory(prefix="nv-school-") as temporary:
        work = Path(temporary)
        env = os.environ | {
            "SCHOOL_FIXTURE_PATH": str(SIM / "fixtures/school.json"),
            "SCHOOL_ARTIFACT_DIR": str(work),
        }
        run(
            [
                "cargo",
                "test",
                "--offline",
                "--manifest-path",
                str(PROJECT / "backend/Cargo.toml"),
                "school_fixture_routes_and_window",
                "--",
                "--ignored",
            ],
            env=env,
        )
        flags = [
            "g++",
            "-std=c++20",
            "-Wall",
            "-Wextra",
            "-Wpedantic",
            "-Werror",
            "-I",
            str(PROJECT / "drone/include"),
        ]
        scenarios = work / "school-scenarios"
        checker = work / "nv-route-check"
        run(
            flags
            + [
                str(SIM / "tests/school_scenarios.cpp"),
                str(PROJECT / "drone/src/avoidance.cpp"),
                "-o",
                str(scenarios),
            ]
        )
        run(
            flags
            + [
                str(PROJECT / "drone/src/mission.cpp"),
                str(PROJECT / "drone/app/route_check.cpp"),
                "-o",
                str(checker),
            ]
        )
        rows = [
            " ".join(
                map(str, [o["id"], o["priority"], *o["position_enu"], o["radius_m"]])
            )
            for o in fixture["obstacles"]
        ]
        window = fixture["window"]
        rows += [
            " ".join(
                map(
                    str,
                    [
                        *window["position_enu"],
                        window["outward_bearing_deg"],
                        window["standoff_m"],
                    ],
                )
            ),
            str(fixture["drone"]["radius_m"]),
        ]
        avoidance = json.loads(
            run([str(scenarios)], text="\n".join(rows) + "\n").stdout
        )
        require(
            avoidance["passed"] == len(avoidance["cases"]) >= 12,
            "avoidance checks missing",
        )
        drone = fixture["drone"]
        args = [
            str(checker),
            "--drone-id",
            drone["id"],
            "--max-power-w",
            str(drone["max_power_w"]),
            "--min-power-w",
            str(drone["modeled_power_w"]),
            "--battery-wh",
            str(drone["battery_wh"]),
        ]
        cross_checks = []
        plans = {}
        for name in ("school-detour", "school-window"):
            plan = json.loads((work / f"{name}.json").read_text())
            report = json.loads(
                run(
                    args + ["--fingerprint", plan["route_fingerprint"]],
                    text=plan["plan_text"],
                ).stdout
            )
            require(report["valid"], f"drone rejected {name}: {report['errors']}")
            payload = plan["payload"]
            index = payload["selected_index"]
            require(report["selected_index"] == index, "candidate selection disagrees")
            for metric in ("distance_m", "estimated_duration_s", "estimated_energy_wh"):
                expected = payload["validations"][index][metric]
                require(
                    math.isclose(report[metric], expected, rel_tol=1e-6, abs_tol=0.01),
                    f"Rust/C++ {metric} mismatch for {name}",
                )
            cross_checks.append({"name": name, **report})
            plans[name] = plan
            (work / f"{name}-cross-check.json").write_text(
                json.dumps(report, indent=2) + "\n"
            )
        plan = plans["school-detour"]
        negative = []
        # Changing exact bytes must fail even when the geometry remains identical.
        rejected = json.loads(
            run(
                args + ["--fingerprint", plan["route_fingerprint"]],
                text=plan["plan_text"] + "\n",
                accepted=(2,),
            ).stdout
        )
        require(
            not rejected["valid"]
            and "expected_fingerprint_mismatch" in rejected["errors"],
            "tampered plan was not rejected",
        )
        negative.append("changed-plan-bytes")
        for name, token, value, error in (
            (
                "expired-plan",
                3,
                str(int(time.time()) - 1),
                "mission_expired_or_invalid_ttl",
            ),
            (
                "blocked-candidate-selected",
                4,
                "0",
                "selected_route_not_minimum_valid_candidate",
            ),
        ):
            lines = plan["plan_text"].splitlines(keepends=True)
            header = lines[0].split()
            header[token] = value
            text = " ".join(header) + "\n" + "".join(lines[1:])
            rejected = json.loads(
                run(
                    args + ["--fingerprint", fnv(text)], text=text, accepted=(2,)
                ).stdout
            )
            require(
                not rejected["valid"] and error in rejected["errors"],
                f"{name} not rejected",
            )
            negative.append(name)
        summary = {
            "synthetic": True,
            "backend_fixture_groups": 1,
            "avoidance": avoidance,
            "cross_checks": cross_checks,
            "rejected_cases": negative,
            "database_writes": options.with_backend,
            "production_database_writes": False,
            "physical_flight": False,
        }
        if options.with_backend:
            from school_backend import check_backend

            summary["backend_api"] = check_backend(work, fixture, args)
        destination = SIM / "logs/school-tests"
        destination.mkdir(parents=True, exist_ok=True)
        for path in work.glob("*.json"):
            shutil.copyfile(path, destination / path.name)
        (destination / "report.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
        )
        print(
            f"PASS: {avoidance['passed']} avoidance scenarios, 2 Rust/C++ route cross-checks, "
            f"{len(negative)} rejection checks; production database untouched"
        )
        if options.with_backend:
            print(
                "PASS: isolated backend API planning, drone cross-check and dispatch gate"
            )
        print(f"Report: {destination / 'report.json'}")


if __name__ == "__main__":
    main()
