# Isolated synthetic PX4 school test

This workflow is for a disposable simulator only. It generates a separately named procedural scene from `fixtures/school.json` using its building and risk-marker collision boxes, plus the stock PX4 `gz_x500` aircraft. It does not restore the missing LFS meshes, reproduce the original artwork, observe real people, or measure camera recognition. The original `school` scene retains its own mandatory mesh preflight.

The agent rejects non-loopback network interfaces and requires explicit synthetic opt-in. Do not use host networking, shared IPC, physical devices, external MAVLink, a production database, or shared MQTT. The example uses a task-owned container, no published ports and no devices. It requires the existing PX4 image, Docker, a local synthetic backend on `127.0.0.1:8083`, a private fixture contract, and the independently compiled native bridge and route checker.

```sh
mkdir -p .omo/evidence/drone-autonomy/sim
python3 sim/scripts/generate-procedural-school.py .omo/evidence/drone-autonomy/sim/school_procedural.sdf
docker run -d --network none --ipc private --shm-size 1g \
  --name nv-autonomy-sitl-7bd1 --entrypoint /bin/bash \
  -e ROS_DOMAIN_ID=217 -e HEADLESS=1 -e PX4_SIM_MODEL=gz_x500 \
  -e PX4_GZ_WORLD=school_procedural -e PX4_GZ_MODEL_POSE=0,0,0.15,0,0,0 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v "$PWD/.omo/evidence/drone-autonomy/sim/school_procedural.sdf:/tmp/school_procedural.sdf:ro" \
  -v "$PWD/sim:/nv-sim:ro" px4io/px4-sitl-gazebo-ros2:latest \
  -lc 'cp /tmp/school_procedural.sdf /opt/px4-gazebo/share/gz/worlds/school_procedural.sdf && exec px4-gazebo -d'
# Once ready, bounded independent takeoff/hover/landing smoke:
timeout 120 docker exec -e NV_SITL_ONLY=network-none nv-autonomy-sitl-7bd1 \
  python3 /nv-sim/sitl/smoke_takeoff.py --synthetic-takeoff-smoke
```

For the end-to-end synthetic run, use a backend-created paid order and automatic route, submit a native check of its exact NVPLAN2 bytes, measure payload and complete backend dispatch gates first. Its private contract identifies the matching API, drone, route, plan, fingerprint, owner session and order. After an actual fresh PX4 position reaches the private backend and the delivery is in flight, run:

```sh
python3 -u sim/sitl/run_verified_mission.py <plan.json> <nv_route_check> <nv_avoidance_bridge> <private-contract.env> --synthetic-only
```

The runner rechecks the active delivery, native validator and exact plan bytes, and binds the authenticated ordered delivery records and their customer coordinates to the verified mandatory stops. It waits at each physical target for its own confirmed delivery, not merely a timer; out-of-order completion fails closed. Pickup timeout requests landing without claiming handoff. The optional `--exercise-replan-hold` explicitly holds on the first leg and waits for an independently backend-activated plan at the documented task-owned paths; it checks the new native-verified plan and ordered remaining customer locations before resuming. This is a deliberately injected hold. `--exercise-physical-obstacle` instead requires the separately named obstacle container and inserts a collision sphere, confirms its Gazebo scene pose, supplies the matching explicitly synthetic ground truth to native avoidance, and holds on native `request_replan`. A backend operator must obtain, verify and activate a replacement route before the controller resumes. The optional `--exercise-stale-observation` deliberately ages the complete static fixture observation for three seconds and records the native hold; it is a synthetic fault injection, not camera input. Positive native `slow` uses reduced position lookahead and measured-speed feedback; zero desired speed, `hold`, and `request_replan` hold. Native-proposed obstacle detours are not automatically activated.

The MAVLink agent sends setpoints independently of backend requests and native decisions, with a host-decision watchdog and local/GPS freshness checks. It converts each target using the current PX4 local and GPS frames rather than retaining an arming-time altitude offset. Level legs command their verified altitude and the runner aborts on excessive measured altitude, cross-track error or velocity. Its GPS timestamp is captured on receiving the simulated source sample; all avoidance ages use monotonic seconds. PX4 arm is not forced and its preflight checks remain active. The ground-truth observation source is valid only for the complete static procedural fixture, not for unknown or moving obstacles in the real world.

The synthetic `--auto-global-replan` option additionally requires `--exercise-physical-obstacle`. Once native avoidance requests a replan and the aircraft has physically stabilized, it creates the fixture-specific exclusion zone through the private backend, submits a global plan, runs the native cross-check, and attempts authenticated activation using a fresh accepted telemetry ID. It continues holding until the activated plan, mandatory customer stops and live native avoidance have been checked. Errors close the control transport and trigger its landing failsafe. This is an isolated test profile, not a production perception or credential configuration.

Stop and remove only your own container after dependent evidence capture is complete. Never run `docker compose down -v` against shared services.
