# Synthetic school scene

This is a fictional school fixture, not a real school, recipient address, or authorized flight site. `sim/fixtures/school.json` is the single source for scene geometry, the aircraft selection, and mission test inputs. All positions are local ENU metres: x east, y north, z up. The geographic origin is synthetic.

The campus is the supplied `School_map.stl`, scaled to 120 m wide and loaded as a static visual and collision mesh. Fixture boxes remain for the ground and school no-fly collision proxy; their duplicate visuals are hidden. The priority markers represent logical obstacle test positions; the red marker named `student` is a geometric proxy at test flight altitude, not a realistically modeled child. These proxies are unsuitable as evidence of YOLO person/car detection accuracy.

The drone is the existing PX4 `gz_x500_depth` aircraft spawned by `px4-gazebo`. Its visible mesh comes from `models/drones/x500_base/meshes/drone.stl`; the stock x500 mass, collisions, sensors, and four-motor flight dynamics remain in use. The STL propellers are part of one visual mesh and do not spin independently.

## Regenerate and check

From the repository root:

```bash
python3 sim/scripts/generate-school-world.py
python3 sim/scripts/generate-school-world.py --check
# Offline XML and fixture checks only (does not establish runtime asset readiness):
python3 sim/scripts/generate-school-world.py --check --geometry-only
```

Or from `sim/`:

```bash
make school-check
```

The generator writes `models/environments/school-demo/model.sdf`, `model.config`, and `worlds/school.sdf`. Its check parses the XML, checks fixture geometry and the campus STL reference, asserts that the world contains no aircraft, and fails if a generated file is stale. Python's standard library is sufficient. Edit the JSON fixture and regenerate; editing the generated SDF directly will fail the check.

## PX4 and Gazebo

With the project's existing simulation image and container ready:

```bash
cd sim
make build
make up
make school HEADLESS=1
# For a host with an available X11 display:
make x11
make school HEADLESS=0
```

Stop the running SITL session before launching another. `make school` checks the generated files, selects `PX4_GZ_WORLD=school`, sets `NV_SCHOOL_SCENE=1`, and reuses the container entrypoint. School mode loads the aircraft model and takeoff pose from the mounted fixture. Normal `make px4` behavior is unchanged.

The Compose mounts expose `/worlds`, `/models`, and `/fixtures` read-only. The entrypoint adds `/worlds` and the existing model search directories to `GZ_SIM_RESOURCE_PATH` and copies the school SDF into the packaged PX4 world directory in school mode, because its launcher resets `PX4_GZ_WORLDS`. The world uses the fixture's latitude/longitude for Gazebo spherical coordinates; a navigation adapter must still agree on altitude datum and the local ENU origin.

On Docker Desktop without an X11 socket, use a workspace directory instead of mounting `/tmp/.X11-unix`:

```bash
SIM_X11_SOCKET=./logs/x11 HEADLESS=1 LIBGL_ALWAYS_SOFTWARE=1 make up
SIM_X11_SOCKET=./logs/x11 make school HEADLESS=1
```

Headless school mode uses PX4 daemon mode to avoid an interactive prompt loop when stdin is redirected. Stop the container with `make down` after testing; closing the PX4 process alone can leave Gazebo running.

## What this establishes

The fixture can drive offline route validation, detour planning, window standoff checks, and local priority avoidance. The complete school building remains solid; its blue window is a visual target and does not cut an entrance into the wall. A valid window mission ends at the outside hover standoff, then holds for the configured duration.

XML/self-check success verifies generated geometry and reference consistency. It does not establish GUI rendering, PX4 takeoff, autonomous route following, or physical collision avoidance. The C++ mission and avoidance layers remain decisions until a real SITL navigation adapter connects state estimates and verified commands. Separate image-based tests are required for YOLO accuracy.

## 실제 판단 코드의 자동 검증

프로젝트 루트에서 `python3 sim/scripts/test-school.py`, 또는 `sim`에서
`make school-test`를 실행합니다. Python 표준 라이브러리와 Rust/C++ 컴파일러를
사용하며 DB와 Docker 없이 동작합니다. 백엔드의 기존 순수 경로 검증·우회 생성·계획
직렬화와 드론의 기존 독립 검증기를 연결합니다. 백엔드 테스트는 `SCHOOL_FIXTURE_PATH`를
명시하는 ignored 테스트라 일반 `cargo test`에 외부 저장소 의존성을 추가하지 않습니다.

검사 항목:

- 건물을 통과하는 직선 경로 거부 및 외부 우회 경로 선택
- 창문 바깥 3m 정지, 수령 대기 시간과 에너지 포함
- Rust와 C++의 선택 후보·거리·시간·에너지 일치
- 바뀐 계획 바이트·만료·막힌 후보 선택 거부
- 학생·차량·나뭇잎의 경로 관련성, 긴급 충돌 우선순위, 센서 오류와 불확실한 거리
- 좌우 우회 전체 구간 검사와 창문 외부 호버, 카메라 방사 거리·미러 좌표 변환

결과와 테스트용 API 요청은 `logs/school-tests/`에 기록됩니다. 이 파일을 만들었다고
실제 백엔드의 계획/배송이 저장되거나 드론이 시동되는 것은 아닙니다.

이 학교와 기체 전력·배터리 수치는 검증용 더미입니다. 실제 학교 좌표·학생 정보·비행 허가와
연결되지 않으며 실기체의 제동거리·소비전력 보정을 대신하지 않습니다.

## 실제 백엔드 API 및 실행 확인

`make school-test-live` 또는 프로젝트 루트의 `python3 sim/scripts/test-school.py --with-backend`는
Docker의 `pgvector/pgvector:pg16` 이미지를 사용합니다. 이미지가 없다면 먼저
`docker pull pgvector/pgvector:pg16`을 실행하세요. 고유 이름의 일회용 DB에 마이그레이션과
가상 학교 금지구역을 적용하고 실제 백엔드 API로 다음을 검사합니다.

- 일반·창문 경로 생성 → C++ 독립 검증 → 교차 검증 결과 저장
- 거의 동일한 우회 후보의 Rust/C++ 부동소수점 판정 일치
- JSONB 저장·복원 후 계획 원문과 좌표 일치
- 창문 배송은 검증 전 출발 거부, 검증 후 dispatched → in_flight 허용

백엔드는 별도 로컬 포트와 명시적인 테스트 설정으로 실행합니다. 운영 DB를 사용하지 않으며
테스트 DB 컨테이너는 종료 시 제거합니다. 테스트 API 결과는 동일한 `logs/school-tests/`에
저장됩니다. 오프라인 실행은 실제 API·DB를 호출하지 않습니다.

2026-10-22 실행 결과: 회피 16개, Rust/C++ 경로 2개, 거부 조건 3개와 위 API 흐름이
통과했습니다. Docker의 학교 월드에 `x500_depth_0` 기체가 생성됐고 `/depth_camera`에서
640×480 프레임을 실제 수신했습니다. 이 확인은 기체 시동·이륙·자율 배송을 포함하지 않습니다.

The default `--check` and the container school entrypoint require readable, nonempty STL geometry. Git LFS pointer files are rejected with a restoration message; the offline geometry-only mode does not demonstrate that Gazebo can render the school. Rebuild the simulation image after updating the shared preflight script.
