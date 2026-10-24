# NeuroVector Sim

드론 시뮬레이션 환경입니다. 기성 PX4 SITL 이미지를 기반으로 RViz2 · Foxglove 를 더해 한 컨테이너에서 돌립니다.

[안전 브리지 실행 환경](docs/safety-bridge.md): Conan·ROS 2 환경 구성,
실제 PX4 상태를 사용하는 안전 판단·정지/복귀 명령 연결과 통합 검증을 제공합니다.
일반 RGB 카메라와 전방·하방 초음파 입력은 `config/rgb_ultrasound.yaml`로
구성합니다. 실제 보정과 오차 범위를 설정하기 전에는 비행 진행을 차단합니다.

| 구성 요소 | 버전 / 이미지 |
| --- | --- |
| PX4 Autopilot (SITL) | `px4io/px4-sitl-gazebo-ros2` (패키징된 펌웨어, 소스 빌드 불필요) |
| Gazebo | Harmonic |
| ROS 2 | Jazzy (Ubuntu 24.04) |
| RViz2 | `ros-jazzy-rviz2` |
| Foxglove | `ros-jazzy-foxglove-bridge` |
| PX4 ↔ ROS 2 | Micro XRCE-DDS Agent v2.4.3 (이미지 내장) + `px4_msgs` |

> 왜 Docker 인가: ROS 2 Jazzy / Gazebo Harmonic 은 Ubuntu 24.04 대상이라 이 머신(Debian 13)에는 네이티브 설치가 어렵습니다.

## 요구 사항

- Docker Engine + Compose v2 (호스트에 아직 없으면 `scripts/setup-host.sh` 안내 참고)
- Linux + X11 (Gazebo/RViz GUI용). GUI 없이 `HEADLESS=1` 로도 동작합니다.
- 최초 빌드 시 수 GB 다운로드

## 빠른 시작

```bash
./scripts/setup-host.sh     # Docker 확인 + .env 생성 + X11 허용 + 이미지 빌드
make up                     # 컨테이너 기동 (내부적으로 --profile sim 사용)
```

이후 터미널을 나눠서 실행합니다.

```bash
make agent      # 터미널 2: Micro XRCE-DDS Agent (ROS 2 <-> PX4)
make px4        # 터미널 3: PX4 SITL + Gazebo GUI
make foxglove   # 터미널 4: Foxglove bridge  -> ws://localhost:8765
make rviz       # 터미널 5: RViz2
```

헤드리스(서버/CI)로 돌릴 때:

```bash
make headless   # Gazebo GUI 없이 PX4 + Gazebo 실행
```

## 구조

```
compose.yaml            # 단일 sim 서비스 (host 네트워킹, `sim` 프로필)
docker/
├── Dockerfile          # 기성 PX4 이미지 + rviz2/foxglove-bridge/ros_gz
├── entrypoint.sh       # ROS 2 + px4_ros2 + /ros2_ws 오버레이 소싱
└── bashrc.append       # 셸 편의 설정
config/
├── gz_bridge.yaml      # ros_gz_bridge 파라미터 (Gazebo -> ROS 2)
└── px4.rviz            # RViz2 기본 설정
models/                 # Gazebo 3D 모델 (Git LFS)
ros2_ws/src/nv_telemetry/   # 예제 ROS 2 패키지
scripts/setup-host.sh
logs/                   # PX4 ulog 저장 위치
Makefile
```

> `sim` 서비스는 프로필로 분리되어 있어 `platform` 레포에서 `docker compose up` 할 때는 뜨지 않습니다.
> platform 에서 함께 띄우려면 `docker compose --profile sim up -d`.

## 3D 모델 (Git LFS)

Gazebo 기체/환경/소품 모델은 `models/` 에 두고 Git LFS 로 관리합니다.
별도 모델 레포는 필요하지 않습니다.

```bash
git lfs install          # 최초 1회 (미설치 시 apt/brew 로 git-lfs 설치)
cp -R /path/to/model models/drones/my-drone
git add models/drones/my-drone
git lfs ls-files
git commit -m "feat: my-drone Gazebo 모델 추가"
```

- `models/` 는 컨테이너의 `/models` 로 마운트되고 `GZ_SIM_RESOURCE_PATH` 에
  `/models`, `/models/props` 등이 자동 추가됩니다.
- `.dae`, `.stl`, `.obj`, `.fbx`, `.glb`, `.step`, `.usd*` 등은 `.gitattributes` 에서 LFS 로 추적합니다.
- `model.config`, `model.sdf`, `*.urdf`, `*.xacro` 같은 텍스트 파일은 LFS 로 보내지 않습니다.
- 자세한 구조와 주의 사항은 [`models/README.md`](models/README.md) 참고.

### 샘플(더미) 모델

Gazebo Fuel 의 CC0 모델 10개를 `models/props/` 에 포함해 두었습니다.
모두 `model://<이름>` URI 로 바로 사용할 수 있습니다.

| 디렉터리 | 설명 | 크기(대략) |
| --- | --- | --- |
| `construction-barrel` | 공사용 배럴 | 0.6 × 1.0 m |
| `construction-cone` | 라바콘 | 0.4 × 0.7 m |
| `jersey-barrier` | 콘크리트 방호벽 | 2.0 × 0.8 m |
| `fire-hydrant` | 소화전 | 0.4 × 1.0 m |
| `oak-tree` | 참나무 | 3 × 6 m |
| `pine-tree` | 소나무 | 3 × 7 m |
| `prius-hybrid` | 승용차 | 1.8 × 1.5 m |
| `office-building` | 사무용 건물 | 10 × 8 m |
| `dumpster` | 쓰레기 수거함 | 2.0 × 1.4 m |
| `walking-actor` | 보행자(애니메이션) | 0.5 × 1.7 m |

Gazebo/PX4 를 실행한 뒤 원하는 위치에 배치할 수 있습니다.

```bash
make px4
make spawn-obstacle MODEL=construction-barrel X=8 Y=0
make spawn-obstacle MODEL=oak-tree X=15 Y=3
make spawn-obstacle MODEL=walking-actor X=10 Y=-2
```

다시 내려받거나 다른 샘플을 추가하려면:

```bash
python3 scripts/fetch-sample-models.py           # 없는 모델만 다운로드
python3 scripts/fetch-sample-models.py --force   # 전체 재다운로드
```

모델별 원본 URL/라이선스는 각 디렉터리의 `ATTRIBUTION.md` 에 있습니다.
`GZ_SIM_RESOURCE_PATH` 에 `/models/props` 가 포함되므로 월드 파일에서도
`<uri>model://construction-barrel</uri>` 처럼 참조할 수 있습니다.

## 포트 / 인터페이스

`network_mode: host` 라서 아래 포트가 호스트에서 그대로 보입니다.

| 포트 | 프로토콜 | 용도 |
| --- | --- | --- |
| 14550 | UDP | QGroundControl (MAVLink) |
| 14540 | UDP | MAVSDK / offboard API |
| 8888 | UDP | uXRCE-DDS (ROS 2 <-> PX4) |
| 8765 | TCP | Foxglove bridge (`ws://localhost:8765`) |

Foxglove Studio 에서 `Open connection` → `Foxglove WebSocket` → `ws://localhost:8765` 로 연결하면 토픽을 바로 볼 수 있습니다.

## GUI (X11)

```bash
make x11        # xhost +local:root
make px4
```

- 렌더링이 깨지면 `.env` 에서 `LIBGL_ALWAYS_SOFTWARE=1` 로 소프트웨어 렌더링을 켜세요.
- X11 소켓(`/tmp/.X11-unix`)과 `DISPLAY` 는 compose 에서 자동으로 전달됩니다.

## ROS 2 사용

컨테이너 셸:

```bash
make shell
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_status
```

예제 패키지 빌드/실행:

```bash
make shell
colcon build --symlink-install
source install/setup.bash
ros2 launch nv_telemetry telemetry.launch.py
```

Gazebo 카메라를 ROS 2 로 가져와 RViz/Foxglove 에서 보려면:

```bash
make bridge     # /camera/image_raw, /camera/camera_info, /clock
```

실제 Gazebo 토픽 이름은 기체/월드에 따라 다르니 `make gz-topics` 로 확인하고 `config/gz_bridge.yaml` 을 수정하세요.

## 환경 변수

`.env.example` 을 복사해 `.env` 로 사용합니다.

| 변수 | 기본값 | 설명 |
| --- | --- | --- |
| `SIM_BASE_IMAGE` | `px4io/px4-sitl-gazebo-ros2:latest` | 기반 이미지 (재현성 위해 digest 고정 권장) |
| `PX4_SIM_MODEL` | `gz_x500_depth` | 기체 모델. `gz_x500` 은 카메라 없음 |
| `PX4_GZ_WORLD` | `default` | Gazebo 월드 |
| `HEADLESS` | `0` | `1` 이면 GUI 없이 실행 |
| `ROS_DOMAIN_ID` | `0` | 호스트의 다른 ROS 2 와 맞춰야 통신됨 |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | CycloneDDS 를 쓰려면 `rmw_cyclonedds_cpp` |

## 문제 해결

**`make px4` 가 즉시 끝난다 / Gazebo 창이 안 뜬다**
`make x11` 을 실행했는지, `DISPLAY` 가 설정돼 있는지 확인하세요. 원격 접속이면 `LIBGL_ALWAYS_SOFTWARE=1`.

**`ros2 topic list` 에 `/fmu/out/*` 가 없다**
`make agent` 가 먼저 떠 있어야 합니다. 그 다음 `make px4` 로 PX4 를 실행하세요.

**호스트의 ROS 2 와 통신이 안 된다**
`ROS_DOMAIN_ID` 를 양쪽에서 동일하게 맞추고, 같은 네트워크(host)인지 확인하세요.

**`/dev/shm` 관련 오류 (Fast DDS)**
compose 에 `ipc: host` 가 들어 있습니다. 호스트와 ROS 2 를 함께 쓸 때는 이 설정을 유지하세요.

**호스트 PX4 펌웨어를 수정하고 싶다면**
이 구성은 패키징된 펌웨어를 씁니다. PX4 소스를 수정하려면 `ghcr.io/px4/px4-dev-ros2-gazebo` 툴체인 이미지로 별도 빌드가 필요합니다.

## 관련 레포

- [`platform`](https://github.com/NeruoVector/platform) — PostgreSQL / Redis / Kafka 인프라
- [`vision`](https://github.com/NeruoVector/vision) — 장애물 감지 (브릿지한 `/camera/image_raw` 를 입력으로 사용 가능)
- [`drone`](https://github.com/NeruoVector/drone) — 드론 하드웨어 HAL (C/C++ + Conan 2)
- [`backend`](https://github.com/NeruoVector/backend) — API 서버
- [`frontend`](https://github.com/NeruoVector/frontend) — 대시보드

## 라이선스

MIT

## 가상 학교 배송 테스트

실제 ROS/RViz/Foxglove 연동은 [학교 스택 실행·검증 안내](docs/school-stack.md)를 참고하세요.

[학교 테스트 안내](docs/school-demo.md)를 참고하세요. 실제 학교 정보 대신
`fixtures/school.json`의 가상 학교·창문·드론 사양·1/2/3단계 장애물을 사용합니다.
같은 파일에서 Gazebo 월드와 경로·회피 테스트 입력을 만듭니다.

```bash
make school-test          # Rust/C++ 실제 로직의 오프라인 교차 검증
make school-test-live     # 일회용 DB에서 실제 API·배송 상태 전환 검증
make school-view          # http://127.0.0.1:8766/viewer/ · STL/경로/위험 요소와 계획 재생
make up
make school HEADLESS=1    # 기존 PX4 gz_x500_depth + 가상 학교, 비행 시동 명령 없음
```

Docker Desktop에서 X11 마운트가 거부되면 `SIM_X11_SOCKET=./logs/x11`을
`make up`과 `make school HEADLESS=1` 앞에 붙이세요.

오프라인 검증 결과는 `logs/school-tests/report.json`에 저장됩니다. 운영 DB에는
더미 데이터를 쓰지 않으며, Gazebo 표시용 위험 표식은 실제 사람·차량 모델을 대체하는
색상 박스입니다. 센서 영상에 대한 인식 정확도 평가는 별도로 수행해야 합니다.
