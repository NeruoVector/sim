# NeuroVector Sim

드론 시뮬레이션 환경입니다. 기성 PX4 SITL 이미지를 기반으로 RViz2 · Foxglove 를 더해 한 컨테이너에서 돌립니다.

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
make up                     # 컨테이너 기동
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
compose.yaml            # 단일 sim 서비스 (host 네트워킹)
docker/
├── Dockerfile          # 기성 PX4 이미지 + rviz2/foxglove-bridge/ros_gz
├── entrypoint.sh       # ROS 2 + px4_ros2 + /ros2_ws 오버레이 소싱
└── bashrc.append       # 셸 편의 설정
config/
├── gz_bridge.yaml      # ros_gz_bridge 파라미터 (Gazebo -> ROS 2)
└── px4.rviz            # RViz2 기본 설정
ros2_ws/src/nv_telemetry/   # 예제 ROS 2 패키지
scripts/setup-host.sh
logs/                   # PX4 ulog 저장 위치
Makefile
```

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
