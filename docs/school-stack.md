# 학교 시뮬레이션 스택 검증

## 실행

`sim/` 디렉터리에서 실행합니다. Docker Desktop에서 host networking을 사용하지 않는
경우 아래 환경 변수를 **각 터미널에** 설정하세요. 기본 Linux host 네트워크에서는
`COMPOSE_FILE`을 설정하지 않아도 됩니다.

```bash
export COMPOSE_FILE=compose.yaml:compose.desktop.yaml
export SIM_X11_SOCKET=./logs/x11
export LIBGL_ALWAYS_SOFTWARE=1
```

```bash
make build
make school-test        # 저장된 계획이 없을 때
make up
make agent             # 터미널 1: PX4 DDS agent
make school HEADLESS=1 # 터미널 2: Gazebo + PX4
make school-gui        # 별도 터미널: 기존 학교 서버에 실제 Gazebo 창 연결
make school-bridge     # 터미널 3: RGB/depth/point cloud/ground-truth pose
make school-scene      # 터미널 4: 학교 STL, 계획 경로, 실제 기체 TF
make foxglove          # 터미널 5: Foxglove bridge
make rviz              # 터미널 6: 호스트 X11 사용 가능할 때
# X11이 없으면 대신 가상 화면에서 렌더링:
make rviz-headless
```

한 역할을 중복 실행하지 마세요. 특히 `/clock` 브릿지가 두 개면 메시지 도착 순서 때문에
시간이 뒤로 흐르는 경고가 발생할 수 있습니다. 종료는 `make down`으로 진행합니다.

`make school-gui`는 브라우저 뷰어가 아닌 Gazebo Harmonic의 실제 GUI를 엽니다.
현재 학교 서버가 실행 중이어야 합니다. 학교 전체를 보는 카메라 시점으로 시작하며
`gz sim -g`만 실행하므로 두 번째 물리 서버나 PX4 기체를 만들지 않습니다.
로컬 X11 데스크톱이 필요합니다. Docker Desktop에서는 호스트의 `socat`과 `xauth`를 사용해
localhost X11 릴레이와 임시 인증 쿠키로 연결합니다. GUI 실행 명령 종료 시 릴레이와
임시 인증 파일을 정리하며, X 서버의 접근 제어를 해제하지 않습니다.

`make school-scene MISSION=outdoor`는 일반 배송 경로, 기본값은 창문 밖 정지 경로입니다.
경로는 저장된 테스트 계획이며 비행 명령으로 전송하지 않습니다. 기체 위치는 재생값이 아닌
Gazebo ground-truth odometry입니다. PX4 추정 위치는 별도 `/fmu/out/vehicle_local_position_v1`
토픽으로 확인할 수 있으며 NED 좌표이므로 ENU의 `map`에 그대로 표시하면 안 됩니다.

## Foxglove

Foxglove에서 Foxglove WebSocket 연결을 만들고 `ws://127.0.0.1:8765`에 연결합니다.
3D 패널의 기준 프레임을 `map`으로 설정하고 `/school/scene`, `/school/planned_route`,
`/tf`를 활성화하세요. Image 패널에서는 `/camera/image_raw`를 선택합니다.
브릿지는 학교와 드론 STL 두 파일만 asset 요청으로 제공합니다.

검증된 bridge 3.6.0 서버는 `foxglove.sdk.v1`로 연결됐습니다. 구형 프로토콜만 사용하는
클라이언트가 HTTP 400을 받으면 최신 Foxglove 클라이언트로 확인하세요.
Docker Desktop용 override는 브릿지 포트를 localhost에 노출하며 외부 장비의 DDS 접속이나
MAVLink 연결을 별도로 설정하지 않습니다.

## 검증 명령과 결과

스택 실행 후 별도 터미널에서:

```bash
make school-stack-check
```

검증기는 비행 명령을 보내지 않습니다. 실제 메시지를 반복 수신해 RGB·camera info 크기,
depth encoding, point cloud, PX4 비무장 상태, 학교·드론·창문 Marker, 유효한 경로 좌표와
`map → x500_depth_0 → base_link` TF를 확인합니다. 최대 120초 후 미수신 항목을 오류로 보고합니다.

학교 모드는 소프트웨어 렌더링을 위해 RGB를 640×360, depth를 640×480, 두 센서를 5Hz로
설정합니다. 시야각과 종횡비는 유지하며 RGB CameraInfo는 Gazebo에서 함께 생성됩니다.
복사한 카메라 모델을 사용하며 원본 OakD-Lite 파일은 수정하지 않습니다.
학교 기체에는 시각화용 OdometryPublisher를 추가하고 DDS의 벽시계 동기화를 끕니다.
이 설정은 실기체 시간 동기화·센서 보정을 대신하지 않습니다.

학교 브릿지의 RGB·camera info·depth·point cloud는 ROS 구독자가 있을 때만
Gazebo에 연결합니다(`lazy: true`). 발행 큐는 최신 메시지 1개로 제한합니다.
구독자 감지 주기만큼 첫 메시지가 늦어질 수 있습니다. `/clock`과 odometry는
계속 연결합니다. 학교 마커와 저장된 경로는 시작할 때 한 번 만들고, 매초
타임스탬프와 실제 기체 마커를 갱신해 발행합니다.

2026-10-23 최적화 후 실행 중인 스택에서 검증기를 통과했습니다(마커 12개,
경로점 3개, RGB 640×360, depth 640×480, point cloud 307,200점).
검증기가 종료된 뒤 `gz topic -i`로 depth와 point cloud의 구독자가 사라지는
것도 확인했습니다. 전체 CPU·프레임 속도 향상률은 측정하지 않았습니다.

2026-10-23 실제 실행 기록은 `logs/`에 보관합니다:

확인된 구성은 Gazebo 8.15.0(Harmonic), PX4 SITL 1.18.0 개발 빌드
(`61907962c9220437ec01a98fa26bfbf5cbb254b5`), ROS 2 Jazzy,
RViz 14.1.24, foxglove_bridge 3.6.0입니다. `school-stack-versions.txt`에 원문을 기록했습니다.

- `school-stack-ros-report.json`: ROS 수신·PX4 상태·TF 검증 결과
- `school-stack-foxglove-report.json`: 호스트 WebSocket에서 8개 토픽의 binary 메시지와 STL 두 파일 수신
- `school-stack-rviz.png`: Xvfb 가상 디스플레이에서 RViz가 실제 렌더링한 화면
- `school-stack-sitl.log`, `school-stack-foxglove.log`, `school-stack-rviz.log`: 실행 로그

RViz는 가상 디스플레이에서 확인했습니다. 사용자의 모니터/X11 창과 Foxglove 앱 자체의
패널 렌더링은 별도 확인이 필요합니다. 이번 검증은 시동·이륙·경로 추종·동적 회피·실물 배송을
포함하지 않습니다. STL과 더미 금지구역은 별도 데이터이며 실제 지형 충돌 검증도 남아 있습니다.
