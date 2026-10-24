COMPOSE := docker compose --profile sim
SERVICE := sim
ROS_EXEC := $(COMPOSE) exec $(SERVICE) /usr/local/bin/nv-entrypoint.sh
MODEL ?= construction-barrel
X ?= 8
Y ?= 0
Z ?= 0
HEADLESS ?= 0
MISSION ?= window

.PHONY: help init build up down restart shell x11 px4 headless school school-gui school-check school-test school-test-live school-view school-bridge school-scene school-stack-check agent foxglove rviz rviz-headless bridge spawn-obstacle topics gz-topics logs clean

help:
	@echo "make init      .env 생성 (없을 때만)"
	@echo "make x11       호스트 X11 접근 허용 (GUI 사용 시 1회)"
	@echo "make build     시뮬레이션 이미지 빌드"
	@echo "make up        컨테이너 기동 (백그라운드)"
	@echo "make shell     컨테이너 셸 접속"
	@echo "make px4       PX4 SITL + Gazebo 실행 (GUI)"
	@echo "make headless  PX4 SITL + Gazebo 실행 (헤드리스)"
	@echo "make school    가상 학교 + 기존 gz_x500_depth (HEADLESS=1 가능)"
	@echo "make school-gui  실행 중인 학교 서버에 실제 Gazebo GUI 연결 (X11)"
	@echo "make school-check  학교 fixture / 생성 SDF 일치 검사 (Docker 불필요)"
	@echo "make school-test  학교 데이터로 Rust/C++ 경로·회피 테스트 (Docker 불필요)"
	@echo "make school-test-live  일회용 DB에서 실제 경로·배송 API까지 검증"
	@echo "make school-view  http://127.0.0.1:8766/viewer/ 학교·드론·경로 시각화"
	@echo "make school-bridge  학교 RGB/depth/point cloud/pose → ROS 2"
	@echo "make school-scene  학교·저장된 경로·실시간 기체 위치 → RViz/Foxglove"
	@echo "make school-stack-check  실행 중인 스택의 실제 ROS 메시지·TF 검증"
	@echo "make rviz-headless  가상 X 화면에서 RViz 렌더링 (호스트 X11 불필요)"
	@echo "make agent     Micro XRCE-DDS Agent 실행 (ROS 2 <-> PX4)"
	@echo "make foxglove  Foxglove bridge 실행 (ws://localhost:8765)"
	@echo "make rviz      RViz2 실행"
	@echo "make bridge    Gazebo -> ROS 2 토픽 브릿지 실행"
	@echo "make spawn-obstacle MODEL=construction-barrel X=8 Y=0  샘플 장애물 배치"
	@echo "make topics    ROS 2 토픽 목록"
	@echo "make gz-topics Gazebo 토픽 목록"
	@echo "make down      컨테이너 중지"
	@echo "make clean     컨테이너 + 이미지 + 로그 삭제"

init:
	@test -f .env || cp .env.example .env
	@echo ".env 준비 완료"

x11:
	@xhost +local:root >/dev/null && echo "X11 접근 허용됨 (xhost +local:root)"

build:
	$(COMPOSE) build

up:
	$(COMPOSE) up -d

down:
	$(COMPOSE) down

restart:
	$(COMPOSE) restart

shell:
	$(COMPOSE) exec $(SERVICE) bash -l

px4:
	$(COMPOSE) exec $(SERVICE) bash -lc 'HEADLESS=0 px4-gazebo'

headless:
	$(COMPOSE) exec $(SERVICE) bash -lc 'HEADLESS=1 px4-gazebo -d'

school-check:
	python3 scripts/generate-school-world.py --check

school: school-check
	$(COMPOSE) exec -e NV_SCHOOL_SCENE=1 -e PX4_GZ_WORLD=school -e HEADLESS=$(HEADLESS) $(SERVICE) \
		/usr/local/bin/nv-entrypoint.sh bash -lc 'if [ "$$HEADLESS" = 1 ]; then px4-gazebo -d; else px4-gazebo; fi'

school-gui:
	python3 scripts/gazebo-gui.py

agent:
	$(ROS_EXEC) MicroXRCEAgent udp4 -p 8888

foxglove:
	$(ROS_EXEC) ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 \
		'asset_uri_allowlist:=["^file:///models/environments/school-demo/meshes/school_map[.]stl$$", "^file:///models/drones/x500_base/meshes/drone[.]stl$$"]'

rviz:
	$(ROS_EXEC) rviz2 -d /config/px4.rviz --ros-args -p use_sim_time:=true

rviz-headless:
	$(ROS_EXEC) env QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s '-screen 0 1440x1000x24' rviz2 -d /config/px4.rviz --ros-args -p use_sim_time:=true

bridge:
	$(ROS_EXEC) ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/config/gz_bridge.yaml

school-bridge:
	$(ROS_EXEC) ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/config/school_bridge.yaml

school-scene:
	$(ROS_EXEC) python3 /ros2_ws/src/nv_telemetry/nv_telemetry/school_scene.py --ros-args -p use_sim_time:=true -p mission:=$(MISSION)

school-stack-check:
	$(ROS_EXEC) python3 /scripts/verify-school-ros.py

spawn-obstacle:
	$(COMPOSE) exec $(SERVICE) bash -lc 'ros2 run ros_gz_sim create \
		-world "$${PX4_GZ_WORLD:-default}" \
		-name "nv-$(MODEL)" \
		-file "/models/props/$(MODEL)/model.sdf" \
		-x $(X) -y $(Y) -z $(Z)'

topics:
	$(ROS_EXEC) ros2 topic list

gz-topics:
	$(COMPOSE) exec $(SERVICE) gz topic -l

logs:
	$(COMPOSE) logs -f --tail=100

clean:
	$(COMPOSE) down --rmi local -v
	rm -rf logs/*.ulg

school-test:
	python3 scripts/test-school.py

school-test-live:
	python3 scripts/test-school.py --with-backend

school-view:
	@echo "브라우저에서 http://127.0.0.1:8766/viewer/ 접속 (계획 재생, 실시간 비행 아님)"
	python3 -m http.server 8766 --bind 127.0.0.1 --directory .
