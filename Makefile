COMPOSE := docker compose --profile sim
SERVICE := sim

.PHONY: help init build up down restart shell x11 px4 headless agent foxglove rviz bridge topics gz-topics logs clean

help:
	@echo "make init      .env 생성 (없을 때만)"
	@echo "make x11       호스트 X11 접근 허용 (GUI 사용 시 1회)"
	@echo "make build     시뮬레이션 이미지 빌드"
	@echo "make up        컨테이너 기동 (백그라운드)"
	@echo "make shell     컨테이너 셸 접속"
	@echo "make px4       PX4 SITL + Gazebo 실행 (GUI)"
	@echo "make headless  PX4 SITL + Gazebo 실행 (헤드리스)"
	@echo "make agent     Micro XRCE-DDS Agent 실행 (ROS 2 <-> PX4)"
	@echo "make foxglove  Foxglove bridge 실행 (ws://localhost:8765)"
	@echo "make rviz      RViz2 실행"
	@echo "make bridge    Gazebo -> ROS 2 토픽 브릿지 실행"
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
	$(COMPOSE) exec $(SERVICE) bash -lc 'HEADLESS=1 px4-gazebo'

agent:
	$(COMPOSE) exec $(SERVICE) MicroXRCEAgent udp4 -p 8888

foxglove:
	$(COMPOSE) exec $(SERVICE) ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

rviz:
	$(COMPOSE) exec $(SERVICE) rviz2 -d /config/px4.rviz

bridge:
	$(COMPOSE) exec $(SERVICE) ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/config/gz_bridge.yaml

topics:
	$(COMPOSE) exec $(SERVICE) ros2 topic list

gz-topics:
	$(COMPOSE) exec $(SERVICE) gz topic -l

logs:
	$(COMPOSE) logs -f --tail=100

clean:
	$(COMPOSE) down --rmi local -v
	rm -rf logs/*.ulg
