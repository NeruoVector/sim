# ROS 2 워크스페이스

컨테이너 안의 `/ros2_ws` 로 마운트됩니다.

## 패키지

| 패키지 | 설명 |
| --- | --- |
| `nv_telemetry` | PX4 `/fmu/out/*` 토픽을 구독해 상태를 로그로 출력하는 예제 |

## 빌드

호스트에서 컨테이너를 띄운 뒤:

```bash
make shell
colcon build --symlink-install
source install/setup.bash
```

또는 한 줄로:

```bash
docker compose exec sim bash -lc 'colcon build --symlink-install && source install/setup.bash'
```

빌드 후 셸을 새로 열면 엔트리포인트가 `install/setup.bash` 를 자동으로 소싱합니다.

## 토픽 확인

```bash
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_status
```
