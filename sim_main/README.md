# sim_main

여러 패키지의 동작을 **순서대로 지휘하는 오케스트레이터**.
외부 신호를 받으면 P1 좌표로 이동한 뒤, 도착하면 파란 큐브 추적을 시작한다.

```
외부 트리거(/cctv_trigger)
      ↓
P1 좌표로 이동 (Nav2 navigate_to_pose)
      ↓ (도착)
파란 큐브 추적 시작 (/tracking_enable → tracking 노드)
```

> sim_main은 **기능 코드를 갖지 않는다.** 이동은 Nav2 액션, 추적은 `tracking` 패키지에
> 맡기고, 상태 전환만 지휘한다 (move_p1·tracking 패키지는 독립적으로 유지).

---

## 상태 머신

| 상태 | 설명 | 전환 조건 |
|------|------|-----------|
| `IDLE` | 외부 트리거 대기 | `/cctv_trigger`(Bool true) 수신 |
| `NAVIGATING` | P1으로 Nav2 이동 | 도착(액션 결과) |
| `TRACKING` | `/tracking_enable` True 발행 | (종료) |

---

## 실행

```bash
# 터미널 1 — 전체 (시뮬+큐브+localization+nav2+rviz+tracking+sim_main)
sim-env
ros2 launch sim_main sim_main.launch.py

# 터미널 2 — 언독
sim-robot-undock

# 터미널 3 — 큐브 wasd 조종 (추적 시작 후 큐브 움직이기)
sim-env
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/model/blue_cube/cmd_vel

# 터미널 4 — 외부 가짜신호 발사 (시나리오 시작!)
sim-env
ros2 topic pub --once /cctv_trigger std_msgs/msg/Bool "{data: true}"
```

**동작 흐름**
1. 터미널 4 신호 → 로봇이 P1으로 자율주행
2. P1 도착 → 추적 자동 시작 (정면에 파란 큐브)
3. 터미널 3에서 큐브를 움직이면 로봇이 따라감

> 초기 위치는 `localization_auto.yaml`로 자동 설정되어 **RViz 2D Pose Estimate 불필요**.
> `sim-env`, `sim-robot-undock`은 `~/.bashrc`의 시뮬레이션 alias.

---

## 파라미터 (sim_main_node)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `namespace` | `''` | 실로봇은 `robot2` |
| `trigger_topic` | `/cctv_trigger` | 외부 신호 토픽 (Bool) |
| `p1_x`, `p1_y` | -2.93, 6.91 | P1 목표 좌표 (map 기준) |
| `p1_yaw` | 4.10 | 도착 시 바라볼 방향 (큐브 쪽, rad) |

```bash
# 좌표/방향 바꿔 실행
ros2 launch sim_main sim_main.launch.py    # (노드 파라미터는 launch에서 조정)
```

---

## 구성

| 파일 | 역할 |
|------|------|
| `sim_main/sim_main_node.py` | 오케스트레이터 (상태머신) |
| `launch/sim_main.launch.py` | 시뮬+큐브브릿지+localization+nav2+rviz+tracking+sim_main 통합 |

### 의존 패키지 (조립 대상)
- `gazebo_simulation` — 방 구조 world(empty.sdf) + 파란 큐브
- `turtlebot4_navigation` — localization / nav2
- `move_p1` — nav2/localization 설정 재사용 (`nav2_camera.yaml`, `localization_auto.yaml`)
- `tracking` — 파란 큐브 추적 노드 (`/tracking_enable`로 on/off)

### 파란 큐브 (gazebo_simulation/worlds/empty.sdf)
- P1 도착 시 카메라에 보이는 위치 `(2.82, -2.13)`에 배치
- `VelocityControl` → `/model/blue_cube/cmd_vel`로 wasd 제어

---

## 실로봇 적용 시

- `namespace:=robot2`, `use_sim_time:=false`로 실행
- 초기 위치(`localization_auto.yaml`)는 실로봇 독 위치 기준으로 재측정 필요
- 카메라 HSV 색범위(`tracking_node.py`)는 실제 조명에 맞게 재튜닝
- P1 좌표는 실로봇 지도(`maps_real/robot2_map.yaml`) 기준으로 변경
