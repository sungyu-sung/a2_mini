# a2_mini_ws

ROS 2 Humble 기반 TurtleBot4 시뮬레이션 / 실로봇 워크스페이스.

포함 패키지:
- **gazebo_simulation** — TurtleBot4 Gazebo(Ignition) 시뮬레이션 (커스텀 방 world, 카메라/라이다/depth 브릿지)
- **make_map** — SLAM + 자율 탐색(explore_lite) + RViz 한 번에 실행
- **camera_avoid** — depth 카메라로 장애물 감지 후 목표지점까지 회피 주행

---

## 사전 준비

### 1. 의존 패키지 받기
```bash
cd ~/a2_mini_ws
vcs import src < dependencies.repos
```

### 2. 빌드
```bash
cd ~/a2_mini_ws
colcon build
source install/setup.bash
```

---

## 환경 분리 (중요)

실제 로봇은 `ROS_DOMAIN_ID=2` + Discovery Server를 쓰고, 시뮬레이션은 충돌을 막기 위해
`ROS_DOMAIN_ID=10`을 쓰는 별도 환경에서 돌립니다. `~/.bashrc`에 아래 함수/alias가 설정되어 있습니다.

```bash
# 시뮬레이션 환경 적용 함수
sim-env() {
  export ROS_DOMAIN_ID=10
  unset ROS_DISCOVERY_SERVER
  source ~/a2_mini_ws/install/setup.bash
  echo "✅ 시뮬레이션 환경 적용 (DOMAIN_ID=10)"
}
```

> 시뮬레이션을 쓰다가 실제 로봇으로 돌아갈 땐 `source ~/.bashrc` 한 번이면 됩니다.

---

## 🟦 시뮬레이션 사용법

### 등록된 alias (~/.bashrc)
```bash
# 기존 프로세스 정리 후 Gazebo + 로봇 + 센서 브릿지 실행
alias sim-gazebo='pkill -9 -f ros2; pkill -9 -f ign; pkill -9 -f parameter_bridge; sleep 3; sim-env && ros2 launch gazebo_simulation turtlebot4_empty_world.launch.py'

# 카메라 영상 뷰어
alias sim-camera='sim-env && ros2 run rqt_image_view rqt_image_view'

# 라이다 시각화 (RViz)
alias sim-lidar='sim-env && ros2 launch turtlebot4_viz view_robot.launch.py'

# 도킹 / 언도킹
alias sim-robot-undock='sim-env && ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"'
alias sim-robot-dock='sim-env && ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"'
```

### 실행 순서
```bash
# 터미널 1 — 시뮬레이션 (기존 프로세스 자동 정리)
sim-gazebo

# 터미널 2 — 언도킹 (로봇이 움직이려면 먼저 도크에서 분리)
sim-robot-undock

# 터미널 3 — 카메라 영상
sim-camera

# 터미널 4 — 라이다 RViz
sim-lidar
```

> ⚠️ `sim-gazebo`는 `gz_ros2_control`이 가끔 `robot_state_publisher`보다 먼저 떠서
> 무한 대기(`Waiting messages on topic [robot_description]`)에 빠질 수 있습니다.
> 그럴 땐 한 번 더 `sim-gazebo`를 실행하면 됩니다. (race condition)

### 카메라 회피 주행 (camera_avoid)
```bash
# 시뮬레이션 + 언도킹 후
sim-env
ros2 run camera_avoid camera_avoid
```
목표지점까지 직진하다가 depth 카메라에 가까운 장애물이 잡히면 회피합니다.

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `goal_x`, `goal_y` | -3.5, 0.0 | 목표 지점 (odom 기준) |
| `goal_tolerance` | 0.25 | 목표 도달 허용 오차(m) |
| `obstacle_min` | 0.2 | 장애물 판단 최소 거리(m) |
| `obstacle_max` | 1.0 | 이보다 멀면 장애물로 안 봄(먼 벽 무시) |
| `linear_speed` | 0.2 | 전진 속도 |
| `angular_speed` | 0.6 | 회전 속도 |

```bash
# 파라미터 바꿔 실행 예시
ros2 run camera_avoid camera_avoid --ros-args -p goal_x:=-3.0 -p obstacle_max:=1.2
```

---

## 🟥 실제 로봇 사용법 (robot2)

`~/.bashrc`에 등록된 alias:
```bash
alias robot-undock='ros2 action send_goal /robot2/undock irobot_create_msgs/action/Undock "{}"'
alias robot-dock='ros2 action send_goal /robot2/dock irobot_create_msgs/action/Dock "{}"'
alias robot-camera='ros2 run rqt_image_view rqt_image_view'
alias robot-view='ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot2'
alias robot-keyboard='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot2/cmd_vel'
alias robot-nav='ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot2'
alias robot-loc='ros2 launch turtlebot4_navigation localization.launch.py namespace:=robot2 map:=$HOME/Documents/student_maps/map.yaml'
```

실제 로봇 카메라 토픽: `/robot2/oakd/rgb/preview/image_raw`

### 카메라 회피 주행 (실로봇)
```bash
ros2 run camera_avoid camera_avoid --ros-args -p namespace:=robot2
```

---

## 🗺️ 지도 만들기 (make_map)

SLAM + 자율 탐색 + RViz를 한 번에 실행합니다.
```bash
# 실제 로봇 (robot2)
ros2 launch make_map make_map.launch.py

# 네임스페이스 변경
ros2 launch make_map make_map.launch.py namespace:=/robot1
```

다 돌아다닌 뒤 지도 저장:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/Documents/student_maps/my_map \
  --ros-args -p map_subscribe_transient_local:=true -r __ns:=/robot2
```

---

## 시뮬레이션 World / 토픽 정보

### 방 구조 (empty.sdf)
- 외벽으로 둘러싸인 10m × 8m 방 (벽 높이 0.5m)
- 중앙→동쪽 가로 칸막이 벽 + 가운데 박스 장애물
- 로봇 시작 위치: 오른쪽 상단 `(x=4.3, y=3.0, yaw=1.57)`

### 센서 토픽 (시뮬레이션, 네임스페이스 없음)
| 토픽 | 설명 |
|------|------|
| `/scan` | RPLidar 스캔 (바닥 기준 높이 약 0.193m) |
| `/oakd/rgb/preview/image_raw` | OAK-D RGB 영상 |
| `/oakd/rgb/preview/depth` | OAK-D depth 영상 (회피에 사용) |
| `/cmd_vel` | 주행 명령 |
| `/odom` | 위치/방향 |

---

## 패키지 구조

```
a2_mini_ws/src/
├── dependencies.repos                 # 외부 의존 저장소 목록
├── gazebo_simulation/
│   ├── launch/
│   │   ├── turtlebot4_empty_world.launch.py   # 메인: 시뮬+센서 브릿지
│   │   └── ignition.launch.py                 # 커스텀 gui.config 사용
│   ├── worlds/
│   │   └── empty.sdf                  # 방 구조 world
│   └── gui/
│       └── gui.config                # HMI/Teleop 없는 기본 3D 뷰
├── make_map/
│   └── launch/
│       └── make_map.launch.py        # SLAM + explore + RViz
└── camera_avoid/
    └── camera_avoid/
        └── camera_avoid_node.py      # depth 회피 주행 노드
```
