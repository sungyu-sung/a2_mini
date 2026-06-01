# a2_mini_ws

ROS 2 Humble 기반 TurtleBot4 시뮬레이션 워크스페이스.

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

## 시뮬레이션 실행

### 빈 world에 TurtleBot4 띄우기

```bash
source ~/a2_mini_ws/install/setup.bash
ros2 launch gazebo_simulation turtlebot4_empty_world.launch.py
```

Gazebo가 실행되면서 TurtleBot4 로봇과 충전 도크가 빈 평지에 소환됩니다.  
오른쪽 패널에 **TurtleBot4 HMI**(로봇 상태)와 **Teleop**(이동 조종) 창이 뜹니다.

---

## 카메라 영상 보기

시뮬레이션이 실행 중인 상태에서 새 터미널을 열고:

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

상단 드롭다운에서 다음 토픽을 선택하면 카메라 화면이 출력됩니다:

| 토픽 | 설명 |
|------|------|
| `/oakd/rgb/preview/image_raw` | OAK-D 카메라 RGB 영상 |

---

## 키보드로 로봇 조종

Teleop 패널의 **KEYBOARD** 탭을 사용하거나, 새 터미널에서:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

| 키 | 동작 |
|----|------|
| `i` | 전진 |
| `,` | 후진 |
| `j` | 좌회전 |
| `l` | 우회전 |
| `k` | 정지 |

---

## 센서 정보

### LiDAR (RPLidar A1)
- 바닥 기준 높이: **약 19.3 cm (0.193 m)**
- 토픽: `/scan`

### OAK-D 카메라
- RGB 영상 토픽: `/oakd/rgb/preview/image_raw`
- Depth 토픽: `/oakd/stereo/image_raw`

---

## 패키지 구조

```
a2_mini_ws/
├── dependencies.repos          # 외부 의존 저장소 목록
├── src/
│   └── gazebo_simulation/
│       ├── launch/
│       │   └── turtlebot4_empty_world.launch.py   # 메인 launch 파일
│       └── worlds/
│           └── empty.sdf       # 빈 평지 world
```
