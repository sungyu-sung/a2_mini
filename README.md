# a2_mini

ROKEY-7 team a2 미니 프로젝트 — **ROS2 Humble + TurtleBot4** 워크스페이스의 소스 레포입니다.

> 이 레포는 colcon 워크스페이스의 **`src/` 폴더**입니다. `build/`·`install/`·`log/`는 머신 종속 산출물이라 추적하지 않으며, clone 시 `src` 폴더로 받습니다.

---

## 0. 사전 준비 (최초 1회)

### (1) ROS2 Humble
미리 설치돼 있어야 합니다. 확인:
```bash
ls /opt/ros/humble/setup.bash && echo "ROS2 Humble OK"
```

### (2) Git & 레포 권한
- `git` 설치 후 `user.name`/`user.email` 설정.
- push 하려면 소유자(**sungyu-sung**)가 Collaborator로 초대해야 하며, HTTPS+PAT 또는 SSH 키 인증이 필요합니다.

### (3) Discovery Server 설정 (`/etc/turtlebot4_discovery/setup.bash`)
팀 전원이 같은 네트워크에서 동일한 Discovery Server로 로봇·팀원을 찾습니다. `/etc` 시스템 파일이라 git에 포함되지 않으니 각자 생성하세요:
```bash
sudo mkdir -p /etc/turtlebot4_discovery
sudo tee /etc/turtlebot4_discovery/setup.bash > /dev/null <<'EOF'
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
[ -t 0 ] && export ROS_SUPER_CLIENT=True || export ROS_SUPER_CLIENT=False
export ROS_DOMAIN_ID=2
export ROS_DISCOVERY_SERVER=";;192.168.107.102:11811;"
EOF
```

### (4) `~/.bashrc` 환경 설정
**아래 블록을 `~/.bashrc` 맨 아래에 추가하면** 환경·오버레이·로봇 단축 명령이 모두 동작합니다.
```bash
# ===== a2_mini_ws 팀 공통 환경 =====
source /opt/ros/humble/setup.bash && echo "✅ ROS2 humble"
[ -f ~/turtlebot4_ws/install/setup.bash ] && source ~/turtlebot4_ws/install/setup.bash && echo "✅ turtlebot4_ws overlay"
[ -f ~/a2_mini_ws/install/setup.bash ]    && source ~/a2_mini_ws/install/setup.bash && echo "✅ a2_mini_ws overlay"
[ -f /etc/turtlebot4_discovery/setup.bash ] && source /etc/turtlebot4_discovery/setup.bash && echo "✅ discovery server"

export ROS_DOMAIN_ID=2          # ★ 팀 전원 동일 (다르면 토픽이 안 보임)
export ROS_LOCALHOST_ONLY=0     # ★ 0 유지 (1이면 로봇·팀원 통신 차단)
export IGNITION_VERSION=fortress
echo "✅ ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ] && \
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# discovery 설정 편의 / 로봇 SSH
alias sd='source /etc/turtlebot4_discovery/setup.bash'
alias ed='nano /etc/turtlebot4_discovery/setup.bash'
alias vd='cat /etc/turtlebot4_discovery/setup.bash'
alias ros-restart='ros2 daemon stop; ros2 daemon start'   # 토픽이 안 보일 때
alias ssh-robot='ssh ubuntu@172.30.1.1'

# 로봇 제어 단축 (인자 = 로봇 번호, 예: nav 2 → /robot2)
dock()   { ros2 action send_goal "/robot$1/dock"   irobot_create_msgs/action/Dock   "{}"; }
undock() { ros2 action send_goal "/robot$1/undock" irobot_create_msgs/action/Undock "{}"; }
nav()    { ros2 launch turtlebot4_navigation nav2.launch.py namespace:="/robot$1"; }
loc()    { ros2 launch turtlebot4_navigation localization.launch.py namespace:="/robot$1" map:="$2"; }
rv()     { ros2 launch turtlebot4_viz view_robot.launch.py namespace:="/robot$1"; }
kbd()    { ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:="/robot${1:-2}/cmd_vel"; }
```
- `ROS_DOMAIN_ID`·`ROS_LOCALHOST_ONLY`·discovery 설정은 **팀 전원 동일해야** 통신됩니다.
- 오버레이 source는 `[ -f ]` 가드가 있어 빌드 전이라도 터미널이 깨지지 않습니다.
- `nav`/`loc`/`rv`/`dock`/`undock`/`kbd` 는 `turtlebot4_ws`(아래 온보딩의 `install_deps.sh`로 설치)가 있어야 동작합니다.

---

## 1. 온보딩 (clone → 의존성 → 빌드)

선행 조건: ROS2 Humble 설치(0-1), 로봇과 같은 네트워크.
```bash
mkdir -p ~/a2_mini_ws && cd ~/a2_mini_ws
git clone https://github.com/sungyu-sung/a2_mini.git src

# turtlebot4_ws 소스 clone+빌드 + a2_mini 의존성 (최초 1회, 시간 소요)
bash src/scripts/install_deps.sh

# 빌드
colcon build --symlink-install && source install/setup.bash
```
이후 0-(3) discovery 파일과 0-(4) bashrc 블록을 추가하면 셋업 완료입니다.

> `install_deps.sh` 는 `~/turtlebot4_ws`에 `turtlebot4`·`turtlebot4_simulator`·`turtlebot4_desktop`·`turtlebot4_tutorials`·`m-explore-ros2`를 clone → `rosdep` → `colcon build` 하고, a2_mini 의존성도 `rosdep`으로 해결합니다. a2_mini 자체 패키지는 turtlebot4_ws 없이도 빌드되지만, 로봇 제어 단축 명령은 turtlebot4_ws가 필요합니다.

---

## 2. 디렉터리 구조
```
a2_mini_ws/              ← colcon 워크스페이스 (Git 추적 안 함)
├── src/                 ← ★ 이 레포 (Git 루트)
│   ├── docs/            ← 문서
│   └── <패키지들>/
├── build/   (gitignore)
├── install/ (gitignore)
└── log/     (gitignore)
```

---

## 3. 빌드 & 실행
```bash
cd ~/a2_mini_ws
colcon build --symlink-install
source install/setup.bash      # 빌드한 그 터미널에서만 1회 (새 터미널은 자동)
```
- `--symlink-install`: Python 코드 수정은 재빌드 없이 반영(C++은 재빌드 필요).
- 새 터미널은 bashrc 블록이 오버레이를 자동 source 하므로 그대로 사용 가능합니다.

---

## 4. 패키지

| 패키지 | 실행 노드 | 설명 |
|--------|-----------|------|
| `turtlebot4_beep` | `beep_node`, `beep_node_empirestateofmind` | 로봇 스피커 비프/멜로디 |
| `turtlebot4_image` | `image_publisher`/`image_subscriber`, `data_publisher`/`data_subscriber` | 카메라·센서 데이터 중계/시각화 |
| `turtlebot4_yolo` | `yolo_detector`, `yolo_viewer` | 카메라 영상에 YOLO(`best.pt`) bounding box 발행/구독 |
| `rokey_pjt` | `depth_checker`, `depth_checker_click` | OAK-D stereo depth 거리 확인 |

패키지 생성:
```bash
cd ~/a2_mini_ws/src
ros2 pkg create --build-type ament_python <패키지명>   # C++은 ament_cmake
```
이름은 TurtleBot4 공식 패키지와 겹치지 않게 짓습니다.

---

## 5. 브랜치 & PR 워크플로우

- **`main` 직접 push 금지** — 작업 브랜치에서 PR로만 병합 (소유자 sungyu-sung은 예외).
- 브랜치 이름: `<type>/<이름>-<요약>` (`feature`/`fix`/`docs`/`refactor`).

```bash
git switch main && git pull origin main
git switch -c feature/본인이름-작업요약
git add -A && git commit -m "feat: 작업 요약"
git push -u origin feature/본인이름-작업요약
# GitHub에서 PR 생성 → 리뷰 → 병합
```
커밋 메시지: `<type>: <요약>` (`feat`/`fix`/`docs`/`refactor`/`chore`/`test`).

---

## 6. 트러블슈팅

| 증상 | 해결 |
|------|------|
| `ros2: command not found` | bashrc 블록(0-4) 누락 → 추가 후 `source ~/.bashrc` |
| 내 패키지가 안 잡힘 | 빌드한 그 터미널에서 `source install/setup.bash` 1회 |
| `nav`/`rv` 등 로봇 명령 안 됨 | `turtlebot4_ws` 미설치 → `bash src/scripts/install_deps.sh` |
| 로봇 토픽이 안 보임 | `ROS_DOMAIN_ID` 불일치 / discovery 누락 / `ros-restart` |
| `Permission denied (push)` | Collaborator 미등록 또는 PAT/SSH 인증 |
| `colcon: command not found` | `sudo apt install python3-colcon-common-extensions` |
