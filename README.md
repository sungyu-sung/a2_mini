# a2_mini

ROKEY-7 team a2 미니 프로젝트 — **ROS2 (Humble) + TurtleBot4** 워크스페이스의 소스 레포지토리입니다.

> 📌 **이 레포는 colcon 워크스페이스의 `src/` 폴더입니다 (Pattern A).**
> `build/`, `install/`, `log/`는 colcon이 생성하는 머신 종속 산출물이라 버전 관리하지 않습니다.
> 그래서 clone 할 때 `src` 폴더로 받습니다 (아래 온보딩 참고).

---

## 0. 사전 준비 (팀원 환경에서 최초 1회)

아래가 갖춰져 있어야 온보딩 명령이 그대로 동작합니다.

### (1) Git 설치 & 사용자 설정
```bash
sudo apt update && sudo apt install -y git
git config --global user.name "본인이름"
git config --global user.email "본인@example.com"
```

### (2) 레포 접근 권한 + 인증
- 이 레포는 **저장소 소유자(sungyu-sung)가 Collaborator로 초대**해야 push가 가능합니다.
  GitHub → 레포 → **Settings → Collaborators → Add people** 에서 팀원 계정 추가.
- 초대 수락 후, 팀원은 **인증 수단**이 필요합니다 (둘 중 하나):
  - **HTTPS + PAT**: GitHub → Settings → Developer settings → Personal access tokens 에서 토큰 발급.
    clone/push 시 비밀번호 칸에 이 토큰을 입력.
  - **SSH 키**: `ssh-keygen` 후 공개키를 GitHub에 등록하고, 아래 URL을
    `git@github.com:sungyu-sung/a2_mini.git` 로 바꿔 사용.

### (3) ROS2 Humble (베이스)
```bash
# ROS2 Humble 이 설치돼 있어야 합니다. (없으면 공식 가이드로 먼저 설치)
# 빌드 도구·turtlebot4 등 나머지 의존성은 온보딩의 install_deps.sh 가 처리합니다.
ls /opt/ros/humble/setup.bash && echo "ROS2 Humble OK"
```
> `source /opt/ros/humble/setup.bash` 를 비롯한 환경설정은 (4)의 공통 환경파일이 자동으로 해줍니다. 여기서 따로 `~/.bashrc` 에 적을 필요 없습니다.

### (4) `~/.bashrc` 설정 — 레포의 공통 환경파일 한 줄만 source (권장)

레포에 **팀 공통 환경파일** [`scripts/a2_mini_env.bash`](scripts/a2_mini_env.bash) 가 들어 있습니다.
별도 워크스페이스(`turtlebot4_ws` 등) 없이도 동작하도록 작성돼 있어, `~/.bashrc` 맨 아래에 **한 줄**만 추가하면 끝입니다:

```bash
source ~/a2_mini_ws/src/scripts/a2_mini_env.bash
```

→ 이 파일이 아래를 **자동 처리**합니다 (적용되면 새 터미널마다 `✅ ...` 확인 메시지 표시):

| 이 파일이 설정하는 것 | 구분 | 비고 |
|------|------|------|
| `source /opt/ros/humble/setup.bash` | **필수** | 없으면 `ros2: command not found` |
| `source ~/a2_mini_ws/install/setup.bash` | **필수** | 우리 패키지(`turtlebot4_beep`/`turtlebot4_image`). 가드(`[ -f ]`)로 최초 빌드 전에도 터미널 안 깨짐 |
| `export ROS_DOMAIN_ID=2` | **팀 공통(★)** | **모두 `2`**. 다르면 토픽이 하나도 안 보임 |
| `source ~/turtlebot4_ws/install/setup.bash` | 자동 | 강의록 방식으로 소스 빌드된 turtlebot4 패키지 오버레이 (install_deps.sh 가 생성) |
| `source /etc/turtlebot4_discovery/setup.bash` | **팀 공통(★)** | Discovery Server 설정. 전원 동일 파일 필요 (아래 (5) 참고) |
| `export ROS_LOCALHOST_ONLY=0` | **팀 공통(★)** | **반드시 꺼둠.** `1`이면 자기 PC 안에서만 통신 → 로봇·팀원 차단 |
| `ROS_SUPER_CLIENT`(자동) | 신경 안 써도 됨 | discovery 파일이 자동 설정(터미널=True/백그라운드=False). **통신과 무관**, 조회용 |
| `nav` / `loc` / `rv` / `dock` / `undock` / `kbd` 함수 | 편의 | 로봇 제어 단축 명령 (turtlebot4_ws 빌드 필요 → 온보딩 참고) |

> 💡 **turtlebot4 기능(nav/viz/teleop)** 은 강의록과 동일하게 **`~/turtlebot4_ws` 를 소스 빌드**해서 제공합니다.
> 온보딩의 `install_deps.sh` 가 clone+빌드까지 자동으로 하므로, **clone 한 사람은 누구나** 동일 환경이 됩니다.

> ⚠️ **주의:** 같은 터미널에서 `colcon build` 직후엔 오버레이 갱신을 위해
> `source ~/a2_mini_ws/install/setup.bash` 를 **한 번 더** 실행하세요(새 터미널은 위 한 줄로 자동 적용).

### (5) Discovery Server 설정 파일 (`/etc/turtlebot4_discovery/setup.bash`)

우리 팀은 **모두 같은 네트워크**에서 동일한 **Discovery Server**로 로봇·팀원이 서로를 찾습니다.
따라서 팀원 전원이 **아래 내용의 파일을 `/etc/turtlebot4_discovery/setup.bash` 에 동일하게** 갖고 있어야 합니다.

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
[ -t 0 ] && export ROS_SUPER_CLIENT=True || export ROS_SUPER_CLIENT=False
export ROS_DOMAIN_ID=2
export ROS_DISCOVERY_SERVER=";;192.168.107.102:11811;"   # 팀 공통 Discovery Server 주소
```

없으면 아래로 생성하세요 (`/etc` 는 시스템 폴더라 `sudo` 필요):

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

- `ROS_DISCOVERY_SERVER` 의 `192.168.107.102:11811` 는 **팀 공통 Discovery Server 주소**입니다(전원 동일).
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` : Discovery Server는 Fast DDS 전용 기능이라 **모두 Fast DDS로 고정**해야 함.
- `[ -t 0 ] && ROS_SUPER_CLIENT=True || ...=False` : **사람이 연 터미널이면 자동 `True`** → `ros2 topic list`·`rqt` 등에서 시스템 전체 노드/토픽이 보임. 백그라운드 노드면 `False`(가볍게). **통신 자체와는 무관**하니 그대로 두면 됨.
- 이 파일은 **시스템 전역(`/etc`) 설정이라 git 레포에는 포함되지 않습니다.** 각자 PC에 위처럼 직접 두세요.
- 편의 alias: `vd`(보기) / `ed`(편집) / `sd`(다시 source).

---

## 1. 온보딩 (clone → 의존성 → 빌드)

기본 터미널 경로가 `~/`든 어디든, 아래는 절대경로 기반이라 그대로 붙여넣으면 됩니다.
turtlebot4 기능은 **강의록과 동일하게 `~/turtlebot4_ws` 를 소스 빌드**하여 제공합니다 — 이 과정도 아래 스크립트가 자동으로 해줍니다.

> **선행 조건:** ROS2 **Humble** 이 미리 설치돼 있어야 합니다 (0-(3) 참고). 그리고 **로봇과 같은 네트워크**에 연결돼 있어야 합니다.

```bash
mkdir -p ~/a2_mini_ws
cd ~/a2_mini_ws
git clone https://github.com/sungyu-sung/a2_mini.git src

# 환경 셋업 (turtlebot4_ws 소스 clone+빌드 + a2_mini 의존성) — 최초 1회, 시간 좀 걸림
bash src/scripts/install_deps.sh

# a2_mini_ws 빌드
colcon build --symlink-install && source install/setup.bash

# ★ Discovery Server 설정 파일 생성 (없으면 로봇 통신 불가) — 최초 1회
#    install_deps.sh 가 자동으로 못 만드는 /etc 시스템 파일이라 직접 만들어야 합니다.
sudo mkdir -p /etc/turtlebot4_discovery
sudo tee /etc/turtlebot4_discovery/setup.bash > /dev/null <<'EOF'
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
[ -t 0 ] && export ROS_SUPER_CLIENT=True || export ROS_SUPER_CLIENT=False
export ROS_DOMAIN_ID=2
export ROS_DISCOVERY_SERVER=";;192.168.107.102:11811;"
EOF

# ~/.bashrc 에 환경 한 줄 추가 (최초 1회)
echo 'source ~/a2_mini_ws/src/scripts/a2_mini_env.bash' >> ~/.bashrc
source ~/.bashrc
```

> ℹ️ `install_deps.sh` 가 하는 일 (강의록 방식 그대로):
> 1. `~/turtlebot4_ws/src` 에 `turtlebot4`, `turtlebot4_simulator`, `turtlebot4_desktop`,
>    `turtlebot4_tutorials`, `m-explore-ros2`(explore_lite) 를 git clone
> 2. `rosdep` 으로 의존성 설치 → `colcon build --symlink-install`
> 3. a2_mini_ws 자체 의존성도 `rosdep` 으로 해결
>
> 위 6단계(clone → install_deps → build → **discovery 파일** → bashrc 한 줄)를 마치면, turtlebot4_ws 가 없던
> 팀원도 **우리와 100% 동일한 환경**이 됩니다. 환경변수·오버레이 source 는 `a2_mini_env.bash` 한 줄이 전부 처리합니다.
>
> **꼭 짚을 점:** ① ROS2 Humble 은 선행 설치(0-(3)), ② Discovery 파일은 `/etc` 시스템 파일이라 스크립트가 못 만들어
> 위 `sudo tee` 로 직접 생성해야 함, ③ 로봇과 같은 네트워크. 이 셋만 충족하면 나머지는 자동입니다.

### 1-1. TurtleBot4 환경(`~/turtlebot4_ws`) — 강의 안 들은 사람용

우리 팀 환경(`a2_mini_env.bash`)은 turtlebot4 기능(nav/viz/teleop 등)을 **소스 빌드한 `~/turtlebot4_ws`** 에서 가져옵니다.
강의를 안 들어 `turtlebot4_ws` 가 없으면 **둘 중 하나**로 설치하세요.

**방법 A — 자동 (권장):** 위 온보딩의 `install_deps.sh` 가 아래를 전부 자동으로 합니다.

**방법 B — 수동 (강의록 원문 그대로):**
```bash
mkdir -p ~/turtlebot4_ws/src
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4.git -b humble
git clone https://github.com/turtlebot/turtlebot4_simulator.git -b humble
git clone https://github.com/turtlebot/turtlebot4_desktop.git -b humble
git clone https://github.com/turtlebot/turtlebot4_tutorials.git
git clone https://github.com/robo-friends/m-explore-ros2.git

cd ~/turtlebot4_ws
sudo rosdep init        # 최초 1회만 (이미 했으면 생략)
rosdep update
rosdep install --from-path src -yi --rosdistro humble

source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

> 📌 **의존 관계 정리 (헷갈리기 쉬움):**
> - a2_mini **자체 패키지**(`turtlebot4_beep`/`turtlebot4_image`)는 **turtlebot4_ws 없이도 빌드**됩니다.
>   (쓰는 `irobot_create_msgs`·`cv_bridge` 등은 ROS2 Humble/apt 기본 제공)
> - 단 **로봇 제어 단축 명령**(`nav`/`rv`/`loc`/`dock`/`undock`/`kbd`)은 `turtlebot4_navigation`·`turtlebot4_viz` 를
>   쓰므로 **turtlebot4_ws 가 있어야 동작**합니다.
> - 그래서 **"팀과 100% 동일한 환경"을 위해 turtlebot4_ws 설치를 권장**합니다 (위 A 또는 B).

---

## 2. 디렉터리 구조

```
a2_mini_ws/              ← colcon 워크스페이스 (Git 추적 안 함)
├── src/                 ← ★ 이 레포지토리 (Git 루트) ★
│   ├── docs/            ← 설계 문서, 회의록 등
│   └── <패키지들>/       ← ROS2 패키지
├── build/   (gitignore)
├── install/ (gitignore)
└── log/     (gitignore)
```

---

## 3. 빌드 & 실행 (매번 작업할 때)

```bash
cd ~/a2_mini_ws
colcon build --symlink-install
source install/setup.bash      # 빌드한 그 터미널에서만 1회 (새 터미널은 자동)
```
- `--symlink-install`: Python 파일/리소스를 심볼릭 링크로 설치해, 코드 수정 후 재빌드 없이 반영됩니다(C++은 재빌드 필요).
- **새 터미널**은 0-(4)에서 `~/.bashrc` 에 추가한 공통 환경파일
  (`source ~/a2_mini_ws/src/scripts/a2_mini_env.bash`)이 **자동으로 오버레이를 source** 하므로 따로 칠 필요 없습니다.
- 단, **방금 `colcon build` 한 그 터미널**은 오버레이 갱신을 위해 `source install/setup.bash` 를 **한 번만** 해주세요
  (이미 열려 있던 터미널이라 자동 적용이 안 됨).

---

## 4. 패키지 생성

```bash
cd ~/a2_mini_ws/src
# Python
ros2 pkg create --build-type ament_python <패키지명>
# C++
ros2 pkg create --build-type ament_cmake <패키지명>
```
- 우리 패키지는 공통 접두사를 붙여 TurtleBot4 공식 패키지(`turtlebot4_*`)와 구분합니다.

---

## 5. 브랜치 전략 & 협업 워크플로우

### 규칙
- **`main` 브랜치에 직접 push 하지 않습니다.** `main`은 항상 빌드되는 안정 상태를 유지합니다.
- 모든 작업은 **개인 작업 브랜치**에서 하고, **Pull Request(PR)** 로만 `main`에 병합합니다.
- ⚠️ **예외:** 레포 소유자(**sungyu-sung**)는 branch protection 예외로 등록되어 `main` 직접 push가 가능합니다. 그 외 팀원은 위 PR 워크플로우를 따릅니다.

### 브랜치 이름 규칙
```
<type>/<이름>-<작업요약>
```
- `type`: `feature`(기능), `fix`(버그수정), `docs`(문서), `refactor`(리팩터링)
- 예시:
  - `feature/sungyu-navigation`
  - `fix/minji-lidar-timeout`
  - `docs/jihun-readme`

### 작업 흐름 (매 작업마다 반복)

```bash
# 1) 항상 최신 main에서 시작
cd ~/a2_mini_ws/src
git switch main
git pull origin main

# 2) 내 작업 브랜치 생성 (위 이름 규칙대로)
git switch -c feature/본인이름-작업요약

# 3) 작업 → 커밋 (작은 단위로 자주)
git add -A
git commit -m "feat: 작업 내용 요약"

# 4) 원격에 내 브랜치 push (-u는 최초 1회만)
git push -u origin feature/본인이름-작업요약

# 5) GitHub 웹에서 Pull Request 생성 → 팀원 리뷰 → main 병합

# 6) 병합 후 로컬 정리
git switch main
git pull origin main
git branch -d feature/본인이름-작업요약
```

### 커밋 메시지 컨벤션
```
<type>: <한 줄 요약>     예) feat: TurtleBot4 주행 노드 추가
```
- `feat`, `fix`, `docs`, `refactor`, `chore`, `test` 중 하나로 시작.

### 충돌(conflict)이 났을 때
```bash
git switch main && git pull origin main   # 최신 main 받기
git switch feature/내브랜치
git merge main                            # 내 브랜치에 main 반영, 충돌 해결 후 커밋
```

---

## 6. 자주 겪는 문제

| 증상 | 원인 / 해결 |
|------|-------------|
| `ros2: command not found` | 공통 환경파일 미적용 → `~/.bashrc` 에 `source ~/a2_mini_ws/src/scripts/a2_mini_env.bash` 있는지 확인 (0-(4)) |
| 내 패키지가 안 잡힘 | ① `~/.bashrc` 에 공통 환경파일 줄 누락, 또는 ② 빌드한 그 터미널이라 `source ~/a2_mini_ws/install/setup.bash` 한 번 필요 |
| `nav`/`rv` 등 로봇 명령 안 됨 | `turtlebot4_ws` 미설치 → `bash ~/a2_mini_ws/src/scripts/install_deps.sh` (1-1 참고) |
| 로봇 토픽이 안 보임 | `ROS_DOMAIN_ID` 불일치 / discovery 설정 누락 / `ros-restart`(데몬 재시작) |
| `Permission denied (push)` | Collaborator 미등록 or PAT/SSH 인증 문제 |
| `colcon: command not found` | `sudo apt install python3-colcon-common-extensions` |
