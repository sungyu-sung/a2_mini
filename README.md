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

### (3) ROS2 Humble + colcon 설치
```bash
# ROS2 Humble이 이미 설치돼 있다고 가정. 빌드 도구만 확인:
sudo apt install -y python3-colcon-common-extensions

# ~/.bashrc 에 아래가 있는지 확인 (없으면 추가):
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

## 1. 온보딩 (이 4줄이면 끝)

기본 터미널 경로가 `~/`든 어디든, 아래는 절대경로 기반이라 그대로 붙여넣으면 됩니다.

```bash
mkdir -p ~/a2_mini_ws
cd ~/a2_mini_ws
git clone https://github.com/sungyu-sung/a2_mini.git src
colcon build --symlink-install && source install/setup.bash
```

> ℹ️ 현재 `src/` 안에 패키지가 없으면 `colcon build`는 "빌드할 패키지 0개"로 정상 통과합니다(에러 아님).
> 패키지가 추가되면 그때부터 실제 빌드가 일어납니다.

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
source install/setup.bash
```
- `--symlink-install`: Python 파일/리소스를 심볼릭 링크로 설치해, 코드 수정 후 재빌드 없이 반영됩니다(C++은 재빌드 필요).
- 새 터미널을 열 때마다 `source ~/a2_mini_ws/install/setup.bash` 를 해야 워크스페이스 패키지가 잡힙니다.

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
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` 안 됨 → `~/.bashrc` 확인 |
| 내 패키지가 안 잡힘 | `source ~/a2_mini_ws/install/setup.bash` 누락 |
| `Permission denied (push)` | Collaborator 미등록 or PAT/SSH 인증 문제 |
| `colcon: command not found` | `sudo apt install python3-colcon-common-extensions` |
