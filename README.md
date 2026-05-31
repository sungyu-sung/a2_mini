# a2_mini

ROKEY-7 team a2 미니 프로젝트 — ROS2 + TurtleBot4 워크스페이스의 소스 레포지토리입니다.

> **이 레포는 colcon 워크스페이스의 `src/` 폴더입니다 (Pattern A).**
> `build/`, `install/`, `log/`는 colcon이 생성하는 머신 종속 산출물이라 버전 관리하지 않습니다.

## 디렉터리 구조

```
a2_mini_ws/              ← colcon 워크스페이스 (Git 추적 안 함)
├── src/                 ← ★ 이 레포지토리 (Git 루트) ★
│   ├── docs/            ← 설계 문서, 회의록 등
│   └── <패키지들>/       ← ROS2 패키지
├── build/   (gitignore)
├── install/ (gitignore)
└── log/     (gitignore)
```

## 처음 받는 사람 (팀원 온보딩)

```bash
mkdir -p ~/a2_mini_ws
cd ~/a2_mini_ws
git clone https://github.com/sungyu-sung/a2_mini.git src
colcon build
source install/setup.bash
```

## 빌드 & 실행

```bash
cd ~/a2_mini_ws
colcon build --symlink-install
source install/setup.bash
```

## 패키지 생성 예시

```bash
cd ~/a2_mini_ws/src
# Python
ros2 pkg create --build-type ament_python <패키지명>
# C++
ros2 pkg create --build-type ament_cmake <패키지명>
```

## 컨벤션

- 우리 패키지는 공통 접두사를 붙여 TurtleBot4 공식 패키지(`turtlebot4_*`)와 구분합니다.
- 작업은 브랜치에서 진행하고 PR로 병합합니다.
