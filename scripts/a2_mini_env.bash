#!/usr/bin/env bash
# =============================================================================
# a2_mini_ws 팀 공통 환경 (turtlebot4_ws 등 별도 워크스페이스에 의존하지 않음)
#
# ~/.bashrc 맨 아래에 아래 "한 줄"만 추가하면 됩니다:
#     source ~/a2_mini_ws/src/scripts/a2_mini_env.bash
#
# 적용되면 새 터미널마다 "✅ ..." 확인 메시지가 표시됩니다.
# turtlebot4 기능(nav/viz/teleop 등)은 install_deps.sh 로 apt 설치하면 동작합니다.
# =============================================================================

# (1) ROS2 Humble
source /opt/ros/humble/setup.bash && echo "✅ ROS2 humble sourced"

# (2) turtlebot4_ws 오버레이 (강의록 방식 소스 빌드 — install_deps.sh 로 생성)
if [ -f ~/turtlebot4_ws/install/setup.bash ]; then
  source ~/turtlebot4_ws/install/setup.bash
  echo "✅ turtlebot4_ws overlay sourced"
else
  echo "⚠️  turtlebot4_ws 없음 → turtlebot4 기능 미동작 (bash ~/a2_mini_ws/src/scripts/install_deps.sh)"
fi

# (3) a2_mini_ws 오버레이 (빌드 결과가 있을 때만 — 최초 빌드 전 에러 방지)
if [ -f ~/a2_mini_ws/install/setup.bash ]; then
  source ~/a2_mini_ws/install/setup.bash
  echo "✅ a2_mini_ws overlay sourced"
else
  echo "⚠️  a2_mini_ws 미빌드 (cd ~/a2_mini_ws && colcon build --symlink-install)"
fi

# (3) 팀 공통 도메인 ID — ★ 전원 동일해야 서로/로봇이 보입니다 ★
export ROS_DOMAIN_ID=2
echo "✅ ROS_DOMAIN_ID=$ROS_DOMAIN_ID (팀 공통)"

# (4) TurtleBot4 Discovery Server (있으면 자동 적용; 없으면 README 0-(5) 참고해 생성)
if [ -f /etc/turtlebot4_discovery/setup.bash ]; then
  source /etc/turtlebot4_discovery/setup.bash
  echo "✅ turtlebot4 discovery server sourced"
else
  echo "⚠️  /etc/turtlebot4_discovery/setup.bash 없음 → 로봇 통신 불가 (README 0-(5) 참고)"
fi

# (5) 멀티 기기 통신: localhost 전용 모드는 반드시 꺼둠 (★ 켜면 로봇/팀원과 통신 끊김)
export ROS_LOCALHOST_ONLY=0
echo "✅ ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY (다기기 통신 ON)"

# (6) TurtleBot4 시뮬레이션(Ignition/Gazebo) 버전
export IGNITION_VERSION=fortress

# (7) colcon 자동완성 (있으면)
[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ] && \
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# =============================================================================
# 편의 alias / 함수  (turtlebot4 기능은 ~/turtlebot4_ws 필요 — install_deps.sh 로 설치)
#   robot# = 로봇 번호 (예: 2 → /robot2)
# =============================================================================
alias ros-restart='ros2 daemon stop; ros2 daemon start'   # 토픽이 안 보일 때 데몬 재시작
alias sd='source /etc/turtlebot4_discovery/setup.bash'    # discovery 설정 다시 적용
alias ed='nano /etc/turtlebot4_discovery/setup.bash'      # discovery 설정 편집
alias vd='cat /etc/turtlebot4_discovery/setup.bash'       # discovery 설정 보기
alias ssh-robot='ssh ubuntu@172.30.1.1'                   # 로봇 SSH 접속

# 도킹 / 언도킹
dock() {
  [ -z "$1" ] && { echo "Usage: dock <robot#>"; return 1; }
  ros2 action send_goal "/robot$1/dock" irobot_create_msgs/action/Dock "{}"
}
undock() {
  [ -z "$1" ] && { echo "Usage: undock <robot#>"; return 1; }
  ros2 action send_goal "/robot$1/undock" irobot_create_msgs/action/Undock "{}"
}

# 내비게이션(Nav2) / 위치추정(localization) / 시각화(RViz)
nav() {
  [ -z "$1" ] && { echo "Usage: nav <robot#>"; return 1; }
  ros2 launch turtlebot4_navigation nav2.launch.py namespace:="/robot$1"
}
loc() {
  if [ -z "$1" ] || [ -z "$2" ]; then echo "Usage: loc <robot#> <map.yaml>"; return 1; fi
  ros2 launch turtlebot4_navigation localization.launch.py namespace:="/robot$1" map:="$2"
}
rv() {
  [ -z "$1" ] && { echo "Usage: rv <robot#>"; return 1; }
  ros2 launch turtlebot4_viz view_robot.launch.py namespace:="/robot$1"
}

# 키보드 텔레옵 (기본 robot2)
kbd() {
  local r="${1:-2}"
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:="/robot$r/cmd_vel"
}
