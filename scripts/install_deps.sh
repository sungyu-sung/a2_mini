#!/usr/bin/env bash
# =============================================================================
# a2_mini_ws 환경 셋업 (clone 후 최초 1회)
#
#   사용:  bash ~/a2_mini_ws/src/scripts/install_deps.sh
#
# turtlebot4 기능은 강의록과 동일하게 ~/turtlebot4_ws 를 "소스로 clone + colcon build"
# 하여 제공합니다 (m-explore/explore_lite 포함). 그 위에 a2_mini_ws 자체 의존성도 해결합니다.
# 이미 turtlebot4_ws 가 있으면 clone 은 건너뛰고 빌드만 갱신합니다.
# =============================================================================
set -e

A2_WS="${A2_MINI_WS:-$HOME/a2_mini_ws}"
TB4_WS="$HOME/turtlebot4_ws"

source /opt/ros/humble/setup.bash

echo "==> [1/5] turtlebot4_ws 생성 & 소스 clone (강의록 방식)"
mkdir -p "$TB4_WS/src"
cd "$TB4_WS/src"
[ -d turtlebot4 ]            || git clone https://github.com/turtlebot/turtlebot4.git -b humble
[ -d turtlebot4_simulator ] || git clone https://github.com/turtlebot/turtlebot4_simulator.git -b humble
[ -d turtlebot4_desktop ]   || git clone https://github.com/turtlebot/turtlebot4_desktop.git -b humble
[ -d turtlebot4_tutorials ] || git clone https://github.com/turtlebot/turtlebot4_tutorials.git
[ -d m-explore-ros2 ]       || git clone https://github.com/robo-friends/m-explore-ros2.git

echo "==> [2/5] rosdep init / update"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

echo "==> [3/5] turtlebot4_ws 의존성 설치 (rosdep)"
cd "$TB4_WS"
rosdep install --from-path src -yi --rosdistro humble

echo "==> [4/5] turtlebot4_ws 빌드 (최초엔 시간이 좀 걸립니다)"
colcon build --symlink-install

echo "==> [5/5] a2_mini_ws 자체 의존성 설치 (rosdep)"
rosdep install --from-paths "$A2_WS/src" --ignore-src -r -y --rosdistro humble || true

echo
echo "==> 완료! 이제 a2_mini_ws 를 빌드하세요:"
echo "      cd $A2_WS && colcon build --symlink-install && source install/setup.bash"
echo
echo "    새 터미널을 열거나 'source ~/.bashrc' 하면 turtlebot4_ws + a2_mini_ws 가"
echo "    모두 자동으로 잡힙니다 (a2_mini_env.bash 가 둘 다 source)."
