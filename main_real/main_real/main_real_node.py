"""
실로봇 오케스트레이터 (main_real 단독 동작, main_sim 비의존).

  외부 트리거(/robot2/cctv_trigger) 수신
    → 독에서 출발(초기위치 자동 설정 + undock)
    → P1으로 이동 (TurtleBot4Navigator)
    → 도착하면 추적 켜기 (/robot2/tracking_enable)

dock/undock/초기위치/이동은 TurtleBot4Navigator 헬퍼로 처리.
"""

import rclpy
from std_msgs.msg import Bool

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Navigator, TurtleBot4Directions,
)

# ── 실로봇 설정값 (실측) ──
NAMESPACE = '/robot2'
DOCK_XY = [-0.008, 0.052]   # 독 위치 (map 기준, 초기위치용)
DOCK_DEG = 0                # 독에서 바라보는 방향(도)
P1_XY = [-2.5104, 1.3341]   # P1 위치 (map 기준)
P1_DEG = -78.6              # P1 도착 방향(도)
TRIGGER_TOPIC = 'cctv_trigger'      # 노드 namespace 기준 상대 → /robot2/cctv_trigger
TRACK_TOPIC = 'tracking_enable'     # → /robot2/tracking_enable


def main():
    rclpy.init()
    nav = TurtleBot4Navigator(namespace=NAMESPACE)

    # ── 추적 on/off 발행, 트리거 구독 ──
    track_pub = nav.create_publisher(Bool, TRACK_TOPIC, 10)

    state = {'triggered': False}

    def trigger_cb(msg):
        if msg.data:
            state['triggered'] = True

    nav.create_subscription(Bool, TRIGGER_TOPIC, trigger_cb, 10)

    def set_track(on):
        m = Bool()
        m.data = on
        track_pub.publish(m)

    # ── 독에서 시작 + 초기위치 설정 ──
    if not nav.getDockedStatus():
        nav.info('독에 없음 — 도킹 후 초기화')
        nav.dock()

    nav.setInitialPose(nav.getPoseStamped(DOCK_XY, DOCK_DEG))
    nav.waitUntilNav2Active()
    set_track(False)   # 추적 꺼둠

    nav.info('main_real 준비 완료 — 외부 트리거 대기 (IDLE)')

    # ── 트리거 대기 ──
    while rclpy.ok() and not state['triggered']:
        rclpy.spin_once(nav, timeout_sec=0.1)

    if not rclpy.ok():
        return

    # ── P1으로 이동 ──
    nav.info('📨 외부 신호 수신 → undock 후 P1 이동')
    nav.undock()
    nav.startToPose(nav.getPoseStamped(P1_XY, P1_DEG))

    # 도착 대기 (isTaskComplete 내부에서 spin)
    while not nav.isTaskComplete():
        pass

    # ── 도착 → 추적 시작 ──
    nav.info('🎯 P1 도착 → 추적 시작 (TRACKING)')
    set_track(True)

    # 추적 노드가 도는 동안 유지
    rclpy.spin(nav)

    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
