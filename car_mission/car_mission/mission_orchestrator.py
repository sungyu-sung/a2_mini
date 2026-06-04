"""mission_orchestrator — 미션 전체 흐름 상태머신 + cmd_vel 중재.

흐름:
  WAIT_UNDOCK ─(usb_car_undock 도착)─> SEARCH ─(car 보임)─> APPROACH(Nav2로 1m 접근)
              ─(car가 움직임)─> TRACK(car_tracking 추격)
              ─(1m 도달 & 정지)─> DONE

cmd_vel 중재(★): 같은 순간 한 컨트롤러만 cmd_vel 을 잡도록 enable 신호를 상호배제로 관리.
  - APPROACH: /approach_enable=True,  tracking=False   (Nav2가 cmd_vel)
  - TRACK   : /approach_enable=False, tracking=True     (car_tracking이 cmd_vel)
  - 그 외   : 둘 다 False

인터페이스 (★ 표시는 아직 상류 노드가 발행/구현해야 함 — TODO):
  구독:
    /mission/undock_done (Bool)  ★ usb_car_undock 이 감시포인트 도착 시 발행하도록 추가 필요
    /car/visible (Bool)          ★ yolo_depth_detector 가 발행하도록 추가 필요(되돌린 버전엔 없음)
    /car/distance (Float32, m)   ★ 동상
    /approach_done (Bool)        ★ approach_controller(개발중) 가 1m 도달 시 발행
  발행:
    /approach_enable (Bool)              → approach_controller(개발중) on/off
    /<ns>/tracking_enable (Bool)         → car_tracking on/off
    /mission/state (String)              → 현재 상태(디버그/모니터링)
"""
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class State(Enum):
    WAIT_UNDOCK = 'WAIT_UNDOCK'
    SEARCH = 'SEARCH'
    APPROACH = 'APPROACH'
    TRACK = 'TRACK'
    DONE = 'DONE'


class MissionOrchestrator(Node):
    def __init__(self):
        super().__init__('mission_orchestrator')

        # ---- parameters ----
        self.declare_parameter('robot_namespace', '/robot2')
        self.declare_parameter('stop_distance', 1.0)     # car 와 유지 목표 거리(m)
        self.declare_parameter('distance_tol', 0.1)
        self.declare_parameter('move_step', 0.15)        # 한 틱(0.2s)에 거리가 이만큼↑이면 움직임 의심
        self.declare_parameter('move_hits', 3)           # 연속 N회면 '차 움직임' 확정 → TRACK
        self.declare_parameter('lost_timeout', 3.0)      # car 안 보인 지 이 시간이면 SEARCH 복귀
        self.declare_parameter('wait_undock', True)      # False면 SEARCH부터 시작(테스트용)

        ns = self.get_parameter('robot_namespace').value.strip('/')
        prefix = f'/{ns}' if ns else ''
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.distance_tol = float(self.get_parameter('distance_tol').value)
        self.move_step = float(self.get_parameter('move_step').value)
        self.move_hits = int(self.get_parameter('move_hits').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout').value)
        wait_undock = bool(self.get_parameter('wait_undock').value)

        # ---- I/O ----
        self.approach_pub = self.create_publisher(Bool, '/approach_enable', 10)
        self.tracking_pub = self.create_publisher(Bool, f'{prefix}/tracking_enable', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)

        self.create_subscription(Bool, '/mission/undock_done', self._on_undock_done, 10)
        self.create_subscription(Bool, '/car/visible', self._on_visible, 10)
        self.create_subscription(Float32, '/car/distance', self._on_distance, 10)
        self.create_subscription(Bool, '/approach_done', self._on_approach_done, 10)

        # ---- state ----
        self.undock_done = False
        self.car_visible = False
        self.car_distance = float('nan')
        self.approach_done = False
        self.last_seen = self.get_clock().now()
        self.prev_distance = None
        self.move_count = 0

        self.state = None
        self._set_state(State.SEARCH if not wait_undock else State.WAIT_UNDOCK)
        self.create_timer(0.2, self._tick)
        self.get_logger().info('mission_orchestrator 시작')

    # ---- callbacks ----
    def _on_undock_done(self, msg):
        if msg.data:
            self.undock_done = True

    def _on_visible(self, msg):
        self.car_visible = msg.data
        if msg.data:
            self.last_seen = self.get_clock().now()

    def _on_distance(self, msg):
        self.car_distance = msg.data

    def _on_approach_done(self, msg):
        if msg.data:
            self.approach_done = True

    # ---- state machine ----
    def _set_state(self, new_state):
        if new_state == self.state:
            return
        self.get_logger().info(f'상태 전이: {self.state.value if self.state else "-"} → {new_state.value}')
        self.state = new_state
        self.state_pub.publish(String(data=new_state.value))

        # cmd_vel 중재: 상태별로 컨트롤러 enable 을 상호배제로 설정
        approach = (new_state == State.APPROACH)
        tracking = (new_state == State.TRACK)
        self.approach_pub.publish(Bool(data=approach))
        self.tracking_pub.publish(Bool(data=tracking))
        if new_state in (State.APPROACH, State.TRACK):
            self.move_count = 0
            self.prev_distance = None

    @staticmethod
    def _valid(d):
        return d == d  # not NaN

    def _car_lost(self):
        dt = (self.get_clock().now() - self.last_seen).nanoseconds * 1e-9
        return (not self.car_visible) and dt > self.lost_timeout

    def _detect_moving(self):
        """접근 중 거리가 급히 늘면(차가 멀어짐) 움직임으로 판단. (TODO: 측면 이동은 거리만으론 못 잡음 — 향후 car 위치/bearing 사용)"""
        if not self._valid(self.car_distance):
            return False
        if self.prev_distance is not None and \
                (self.car_distance - self.prev_distance) > self.move_step:
            self.move_count += 1
        else:
            self.move_count = 0
        self.prev_distance = self.car_distance
        return self.move_count >= self.move_hits

    def _tick(self):
        if self.state == State.WAIT_UNDOCK:
            # usb_car_undock 가 감시포인트 도착을 알리면 인지 단계로
            if self.undock_done:
                self._set_state(State.SEARCH)

        elif self.state == State.SEARCH:
            # 감시포인트에서 car 가 보이길 대기(수동). 보이면 접근 시작.
            if self.car_visible:
                self._set_state(State.APPROACH)

        elif self.state == State.APPROACH:
            # Nav2(approach_controller)가 1m까지 접근 중. 그 사이 차가 움직이면 추적으로 전환.
            if self._detect_moving():
                self.get_logger().info('차 움직임 감지 → TRACK 전환')
                self._set_state(State.TRACK)
            elif self.approach_done or \
                    (self._valid(self.car_distance) and
                     self.car_distance <= self.stop_distance + self.distance_tol):
                self.get_logger().info('1m 접근 완료 → DONE')
                self._set_state(State.DONE)
            elif self._car_lost():
                self.get_logger().warn('접근 중 car 소실 → SEARCH 복귀')
                self._set_state(State.SEARCH)

        elif self.state == State.TRACK:
            # car_tracking 이 차를 1m 유지하며 추격(reactive). 차가 멈춰 1m에서 안정되면 종료 가능.
            # (지금은 추격 유지가 챌린지 목표 — 별도 종료조건 없이 TRACK 유지)
            pass

        # DONE: 정지 유지(두 enable 모두 False)


def main(args=None):
    rclpy.init(args=args)
    node = MissionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
