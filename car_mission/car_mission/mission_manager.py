"""mission_manager — 미니프로젝트 핵심 상태머신 (PLACEHOLDER).

흐름:
  IDLE ──(CCTV가 'car' 감지)──> NAVIGATING ──(고정좌표 도착)──> SEARCHING
       ──(로봇캠으로 'car' 발견)──> APPROACHING ──(거리 <= 임계값)──> DONE

이 노드가 소유/제어하는 것:
  - /<ns>/cmd_vel 발행 (SEARCHING 회전, APPROACHING 전진/조향)
  - Nav2 NavigateToPose 액션 클라이언트 (고정좌표 이동)
구독:
  - /cctv/car_detected (std_msgs/Bool)          ← cctv_detector
  - /car/visible (Bool) /car/bearing (Float32) /car/distance (Float32) ← car_tracker

TODO: 각 상태의 제어 로직(아래 _tick_* 참고)을 구현.
"""
from enum import Enum

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # nav2 미설치 환경에서도 import는 통과 (TODO: 빌드 의존성 확인)
    NavigateToPose = None


class State(Enum):
    IDLE = 'IDLE'
    NAVIGATING = 'NAVIGATING'
    SEARCHING = 'SEARCHING'
    APPROACHING = 'APPROACHING'
    DONE = 'DONE'


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # ---- parameters (config/mission.yaml 로 오버라이드) ----
        self.declare_parameter('robot_namespace', '/robot2')
        self.declare_parameter('goal_x', 0.0)            # 고정좌표 (map 프레임)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('stop_distance_m', 0.5)   # 이 거리 이하로 접근하면 정지
        self.declare_parameter('search_angular_speed', 0.4)   # rad/s
        self.declare_parameter('approach_linear_speed', 0.15)  # m/s
        self.declare_parameter('yaw_kp', 0.005)          # 접근 시 조향 P 게인 (bearing→angular)
        self.declare_parameter('bearing_centered', 0.1)  # |bearing| 이 값 이하이면 정면으로 간주

        ns = self.get_parameter('robot_namespace').value
        self.stop_distance = float(self.get_parameter('stop_distance_m').value)
        self.search_w = float(self.get_parameter('search_angular_speed').value)
        self.approach_v = float(self.get_parameter('approach_linear_speed').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.bearing_centered = float(self.get_parameter('bearing_centered').value)

        # ---- I/O ----
        self.cmd_pub = self.create_publisher(Twist, f'{ns}/cmd_vel', 10)
        self.create_subscription(Bool, '/cctv/car_detected', self._on_cctv, 10)
        self.create_subscription(Bool, '/car/visible', self._on_visible, 10)
        self.create_subscription(Float32, '/car/bearing', self._on_bearing, 10)
        self.create_subscription(Float32, '/car/distance', self._on_distance, 10)

        self.nav_client = None
        if NavigateToPose is not None:
            self.nav_client = ActionClient(self, NavigateToPose, f'{ns}/navigate_to_pose')

        # ---- state ----
        self.state = State.IDLE
        self.cctv_triggered = False
        self.car_visible = False
        self.car_bearing = 0.0          # 정규화 수평 오프셋 (- 왼쪽 / + 오른쪽)
        self.car_distance = float('nan')  # m, 모르면 NaN
        self._nav_goal_sent = False

        self.create_timer(0.1, self._tick)  # 10 Hz 상태머신
        self.get_logger().info('mission_manager 시작 (PLACEHOLDER) — 상태: IDLE')

    # ---- callbacks ----
    def _on_cctv(self, msg):
        if msg.data:
            self.cctv_triggered = True

    def _on_visible(self, msg):
        self.car_visible = msg.data

    def _on_bearing(self, msg):
        self.car_bearing = msg.data

    def _on_distance(self, msg):
        self.car_distance = msg.data

    # ---- state machine ----
    def _tick(self):
        if self.state == State.IDLE:
            self._tick_idle()
        elif self.state == State.NAVIGATING:
            self._tick_navigating()
        elif self.state == State.SEARCHING:
            self._tick_searching()
        elif self.state == State.APPROACHING:
            self._tick_approaching()
        # DONE: 아무것도 안 함

    def _set_state(self, new_state):
        self.get_logger().info(f'상태 전이: {self.state.value} → {new_state.value}')
        self.state = new_state

    def _tick_idle(self):
        # CCTV가 차를 감지하면 출동
        if self.cctv_triggered:
            self._set_state(State.NAVIGATING)

    def _tick_navigating(self):
        # TODO: Nav2 NavigateToPose 로 고정좌표 전송, 결과(success) 콜백에서 SEARCHING 전이
        #   - goal_x/goal_y/goal_yaw 로 PoseStamped 구성 (frame_id='map')
        #   - self.nav_client.wait_for_server() / send_goal_async / get_result_async
        if not self._nav_goal_sent:
            self.get_logger().warn('TODO: Nav2 고정좌표 전송 미구현')
            self._nav_goal_sent = True

    def _tick_searching(self):
        # 제자리 회전하며 로봇캠 YOLO(car_tracker)로 차량 탐색
        if self.car_visible:
            self._stop()
            self._set_state(State.APPROACHING)
            return
        cmd = Twist()
        cmd.angular.z = self.search_w   # TODO: 한 바퀴 돌아도 못 찾으면 처리(타임아웃/실패)
        self.cmd_pub.publish(cmd)

    def _tick_approaching(self):
        # 차량을 화면 중앙에 유지하며 전진, 거리 임계값에서 정지
        if not (self.car_distance != self.car_distance):  # not NaN
            if self.car_distance <= self.stop_distance:
                self._stop()
                self._set_state(State.DONE)
                self.get_logger().info('차량 접근 완료 — 정지')
                return
        cmd = Twist()
        cmd.linear.x = self.approach_v if self.car_visible else 0.0
        cmd.angular.z = -self.yaw_kp * self.car_bearing  # TODO: 게인/부호 실측 튜닝
        # TODO: 차량을 잃으면(visible=False) SEARCHING 복귀 등 예외 처리
        self.cmd_pub.publish(cmd)

    def _stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
