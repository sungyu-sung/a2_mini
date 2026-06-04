#!/usr/bin/env python3
"""
dori_search_car.py  —  car 탐색/정렬 노드 (usb_car_undock.py 다음 단계 / 단독 테스트 가능)

[업그레이드된 동작]
  실행 즉시 로봇 온보드 카메라(OAK-D) + YOLO(best.pt)로 'car' 탐색.
   1) OBSERVE : 제자리 정지 후 탐지(트리거식). car 보이면 → CENTER, 안 보이면 → ROTATE.
   2) ROTATE  : 오른쪽(시계방향)으로 STEP_ANGLE 만큼 조금씩 회전 → 다시 OBSERVE.
   3) 누적 회전이 360°(원래 방향 복귀)에 도달했는데도 못 찾으면 → BEEP(삐뽀삐뽀) 후 정지.
   4) CENTER  : 발견한 car 의 바운딩박스가 화면 "가로 중앙"에 오도록 회전 정렬 → 완료 시 정지.

회전각 추적은 /robot2/odom 의 yaw 로 한다.
전제: cmd_vel 을 직접 발행하므로 Nav2 가 주행 중이 아니어야 함. 테스트는 로봇을 바닥에 두고.
실행: ros2 run car_mission dori_search_car
"""

import math
import os

import cv2
import rclpy
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

# 로봇캠(OAK-D) 모델 — car_mission 패키지 share에서 로드 (하드코딩 경로 제거)
MODEL_PATH = os.path.join(
    get_package_share_directory('car_mission'), 'models', 'robotview_best.pt')

IMAGE_TOPIC = '/robot2/oakd/rgb/preview/image_raw'
ODOM_TOPIC = '/robot2/odom'
CMDVEL_TOPIC = '/robot2/cmd_vel'
AUDIO_TOPIC = '/robot2/cmd_audio'
TARGET_CLASS = 'car'
CONFIDENCE = 0.90          # car 는 0.90 이상만 인정(오탐 억제). 탐지가 너무 안 되면 0.85로 낮추기

# --- 탐색(스텝 회전) ---
SEARCH_DIR = -1.0          # 오른쪽(시계방향) = angular.z 음수 (REP-103)
STEP_ANGLE_DEG = 30.0      # 한 스텝에 회전할 각도
SEARCH_ANG_SPEED = 0.3     # 스텝 회전 각속도 [rad/s] (천천히)
OBSERVE_FRAMES = 2         # 정지 관찰 중 car 가 이 프레임 수 이상 잡히면 발견 확정
OBSERVE_TIMEOUT = 1.0      # 정지 관찰 최대 시간 [s] (지나면 다음 스텝 회전)
FULL_CIRCLE_DEG = 360.0    # 누적 회전이 이만큼이면 한 바퀴 = 탐색 실패 → beep

# --- 센터링(bbox 중앙 정렬): 멈춤→측정→작은 스텝 회전 반복(펄스식) ---
CENTER_TOL = 0.40            # 정규화 오차 허용치(±40% — 매우 느슨; car가 화면 중앙부에만 들면 '중앙' 인정)
CENTER_TURN_SPEED = 0.12     # 미세 스텝 회전 각속도 [rad/s] (최대한 느리게; 마찰로 안 돌면 0.18~0.25로↑)
CENTER_STEP_GAIN_DEG = 10.0  # |err| 당 스텝 각도 [deg] (작게 → 미세 이동)
CENTER_STEP_MIN_DEG = 0.3    # 최소 스텝 각도 [deg]
CENTER_STEP_MAX_DEG = 4.0    # 최대 스텝 각도 [deg] (한 번에 크게 안 돌게)
CENTER_SETTLE_TICKS = 2      # 스텝 후 측정 전 정지 안정화 틱(잔여 흔들림 제거)
CENTER_HOLD_FRAMES = 3       # 측정 중 연속 허용오차 프레임이면 정렬 완료
CENTER_LOST_LIMIT = 12       # 센터링 중 연속 미탐지 이만큼이면 탐색 복귀

CONTROL_HZ = 10.0
SHOW_WINDOW = True


def ang_norm(a):
    """각도를 -pi..pi 로 정규화."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DoriSearchCar(Node):
    def __init__(self):
        super().__init__('dori_search_car')

        # True면 탐색 완료(CENTER 성공) 또는 한바퀴 탐색 실패(beep) 후 노드 자동 종료. 단독 테스트는 False.
        self.declare_parameter('exit_on_done', True)
        self.exit_on_done = bool(self.get_parameter('exit_on_done').value)
        self.finished = False   # True가 되면 main 루프가 spin 종료(프로세스 exit)

        self.model = YOLO(MODEL_PATH)
        self.bridge = CvBridge()

        self.latest_frame = None
        self.cur_yaw = None              # odom yaw [rad]

        # 상태: OBSERVE / ROTATE / CENTER / DONE / IDLE
        self.state = 'OBSERVE'
        self.hit = 0
        self.lost = 0
        self.center_phase = 'measure'    # 'measure'(정지·측정) | 'turning'(작은 스텝 회전)
        self.center_hold = 0             # 허용오차 내 연속 측정 수
        self.center_settle = 0           # 스텝 후 안정화 틱
        self.center_step_rad = 0.0       # 이번 스텝 회전량 [rad]
        self.center_dir = 0.0            # 스텝 회전 방향(+왼/−오른)
        self.center_turn_start_yaw = None
        self.center_cx = None            # 시각화용 최근 중심 x
        self.total_turned = 0.0          # 누적 회전량 [rad]
        self.rotate_start_yaw = None
        self.observe_start = self.now_s()
        self.last_log = 0.0              # 자가진단 로그 throttle

        # /robot2/odom 은 RELIABLE 로 발행됨 → 맞춰서 구독(확실한 수신 보장)
        odom_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Image, IMAGE_TOPIC, self.image_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, odom_qos)
        self.cmd_pub = self.create_publisher(Twist, CMDVEL_TOPIC, 10)
        self.audio_pub = self.create_publisher(AudioNoteVector, AUDIO_TOPIC, 10)
        self.create_timer(1.0 / CONTROL_HZ, self.control_loop)

        self.get_logger().info(f'dori_search_car 시작 — 오른쪽 스텝 탐색, model={MODEL_PATH}')

    # ---------- 유틸 ----------
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def image_cb(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge 변환 실패: {exc}')

    def odom_cb(self, msg):
        self.cur_yaw = yaw_from_quat(msg.pose.pose.orientation)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def publish_angular(self, wz):
        t = Twist()
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)

    def best_car_cx(self, result):
        """가장 신뢰도 높은 'car' 박스의 가로 중심 픽셀. 없으면 None."""
        best_cx, best_conf = None, -1.0
        for box in result.boxes:
            cid = int(box.cls[0])
            conf = float(box.conf[0])
            if str(result.names[cid]).lower() == TARGET_CLASS and conf > best_conf:
                x1, _, x2, _ = box.xyxy[0].tolist()
                best_cx, best_conf = (x1 + x2) / 2.0, conf
        return best_cx

    # ---------- 상태 전이 ----------
    def enter_observe(self):
        self.state = 'OBSERVE'
        self.hit = 0
        self.observe_start = self.now_s()

    def enter_rotate(self):
        self.state = 'ROTATE'
        self.rotate_start_yaw = self.cur_yaw   # None 이면 do_rotate 에서 보정

    def enter_center(self):
        self.state = 'CENTER'
        self.lost = 0
        self.center_phase = 'measure'
        self.center_hold = 0
        self.center_settle = CENTER_SETTLE_TICKS
        self.center_turn_start_yaw = None
        self.get_logger().info('car 발견 → 바운딩박스 중앙 정렬(CENTER)')

    def enter_done(self):
        self.state = 'DONE'
        self.publish_stop()
        self.get_logger().info('car 중앙 정렬 완료 → 정지(DONE)')
        if self.exit_on_done:
            self.get_logger().info('dori_search_car 종료(다음 단계로)')
            self.finished = True

    def enter_beep(self):
        self.state = 'IDLE'
        self.publish_stop()
        self.send_beep()
        self.get_logger().info(f'한 바퀴({math.degrees(self.total_turned):.0f}°) 탐색 완료 — car 없음 → 삐뽀삐뽀')
        if self.exit_on_done:
            self.get_logger().info('dori_search_car 종료(car 미발견이지만 다음 단계로)')
            self.finished = True

    def send_beep(self):
        msg = AudioNoteVector()
        msg.append = False
        msg.notes = [
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300000000)),
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300000000)),
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300000000)),
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300000000)),
        ]
        self.audio_pub.publish(msg)

    # ---------- 메인 루프 ----------
    def control_loop(self):
        if self.latest_frame is None:
            return
        frame = self.latest_frame
        result = self.model.predict(frame, conf=CONFIDENCE, verbose=False)[0]
        car_cx = self.best_car_cx(result)
        width = frame.shape[1]

        # --- 자가진단 로그 (2초마다): 무엇 때문에 안 도는지 즉시 파악용 ---
        now = self.now_s()
        if now - self.last_log >= 2.0:
            self.last_log = now
            self.get_logger().info(
                f'[진단] state={self.state} '
                f'frame={"O" if self.latest_frame is not None else "X"} '
                f'yaw={"O" if self.cur_yaw is not None else "X(odom 미수신)"} '
                f'car={"O" if car_cx is not None else "X"} '
                f'turned={math.degrees(self.total_turned):.0f}deg'
            )

        if self.state == 'OBSERVE':
            self.do_observe(car_cx)
        elif self.state == 'ROTATE':
            self.do_rotate()
        elif self.state == 'CENTER':
            self.do_center(car_cx, width)
        else:  # DONE / IDLE
            self.publish_stop()

        if SHOW_WINDOW:
            self.show(result, width, car_cx)

    def do_observe(self, car_cx):
        self.publish_stop()                  # 트리거 탐지를 위해 정지 상태로 관찰
        if car_cx is not None:
            self.hit += 1
            if self.hit >= OBSERVE_FRAMES:
                self.enter_center()
                return
        else:
            self.hit = 0
        if self.now_s() - self.observe_start >= OBSERVE_TIMEOUT:
            self.enter_rotate()

    def do_rotate(self):
        if self.cur_yaw is None:
            return                           # odom 대기
        if self.rotate_start_yaw is None:
            self.rotate_start_yaw = self.cur_yaw
        delta = abs(ang_norm(self.cur_yaw - self.rotate_start_yaw))
        if delta >= math.radians(STEP_ANGLE_DEG):
            self.total_turned += delta
            self.publish_stop()
            if self.total_turned >= math.radians(FULL_CIRCLE_DEG) - 0.01:
                self.enter_beep()
            else:
                self.enter_observe()
        else:
            self.publish_angular(SEARCH_DIR * SEARCH_ANG_SPEED)   # 오른쪽 회전

    def do_center(self, car_cx, width):
        if car_cx is None:
            self.lost += 1
            self.center_hold = 0
            self.publish_stop()
            if self.lost >= CENTER_LOST_LIMIT:
                self.get_logger().info('센터링 중 car 놓침 → 탐색 재개')
                self.enter_observe()
            return
        self.lost = 0
        self.center_cx = car_cx

        # --- turning: 측정 때 정한 "작은 각도"만큼만 회전 후 정지 ---
        if self.center_phase == 'turning':
            if self.cur_yaw is None or self.center_turn_start_yaw is None:
                return
            delta = abs(ang_norm(self.cur_yaw - self.center_turn_start_yaw))
            if delta >= self.center_step_rad:
                self.publish_stop()
                self.center_phase = 'measure'
                self.center_settle = CENTER_SETTLE_TICKS
            else:
                self.publish_angular(self.center_dir * CENTER_TURN_SPEED)
            return

        # --- measure: 정지 상태에서 깨끗한 프레임으로 오차 측정 ---
        self.publish_stop()
        if self.center_settle > 0:                 # 스텝 직후 잔여 흔들림 안정화
            self.center_settle -= 1
            return
        err = (car_cx - width / 2.0) / (width / 2.0)   # [-1,1], +면 car 가 오른쪽
        if abs(err) < CENTER_TOL:
            self.center_hold += 1
            if self.center_hold >= CENTER_HOLD_FRAMES:
                self.enter_done()
            return
        self.center_hold = 0

        # 오차에 비례한 "작은" 스텝 각도 산출 → 그만큼만 회전(odom 기준)
        step_deg = max(CENTER_STEP_MIN_DEG,
                       min(CENTER_STEP_MAX_DEG, abs(err) * CENTER_STEP_GAIN_DEG))
        self.center_step_rad = math.radians(step_deg)
        self.center_dir = -1.0 if err > 0 else 1.0   # +err(오른쪽) → 오른쪽 회전(-z)
        self.center_turn_start_yaw = self.cur_yaw
        self.center_phase = 'turning'

    def show(self, result, width, car_cx):
        try:
            vis = result.plot()
            h = vis.shape[0]
            cv2.line(vis, (width // 2, 0), (width // 2, h), (0, 255, 0), 1)   # 화면 중앙선
            band = int(CENTER_TOL * width / 2.0)                             # 허용오차 밴드
            cv2.line(vis, (width // 2 - band, 0), (width // 2 - band, h), (0, 160, 0), 1)
            cv2.line(vis, (width // 2 + band, 0), (width // 2 + band, h), (0, 160, 0), 1)
            cx_show = self.center_cx if (self.state == 'CENTER' and self.center_cx is not None) else car_cx
            if cx_show is not None:
                cv2.line(vis, (int(cx_show), 0), (int(cx_show), h), (0, 0, 255), 1)
            cv2.putText(vis, f'{self.state} turned={math.degrees(self.total_turned):.0f}deg',
                        (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow('dori car search', vis)
            cv2.waitKey(1)
        except cv2.error:
            pass

    def destroy_node(self):
        try:
            for _ in range(3):
                self.publish_stop()
        except Exception:  # noqa: BLE001
            pass
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DoriSearchCar()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
