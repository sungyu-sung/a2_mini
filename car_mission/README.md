# car_mission

CCTV가 YOLO로 **차량을 감지**하면 TurtleBot4를 **고정좌표로 출동**시키고, 도착 후 **제자리 회전 탐색**으로 차량을 다시 찾아 **특정 거리까지 접근 후 정지**하는 미니프로젝트 미션 패키지.

> ⚠️ 현재 **플레이스홀더(골격)** 상태입니다. 각 노드의 제어 로직은 `TODO` 로 표시돼 있습니다.

## 시스템 아키텍처

```
                              ┌──────────────── 단일 PC ────────────────┐
 [USB CCTV 카메라] ──cv2──> cctv_detector ──/cctv/car_detected(Bool)──┐
                              (YOLO best.pt)                          │
                                                                      v
 [TurtleBot4 OAK-D]                                          ┌── mission_manager ──┐
   ├ RGB  ─/robot2/oakd/rgb/.../image_raw─> yolo_detector ─┐ │   (상태머신)         │
   │                          (turtlebot4_yolo) /yolo/detections   IDLE→NAVIGATING  │
   │                                                       └→ car_tracker ─┐ →SEARCHING│
   └ depth ─/robot2/oakd/stereo/image_raw ──────────────────┘  /car/visible  →APPROACHING│
                                                                /car/bearing  →DONE      │
                                                                /car/distance └──────────┘
                                                                      │
                                              /robot2/cmd_vel  <──────┘ (회전/접근)
                                              Nav2 navigate_to_pose <──┘ (고정좌표 이동)
```

## 상태머신 (mission_manager)

| 상태 | 동작 | 전이 조건 |
|------|------|-----------|
| `IDLE` | 대기 | `/cctv/car_detected=True` → `NAVIGATING` |
| `NAVIGATING` | Nav2로 고정좌표 이동 | 도착(액션 success) → `SEARCHING` |
| `SEARCHING` | 제자리 회전하며 로봇캠 탐색 | `/car/visible=True` → `APPROACHING` |
| `APPROACHING` | 차량 중앙 유지하며 전진 | `/car/distance ≤ stop_distance_m` → `DONE` |
| `DONE` | 정지 | - |

## 노드

| 노드 | 역할 | 비고 |
|------|------|------|
| `cctv_detector` | USB CCTV 카메라 + YOLO → 차량 감지 트리거 | `turtlebot4_yolo/models/best.pt` 재사용 |
| `car_tracker` | 로봇캠 YOLO 검출 + depth 융합 → 방위/거리 | `/yolo/detections` 구독 |
| `mission_manager` | 상태머신, cmd_vel·Nav2 제어 | 미션 핵심 |
| (재사용) `turtlebot4_yolo/yolo_detector` | 로봇 RGB → YOLO bbox/검출 | 별도 패키지 |

## 전제 / 의존
- `pip install ultralytics`, `turtlebot4_yolo` 빌드(best.pt 포함).
- `NAVIGATING` 단계는 **Nav2 + localization(map)** 이 실행 중이어야 동작 (`nav 2`, `loc 2 <map.yaml>`).
- 고정좌표(`goal_x/y/yaw`)는 맵 작성 후 `config/mission.yaml` 에 실측값 입력.

## 실행
```bash
cd ~/a2_mini_ws && colcon build --packages-select car_mission && source install/setup.bash
ros2 launch car_mission mission.launch.py
```

## 남은 구현 (TODO)
- `mission_manager`: Nav2 NavigateToPose 전송/결과 처리, 접근 시 조향 게인 튜닝, 탐색 타임아웃·차량 소실 예외.
- `car_tracker`: depth 영상에서 bbox 중심 거리 샘플링(`rokey_pjt.depth_checker` 재사용), RGB/depth 정렬, 이미지 width 실측.
- `cctv_detector`: 카메라 index·해상도 실측, 디바운스 검증.
- `config/mission.yaml`: 고정좌표·거리·속도·게인 실측값.
