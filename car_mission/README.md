# car_mission

미니프로젝트 미션 패키지. **CCTV가 YOLO로 차량을 감지하면 TurtleBot4를 출동(undock)시켜 감시포인트로 이동**시키고, 로봇 카메라로 차량을 검출·거리 측정한다.

> 이전 플레이스홀더 노드(`mission_manager`/`cctv_detector`/`car_tracker`)는 정리됨. 현재는 실사용 노드 2개.

## 노드

| 노드 | 역할 | 입력 | 출력/동작 |
|------|------|------|-----------|
| `usb_car_undock` | PC에 USB로 연결된 **CCTV** + YOLO(topview). `car` 감지 시 **undock → Nav2로 감시포인트 이동** | `/dev/video2`, `/robot2/dock_status` | `Undock` 액션 → `/robot2/navigate_to_pose` |
| `yolo_depth_detector` | 로봇 **OAK-D RGB+depth** → car/dummy 검출 + `car` bbox 중심 거리 측정 | `/robot2/oakd/rgb/image_raw(/compressed)` + `/robot2/oakd/stereo/image_raw` | `/yolo_depth/image_annotated` (박스+거리) |

## 모델 (`models/`, git 포함)
| 파일 | 용도 |
|------|------|
| `topview_best.pt` | CCTV(탑뷰) 검출 — `usb_car_undock` |
| `robotview_best.pt` | 로봇캠 검출 — `yolo_depth_detector` |

두 노드 모두 `model_path` 미지정 시 `share/car_mission/models/` 에서 자동 로드(절대경로 하드코딩 없음).

## 전제 / 의존
- `pip install ultralytics`
- `usb_car_undock` 의 감시포인트 이동은 **Nav2 + localization(map)** 이 실행 중이어야 동작
  (`turtlebot4_navigation nav2.launch.py` / `localization.launch.py`, 맵: `src/map/robot2_map.yaml`).

## 빌드 & 실행
```bash
cd ~/a2_mini_ws && colcon build --packages-select car_mission && source install/setup.bash

# 개별 실행
ros2 run car_mission usb_car_undock
ros2 run car_mission yolo_depth_detector

# 또는 둘 다
ros2 launch car_mission mission.launch.py
```

## 주요 파라미터/상수
- `usb_car_undock` (현재 파일 상단 상수): `CAMERA_DEVICE=/dev/video2`, `CONFIDENCE=0.7`, `REQUIRED_HITS=5`, 감시포인트 `GOAL_X/Y/YAW`.
- `yolo_depth_detector` (ROS 파라미터): `rgb_topic`, `use_compressed`(기본 True), `depth_topic`, `conf`(0.5), `target_class`(car), `show`(True).

## TODO
- `usb_car_undock` 의 하드코딩 상수(카메라 index·감시포인트 좌표)를 ROS 파라미터로 분리.
- 로봇캠 거리 정밀화(stereo+비율매핑 → preview/depth 정렬 역투영). 자세한 배경은 [docs/yolo_depth_and_approach_handoff.md](../docs/yolo_depth_and_approach_handoff.md).
