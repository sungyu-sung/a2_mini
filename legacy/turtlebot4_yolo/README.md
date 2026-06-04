# turtlebot4_yolo

TurtleBot4 OAK-D 카메라 영상에 직접 학습시킨 **YOLO(`best.pt`)** 를 적용해,
**bounding box가 그려진 이미지 토픽을 발행/구독**하는 미니프로젝트 실습 패키지.

`turtlebot4_image`의 `image_publisher` / `image_subscriber` 구조를 참고했습니다.

## 노드

| 노드 | 역할 | 구독 | 발행 |
|------|------|------|------|
| `yolo_detector` | 카메라 영상 → YOLO 추론 → bbox 이미지 + 검출결과 | `/robot2/oakd/rgb/preview/image_raw` | `/yolo/image_annotated` (Image), `/yolo/detections` (String/JSON) |
| `yolo_viewer` | bbox 이미지를 화면에 표시 | `/yolo/image_annotated` | - |
| `yolo_depth_detector` | **단일 노드**: RGB로 car/dummy 탐지 + `car` bbox 중심 depth(거리) 측정. RGB·depth를 시간 동기화해 프레임 어긋남 방지 | `/robot2/oakd/rgb/image_raw` + `/robot2/oakd/stereo/image_raw` | `/yolo_depth/image_annotated` (Image, 거리 오버레이) |

### yolo_depth_detector

RGB YOLO 탐지와 depth 거리 측정을 **한 노드**에서 처리합니다(`message_filters`로 RGB+depth 동기화). `car` 탐지 시 bbox 중심의 거리를 화면/로그에 표시.

```bash
ros2 run turtlebot4_yolo yolo_depth_detector
# 파라미터: rgb_topic, depth_topic, conf, target_class(기본 car), slop(동기화 허용 시간차), show
```

> 해상도 매핑: RGB와 stereo depth 해상도가 달라 bbox 중심을 비율로 매핑합니다. FOV/정렬 차가 크면 오차가 있을 수 있어, 정밀 측정 시 depth-to-RGB 정렬이 필요합니다.

## 사전 준비

```bash
pip install ultralytics        # YOLO 추론 라이브러리
```

학습한 로봇캠 가중치를 다음 위치에 넣습니다:

```
src/turtlebot4_yolo/models/robotview_best.pt
```

## 빌드

```bash
cd ~/a2_mini_ws
colcon build --packages-select turtlebot4_yolo
source install/setup.bash
```

## 실행

### launch (detector + viewer 한 번에)

```bash
ros2 launch turtlebot4_yolo yolo.launch.py
```

옵션:

```bash
# 다른 경로의 모델 / confidence / 입력 토픽 지정
ros2 launch turtlebot4_yolo yolo.launch.py \
    model_path:=/abs/path/to/best.pt conf:=0.4 \
    input_topic:=/robot2/oakd/rgb/preview/image_raw

# viewer 없이 detector만 (headless)
ros2 launch turtlebot4_yolo yolo.launch.py viewer:=false
```

### 노드 개별 실행

```bash
# 터미널 1
ros2 run turtlebot4_yolo yolo_detector
# 모델 경로를 직접 지정하려면:
ros2 run turtlebot4_yolo yolo_detector --ros-args -p model_path:=/abs/path/best.pt

# 터미널 2
ros2 run turtlebot4_yolo yolo_viewer
```

## 파라미터 (yolo_detector)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `model_path` | `''` | 모델 경로. 비우면 `share/turtlebot4_yolo/models/robotview_best.pt` 사용 |
| `input_topic` | `/robot2/oakd/rgb/preview/image_raw` | 입력 카메라 토픽 |
| `annotated_topic` | `/yolo/image_annotated` | bbox 이미지 발행 토픽 |
| `detections_topic` | `/yolo/detections` | 검출결과(JSON) 발행 토픽 |
| `conf` | `0.5` | confidence threshold |
| `device` | `''` | `''`=자동, `'cpu'`, `'0'`(GPU) 등 |
| `publish_detections` | `true` | 검출결과 JSON 발행 여부 |

## 검출결과 JSON 형식 (`/yolo/detections`)

```json
{
  "stamp": {"sec": 0, "nanosec": 0},
  "detections": [
    {"class_id": 0, "class_name": "cup", "conf": 0.91, "bbox_xyxy": [10.0, 20.0, 110.0, 220.0]}
  ]
}
```
