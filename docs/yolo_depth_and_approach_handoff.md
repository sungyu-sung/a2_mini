# YOLO+Depth 인지 & Approach 개발 핸드오프

> **목적**: car 인지(`yolo_depth_detector`)의 동작 사양과, 보류된 **approach(장애물 회피 접근)** 기능의 설계·관례·함정을 정리한다. 담당 팀원이 우리와 동일한 방식으로 이어서 개발할 수 있도록 한다.
>
> **이 문서를 Claude와 쓰는 법**: 작업 시작 시 "이 문서(`src/docs/yolo_depth_and_approach_handoff.md`)를 읽고 시작해줘"라고 하면 설계 맥락·관례·함정을 그대로 이어받는다. 단, **Claude는 실제 코드도 같이 읽어야** 정확하다. 이 문서는 *코드만으론 안 보이는 결정 이유·시도하다 막힌 것·환경 관례*를 담는다. 코드와 문서가 다르면 코드가 진실이다(이 문서를 갱신할 것).

---

## 0. 현재 레포 상태 (main 기준)

| 자산 | 위치 | 상태 |
|------|------|------|
| car 인지(단일 노드) | `turtlebot4_yolo/turtlebot4_yolo/yolo_depth_detector.py` | **동작**(실시간) |
| YOLO 가중치(로봇캠) | `turtlebot4_yolo/models/robotview_best.pt` (gitignore, 8s_augmentation, 클래스 `car`/`dummy`) | 로컬에만 |
| YOLO 가중치(탑뷰/CCTV) | `topview_best.pt` (현재 `src/` 루트, 배치 재구성 예정) | 로컬에만 |
| 미션 골격 | `car_mission/` (`mission_manager`/`cctv_detector`/`car_tracker` 플레이스홀더) | 골격만 |
| 맵 | `map/robot2_map.yaml` + `.pgm` | Nav2 localization용 |
| 참고 구현 | `chanhwi` 브랜치 `tracking` 패키지(반응형 추적), `rokey_pjt`(depth 거리) | 참고용 |

> **approach 기능은 한 번 구현했다가 되돌렸다(main에 없음).** 그 설계는 아래 4장에 보존. 새로 구현 시 그대로 참고.

---

## 1. yolo_depth_detector — RGB·Depth 처리 사양 (핵심)

car/dummy를 RGB로 탐지하고, `car`의 bbox 중심 거리를 depth로 측정하는 **단일 노드**.

### 구독 (둘 다 `BEST_EFFORT` / `KEEP_LAST` / `depth=1`)
| | 토픽(기본값) | 타입 | 디코딩 |
|---|---|---|---|
| RGB | `/robot2/oakd/rgb/image_raw/compressed` | `CompressedImage` | `compressed_imgmsg_to_cv2(.,'bgr8')` |
| Depth | `/robot2/oakd/stereo/image_raw` | `Image` | `imgmsg_to_cv2(.,'passthrough')` → uint16 **mm** |

- `use_compressed=False`면 RGB는 raw `Image`로.
- **QoS `depth=1`가 핵심**: 항상 최신 프레임만, 오래된 건 드롭 → 지연 누적 방지.

### 처리 방식 — **RGB 주도 + depth 최신 캐시 (시간 동기화 안 함)**
- `on_depth`: depth를 `self.latest_depth`에 **캐시만**.
- `on_rgb`/`on_rgb_compressed` → `_process`: **매 RGB 프레임마다** YOLO 추론 → 박스 → `car`면 **가장 최근 depth**에서 거리 샘플.
- RGB·depth를 `message_filters`로 엄격 동기화하지 **않음**(느린 주행에선 시간차 무의미). 정밀 동기화가 필요하면 `message_filters`로 교체.

### 거리 측정
- RGB 픽셀 → depth 픽셀 **해상도 비율 매핑**: `du=cx*dw/rw`, `dv=cy*dh/rh`.
- 중심 주변 **5×5 패치(win=2)에서 0 제외 중앙값(median)** → 노이즈/구멍 견고.
- `mm/1000 → m`. 유효 없으면 `NaN`.

### 출력/표시
- 발행: `/yolo_depth/image_annotated` (`Image`, 박스+거리 오버레이).
- 표시: **메인 루프 `spin_once` + `cv2.imshow`** (콜백 안 imshow는 창이 안 갱신됨 → 이 패턴 필수).

### 파라미터
`model_path`(빈값=share의 robotview_best.pt), `rgb_topic`, `use_compressed`(기본 True), `depth_topic`, `annotated_topic`, `conf`(0.5), `target_class`(car), `show`(True).

### 실행
```bash
cd ~/a2_mini_ws && source install/setup.bash
ros2 run turtlebot4_yolo yolo_depth_detector
# 가볍게: --ros-args -p rgb_topic:=/robot2/oakd/rgb/preview/image_raw
# headless: -p show:=false
```

---

## 2. 실시간성을 만든 3요소 (그대로 유지)

처음엔 ~1fps + 4초 지연이었다. 원인은 **YOLO가 아니라(GPU 5ms/frame) 네트워크**였다. 해결:
1. **compressed RGB** — full 704라도 JPEG ~50KB → wifi 대역 6배↓ (raw preview 300KB보다도 작음).
2. **QoS `KEEP_LAST depth=1`** — backlog 안 쌓임(오래된 프레임 드롭).
3. **UDP 수신버퍼 확대** — `/etc/sysctl.d/60-ros2-dds.conf` 에 `net.core.rmem_max=net.core.rmem_default=2147483647` (PC별 1회 설정, 재부팅 유지).

---

## 3. 정확도 한계 (approach 개발자 필독)

- **현재 거리는 근사값.** RGB(`oakd/rgb`, 광각)와 `oakd/stereo` depth는 **해상도·FOV가 달라** 단순 비율 매핑이라 화면 가장자리·먼 물체일수록 오차.
- **정밀 3D가 필요하면**: `oakd/rgb/preview/depth`(RGB에 정렬된 depth) + `oakd/rgb/preview/camera_info`(intrinsics)로 바꿔 **역투영(deproject)**:
  `X=(u-cx)·d/fx, Y=(v-cy)·d/fy, Z=d` (카메라 광학 프레임). `rokey_pjt/depth_checker.py`가 `camera_info.k`로 같은 방식. chanhwi `tracking`은 `preview/depth`를 씀.
- 현재 노드는 **거리(스칼라)만** 표시/발행. approach엔 **3D 좌표(PointStamped) + tf(map 변환)** 가 추가로 필요.

---

## 4. Approach 기능 설계 (보류됨 — 재구현 시 청사진)

**목표**: car까지 **1m**가 될 때까지 **장애물을 회피하며** 접근. **Nav2식 goal 생성**(NavigateToPose)으로 회피는 Nav2 costmap이 담당.

### 결정사항
- **Full Nav2(map+localization)** 사용 → goal은 `map` 프레임.
- 패키지 위치: **`car_mission`에 통합**(APPROACHING 상태).

### 파이프라인 (구현했다가 되돌린 구조)
```
yolo_depth_detector(인지 확장) ── car 검출 시 역투영 →
   /car/point(PointStamped, 카메라 광학 프레임), /car/distance(Float32), /car/visible(Bool)
        │
   car_mission/approach_controller(신규)
     ├ tf2: /car/point → map 프레임
     ├ 로봇(map→base_link)과 잇는 직선에서 car 1m 앞 + car 향한 yaw 를 goal 로
     ├ Nav2 NavigateToPose 전송 (장애물 회피 = Nav2 costmap)
     └ /car/distance ≤ stop_distance(+tol) 또는 Nav2 success → goal 취소·/approach_done
        │
   mission_manager: APPROACHING 진입 시 /approach_enable=True, 이때 cmd_vel 직접발행 중단(Nav2 담당),
     /approach_done 받으면 DONE
```

### 구현 시 주의 (실제로 짜며 정한 것들)
- **cmd_vel 소유권**: SEARCHING(회전)=mission_manager 직접 발행 / APPROACHING=Nav2가 발행. 겹치면 충돌 → 단계 전환 시 한쪽이 양보.
- **goal 재전송**: car 추정이 `goal_update_dist`(예 0.2m) 이상 변할 때만 재전송(Nav2 스팸 방지).
- **종료 이중화**: 라이브 depth 거리 ≤ 1m **또는** Nav2 goal 도달(status=4).
- **car 소실**: `lost_timeout` 지나면 goal 취소(정지).
- **goal이 막힘**: car가 costmap 장애물로 잡히면 inflation 때문에 "1m 앞"이 점유영역일 수 있음 → `inflation_radius`/goal tolerance 조정.
- **chanhwi `tracking`은 반응형(회피 없음)** — 인지→bearing/거리 로직만 참고, 직선 전진 부분은 Nav2 goal로 대체.

---

## 5. 환경 전제 & 검증 (Phase 0)

approach(또는 Nav2 쓰는 모든 작업) 전 확인:
```bash
# Nav2 + localization (alias 원본)
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot2
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot2 \
  map:=/home/sungyu/a2_mini_ws/src/map/robot2_map.yaml
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot2   # RViz → 2D Pose Estimate로 초기위치 지정(필수)

# 검증
ros2 lifecycle get /robot2/bt_navigator           # active [3]
ros2 action list | grep navigate_to_pose          # /robot2/navigate_to_pose (이게 Nav2. navigate_to_position은 Create3 내장이라 다름)
ros2 run tf2_ros tf2_echo odom base_link          # 프레임명 확인 (안 되면 robot2/odom robot2/base_link)
ros2 run tf2_ros tf2_echo map base_link           # localization+초기위치 후 나옴
```
- **`map→base_link` tf는 nav2가 아니라 localization(AMCL)+RViz 초기위치가 있어야** 생긴다.
- tf 프레임이 `base_link`인지 `robot2/base_link`인지 확인 후 `approach_controller`의 `base_frame` 파라미터로 맞출 것.

---

## 6. 워크스페이스 관례 / 함정 (꼭 지킬 것)

- **colcon build는 반드시 워크스페이스 루트(`~/a2_mini_ws`)에서.** 패키지 폴더 안에서 돌리면 중첩 `install`이 생기고 루트 `install`이 갱신 안 돼 *소스는 고쳤는데 옛 코드가 도는* 함정. 빌드 후 `install/<pkg>/.../<file>.py`로 실제 갱신 확인.
- **파일 삭제/엔트리포인트 변경 후엔** 해당 패키지 `build/`,`install/` 지우고 재빌드(잔재 executable 남음).
- **새 노드 추가 시**: `setup.py` console_scripts + `package.xml` 의존성 추가.
- **카메라/depth 토픽 구독은 `BEST_EFFORT`** 로(센서 QoS). RELIABLE로 받으면 안 들어오거나 지연 누적.
- **로봇 토픽 안 들어올 때 점검 순서**: ① 같은 wifi/`ROS_DOMAIN_ID=2`/discovery server(192.168.107.102) ② depth 등 대형 메시지는 QoS·UDP버퍼(2장) ③ 로봇 카메라 실제 발행 여부(로봇에서 `ros2 topic hz`) ④ 비대화형 셸은 `ROS_SUPER_CLIENT=False`라 이미지 스트림을 못 볼 수 있음(대화형 터미널에서 확인).
- 환경설정은 `~/.bashrc` 하단 "a2_mini_ws 팀 공통 환경" 블록(README 0-(4)). `a2_mini_env.bash` 파일 방식은 폐기됨.

---

## 7. 남은 작업(TODO)

- `yolo_depth_detector`: (approach용) preview/depth+camera_info 역투영으로 `/car/point` 발행 추가.
- `car_mission/approach_controller`: 4장 설계대로 신규 구현.
- `mission_manager`: `NAVIGATING`(고정좌표 Nav2 전송) 구현, 탐색 타임아웃, APPROACHING 연결.
- `cctv_detector`: USB 카메라 index·해상도 실측.
- 튜닝: goal 재전송 주기, stop 거리, costmap inflation, car 소실 복구.
