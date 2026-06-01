# tracking

TurtleBot4가 카메라로 **파란 큐브를 추적**하는 패키지.
큐브를 화면 중심에 두도록 회전하고, 설정 거리(기본 0.8m)를 유지하며 따라간다.

---

## 동작 원리

```
OAK-D RGB 영상 → 파란색 검출(HSV) → 큐브 중심 픽셀
        ↓                              ↓
   화면 중심과 비교 → 회전        depth로 거리 측정 → 전진/후진
        ↓
   /cmd_vel 발행 (로봇 제어)
```

- **회전**: 큐브 중심 픽셀이 화면 중앙에서 벗어난 만큼 비례 회전
- **거리 유지**: depth 카메라로 큐브까지 거리 측정 → 멀면 전진, 가까우면 후진
- **큐브 안 보이면**: 정지

---

## 실행

```bash
# 터미널 1 — 시뮬(평지+파란큐브) + 큐브 브릿지 + 추적 노드
sim-env
ros2 launch tracking tracking.launch.py

# 터미널 2 — 언독
sim-robot-undock

# 터미널 3 — 카메라 영상 보기
sim-camera
#   드롭다운에서 /oakd/rgb/preview/image_raw 선택

# 터미널 4 — 큐브를 키보드로 움직이기
sim-env
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/model/blue_cube/cmd_vel
```

터미널 4에서 큐브를 움직이면, 로봇이 카메라로 따라 회전하고 거리를 유지하며 추적한다.

> `sim-env`, `sim-robot-undock`, `sim-camera`는 `~/.bashrc`에 정의된 시뮬레이션 alias.

---

## 구성

| 파일 | 역할 |
|------|------|
| `tracking/tracking_node.py` | 파란색 검출 + depth 거리 → cmd_vel 추적 노드 |
| `launch/tracking.launch.py` | 시뮬 + 큐브 cmd_vel 브릿지 + 추적 노드 |
| `gazebo_simulation/worlds/tracking_world.sdf` | 평지 + 움직이는 파란 큐브(VelocityControl) |

### 큐브 (tracking_world.sdf)
- 0.3m 파란 큐브, 로봇 정면 `(-1.5, 0)`에 배치
- `VelocityControl` 플러그인 → `/model/blue_cube/cmd_vel`로 제어
- 로봇은 `(0, 0, 0)`에서 시작

---

## 파라미터 (tracking_node)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `namespace` | `''` | 실로봇은 `robot2` |
| `target_distance` | 0.8 | 큐브와 유지할 거리(m) |
| `distance_tol` | 0.15 | 거리 허용 오차(m) |
| `max_linear` | 0.25 | 최대 전진 속도 |
| `max_angular` | 0.8 | 최대 회전 속도 |
| `center_tol` | 30 | 중심 허용 픽셀 오차 |

```bash
# 파라미터 바꿔 실행 예시
ros2 run tracking tracking --ros-args -p target_distance:=1.0 -p max_angular:=1.0
```

### 파란색 HSV 범위 (tracking_node.py)
검출이 안 되거나 다른 색을 잡으면 `rgb_cb`의 범위를 조정:
```python
lower = np.array([100, 120, 50])
upper = np.array([130, 255, 255])
```

---

## 실로봇 적용

```bash
ros2 run tracking tracking --ros-args -p namespace:=robot2
```
토픽이 `/robot2/oakd/...`, `/robot2/cmd_vel`로 자동 매핑된다.
실로봇 OAK-D의 파란색 톤에 맞춰 HSV 범위 조정이 필요할 수 있다.
