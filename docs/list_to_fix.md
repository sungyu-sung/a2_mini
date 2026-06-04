# List to Fix (car_mission 진행 체크리스트)

> 현재 car_mission 노드: `usb_car_undock`, `dori_search_car`, `yolo_depth_detector`, `car_tracking`, `mission_orchestrator`(골격).
> 모델: `car_mission/models/` 의 `topview_best.pt`(CCTV) / `robotview_best.pt`(로봇캠).

## 1. 통합 PC에서 기능단위 테스트
- [ ] `usb_car_undock` — CCTV(`/dev/video4`) car 감지 → undock → Nav2 감시포인트 이동
- [ ] `dori_search_car` — 감시포인트에서 회전 탐색 → car 중앙 정렬(못 찾으면 360°후 beep)
- [ ] `car_tracking` — 움직이는 car 추적(1m 유지)
- [ ] (참고) `yolo_depth_detector` 는 기능 테스트 완료 ✅
- 주의: 각 노드 모두 `/robot2/cmd_vel` 직접 발행(또는 Nav2) → **단독으로 하나씩** 테스트(동시 실행 금지).

## 2. 전체 플로우 구현 (`mission_orchestrator.py`)
- [ ] 순차 단일활성 오케스트레이션: `usb_car_undock → dori_search_car → yolo_depth_detector → car_tracking`
- [ ] 각 노드에 **enable 게이트**(자기 차례 전 cmd_vel 안 냄) + **done 신호** 추가
      (현재 `car_tracking`만 enable 있음, done은 전부 없음)
- [ ] orchestrator 가 done 받고 다음 단계 enable (한 순간 cmd_vel 1개 → 충돌 회피)
- contract(안): `/mission/undock_done`, `/mission/search_enable`·`/mission/search_done`,
  `/mission/approach_enable`·`/mission/approach_done`, `/robot2/tracking_enable`

## 3. msi · vicuts(노트북)에서 전체 플로우 테스트
- [ ] msi 노트북에서 전체 플로우 실행
- [ ] vicuts 노트북에서 전체 플로우 실행
- 주의: PC마다 다른 값은 **파라미터로** 맞추기 — 특히 `camera_device`(usb_car_undock, v4l 인덱스 PC마다 다름), 감시포인트 좌표 등.
- 전제: Nav2 + localization(`src/map/robot2_map.yaml`), **PC-로봇 시간 동기화**(이전에 tf `odom 없음`/lidar 타임스탬프 오류 = clock skew 였음).

## 4. 기능 개선 — yolo_depth_detector 를 Nav2 기반으로
- [ ] 현재 target본은 **Nav2 없이 reactive 직진 접근**(장애물 회피 없음)
- [ ] → car 위치를 map 프레임 goal 로 변환해 **Nav2 NavigateToPose 로 1m 접근**(장애물 회피)
- 참고 설계: `docs/yolo_depth_and_approach_handoff.md` 4장(approach_controller 청사진)
