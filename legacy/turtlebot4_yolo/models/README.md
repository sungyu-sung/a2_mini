# models

로봇 카메라(OAK-D)용 학습 가중치 **`robotview_best.pt`** 를 여기에 넣으세요.

```
turtlebot4_yolo/models/robotview_best.pt
```

- `yolo_detector` / `yolo_depth_detector` 는 `model_path` 파라미터가 비어 있으면
  이 `models/robotview_best.pt` 를 자동으로 사용합니다.
- `setup.py` 가 `models/*.pt` 를 패키지 share로 설치하므로, 파일을 넣은 뒤 **다시 빌드**하세요.
- 다른 경로의 모델을 쓰려면 실행 시 `model_path` 파라미터로 절대 경로를 넘기면 됩니다.

> 참고: 탑뷰(CCTV)용 모델은 `topview_best.pt` 로 별도 관리한다(현재 `src/` 루트, 작업트리 재구성 시 배치 예정).
> `.pt` 파일은 용량이 커서 git에 커밋하지 않는 것을 권장합니다(`.gitignore`의 `models/*.pt`).
