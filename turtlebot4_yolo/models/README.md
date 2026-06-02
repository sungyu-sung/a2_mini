# models

여기에 학습시킨 **`best.pt`** 파일을 넣으세요.

```
turtlebot4_yolo/models/best.pt
```

- `setup.py`가 `models/*.pt`를 패키지 share로 설치하므로, 파일을 넣은 뒤 **다시 빌드**하면
  `model_path` 파라미터를 지정하지 않아도 자동으로 이 파일을 사용합니다.
- 다른 경로의 모델을 쓰려면 실행 시 `model_path` 파라미터로 절대 경로를 넘기면 됩니다.

> `.pt` 파일은 용량이 커서 git에 커밋하지 않는 것을 권장합니다.
