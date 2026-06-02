import sys

from ultralytics import YOLO

BASE = "/home/kimi/a2_mini_ws/src/yolo/yolo_by_view"

DATA = {
    "top_view": f"{BASE}/top_view/data.yaml",
    "turtlebot_view": f"{BASE}/turtlebot_view/data.yaml",
    "turtlebot_view_amr": f"{BASE}/turtlebot_view_amr/data.yaml",
}


def train(view):
    model = YOLO("yolov8s.pt")
    model.train(
        data=DATA[view],
        epochs=100,
        patience=20,
        batch=16,
        imgsz=640,
        name=f"yolo8s_{view}",
    )


if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in DATA:
        print(f"usage: python {sys.argv[0]} [{' | '.join(DATA)}]")
        sys.exit(1)
    train(sys.argv[1])
