"""Vision detection node.

Subscribes to the Gazebo camera topic, runs YOLOv8n on each frame, prints
detections, saves annotated frames when a target class appears, and writes
the latest target bbox to /tmp/target_state.json for the follow controller.

Usage:
    python -m vision.detector --target person
    python -m vision.detector --target car --min-conf 0.4
"""
import argparse
import pathlib
import sys
import time
from collections import deque

import cv2
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from ultralytics import YOLO

# Import from the sibling pipeline package for shared state
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from pipeline.target_state import TargetState, write_state, clear_state

CAMERA_TOPIC = "/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image"
DETECTIONS_DIR = pathlib.Path("detections")
DETECTIONS_DIR.mkdir(exist_ok=True)


def gz_image_to_bgr(msg: Image) -> np.ndarray:
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    return cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)


class Detector:
    def __init__(self, target: str, min_conf: float, save_every_n_hits: int = 15):
        self.target = target.lower()
        self.min_conf = min_conf
        self.save_every_n_hits = save_every_n_hits
        self.model = YOLO("yolov8n.pt")
        self.names = self.model.names
        self.frame_count = 0
        self.saved_count = 0
        self.hit_streak = 0
        self.last_status_time = 0.0
        self.frame_stamps = deque(maxlen=10)
        clear_state()  # start clean

    def on_image(self, msg: Image):
        self.frame_count += 1
        try:
            frame = gz_image_to_bgr(msg)
        except Exception as e:
            print(f"[detector] frame decode failed: {e}", file=sys.stderr)
            return

        h, w = frame.shape[:2]
        results = self.model(frame, verbose=False, conf=self.min_conf)[0]

        # Find the largest bbox of our target class (biggest = closest = most relevant)
        best = None
        for box in results.boxes:
            cls_id = int(box.cls[0])
            cls_name = self.names[cls_id].lower()
            if cls_name != self.target:
                continue
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            area = (x2 - x1) * (y2 - y1)
            if best is None or area > best[3]:
                best = (cls_name, conf, (x1, y1, x2, y2), area)

        if best:
            cls_name, conf, bbox, _ = best
            self.hit_streak += 1
            # Publish to shared state for the follow controller
            write_state(TargetState(
                timestamp=time.time(), cls=cls_name, conf=conf,
                bbox=bbox, frame_w=w, frame_h=h,
            ))
            # Save an annotated JPG occasionally so we don't create thousands
            if self.hit_streak == 1 or self.hit_streak % self.save_every_n_hits == 0:
                self._save_annotated(frame, results, cls_name, conf)
        else:
            self.hit_streak = 0

        self.frame_stamps.append(time.time())
        now = time.time()
        if now - self.last_status_time > 1.0:
            fps = ((len(self.frame_stamps) - 1)
                   / (self.frame_stamps[-1] - self.frame_stamps[0])
                   if len(self.frame_stamps) >= 2 else 0.0)
            hit = 1 if best else 0
            print(f"[detector] frame {self.frame_count} | {len(results.boxes)} det | "
                  f"{hit} '{self.target}' | {fps:.1f} FPS")
            self.last_status_time = now

    def _save_annotated(self, frame, results, cls_name, conf):
        self.saved_count += 1
        annotated = results.plot()
        cv2.rectangle(annotated, (0, 0), (annotated.shape[1], 40), (0, 0, 200), -1)
        cv2.putText(annotated, f"TARGET DETECTED: {cls_name} ({conf:.2f})",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        fname = DETECTIONS_DIR / f"target_{self.saved_count:04d}_{int(time.time())}.jpg"
        cv2.imwrite(str(fname), annotated)
        print(f"[detector] >>> TARGET '{cls_name}' conf={conf:.2f} — saved {fname}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="person")
    ap.add_argument("--min-conf", type=float, default=0.35)
    ap.add_argument("--topic", default=CAMERA_TOPIC)
    args = ap.parse_args()

    detector = Detector(args.target, args.min_conf)
    node = Node()
    if not node.subscribe(Image, args.topic, detector.on_image):
        print(f"[detector] failed to subscribe to {args.topic}", file=sys.stderr)
        sys.exit(1)

    print(f"[detector] subscribed to {args.topic}")
    print(f"[detector] target class: '{args.target}' (min_conf={args.min_conf})")
    print(f"[detector] state file: /tmp/target_state.json (for follow controller)")
    print(f"[detector] annotated hits: ./{DETECTIONS_DIR}/  Ctrl+C to stop.\n")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print(f"\n[detector] stopped. processed {detector.frame_count} frames, "
              f"saved {detector.saved_count} target hits.")


if __name__ == "__main__":
    main()
