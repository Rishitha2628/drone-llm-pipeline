"""Target state — the shared contract between vision.detector and pipeline.follow_controller.

The detector writes to /tmp/target_state.json every N frames; the follow
controller reads it every control tick. This is deliberately simple: a JSON
file. No sockets, no shared memory, no ROS. If the file is stale (older than
STALE_S seconds), the target is considered lost.

Format:
{
  "timestamp": 1234567890.123,       # unix seconds, float
  "class": "person",                  # YOLO class name
  "conf": 0.61,                       # detection confidence
  "bbox": [x1, y1, x2, y2],           # pixel coords in the camera frame
  "frame_w": 640, "frame_h": 480,     # frame dimensions
}

If the detector has never seen the target, the file simply does not exist.
"""
import json
import os
import time
from dataclasses import dataclass, asdict
from typing import Optional, Tuple

STATE_PATH = "/tmp/target_state.json"
STALE_S = 1.0   # target considered lost if state file is older than this


@dataclass
class TargetState:
    timestamp: float
    cls: str
    conf: float
    bbox: Tuple[float, float, float, float]   # x1, y1, x2, y2 in pixels
    frame_w: int
    frame_h: int

    def as_dict(self):
        d = asdict(self)
        # rename 'cls' -> 'class' in the on-disk JSON since 'class' is Python-reserved
        d["class"] = d.pop("cls")
        return d

    @property
    def cx(self) -> float:
        return 0.5 * (self.bbox[0] + self.bbox[2])

    @property
    def cy(self) -> float:
        return 0.5 * (self.bbox[1] + self.bbox[3])

    @property
    def bbox_area(self) -> float:
        return max(0.0, (self.bbox[2] - self.bbox[0])) * max(0.0, (self.bbox[3] - self.bbox[1]))

    @property
    def area_fraction(self) -> float:
        """bbox area as a fraction of the whole frame (0..1). Distance proxy."""
        total = float(self.frame_w * self.frame_h)
        return self.bbox_area / total if total > 0 else 0.0

    def horizontal_error_norm(self) -> float:
        """
        Horizontal centroid offset from the frame centre, normalised to [-1, +1].
        Positive = target is to the right of centre.
        """
        return (self.cx - 0.5 * self.frame_w) / (0.5 * self.frame_w)


def write_state(state: TargetState) -> None:
    """Atomic-ish write so the reader never sees a half-written file."""
    tmp = STATE_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump(state.as_dict(), f)
    os.replace(tmp, STATE_PATH)


def read_state() -> Optional[TargetState]:
    """Read the latest target state. Returns None if missing, malformed, or stale."""
    try:
        with open(STATE_PATH) as f:
            d = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    if time.time() - d.get("timestamp", 0) > STALE_S:
        return None
    return TargetState(
        timestamp=d["timestamp"],
        cls=d["class"],
        conf=d["conf"],
        bbox=tuple(d["bbox"]),
        frame_w=d["frame_w"],
        frame_h=d["frame_h"],
    )


def clear_state() -> None:
    """Wipe the state file. Call at follow-controller startup."""
    try:
        os.remove(STATE_PATH)
    except FileNotFoundError:
        pass
