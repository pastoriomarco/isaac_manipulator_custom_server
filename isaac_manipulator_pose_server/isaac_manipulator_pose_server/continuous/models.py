from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import Pose
from vision_msgs.msg import BoundingBox2D, Detection2D


@dataclass(frozen=True)
class Candidate:
    class_id: str
    class_score: float
    detection: Detection2D
    received_monotonic_sec: float

    @property
    def bbox(self) -> BoundingBox2D:
        return self.detection.bbox


@dataclass
class WorkerResult:
    worker_id: str
    success: bool
    object_id: int
    frame_name: str
    candidate: Candidate
    pose: Optional[Pose] = None
    error: str = ''
    timed_out: bool = False
    was_tracking_refresh: bool = False
