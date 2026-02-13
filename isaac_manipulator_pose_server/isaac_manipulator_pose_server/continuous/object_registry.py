import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from rclpy.node import Node
from vision_msgs.msg import BoundingBox2D, Detection2D

from isaac_manipulator_pose_server.config import WorkflowConfig
from isaac_manipulator_pose_server.continuous.models import Candidate
from isaac_manipulator_server_interfaces.msg import TrackedObject, TrackedObjectArray


@dataclass
class _TrackedRecord:
    object_id: int
    class_id: str
    class_score: float
    frame_name: str
    pose: Pose
    bbox: BoundingBox2D
    status: int
    worker_id: str
    first_seen_stamp: Time
    last_update_stamp: Time
    first_seen_monotonic_sec: float
    last_update_monotonic_sec: float


class ObjectRegistry:
    """Stores tracked object lifecycle state."""

    def __init__(self, node: Node, config: WorkflowConfig):
        self._node = node
        self._config = config
        self._lock = threading.Lock()
        self._next_object_id = 0
        self._records: Dict[int, _TrackedRecord] = {}

    def clear(self):
        with self._lock:
            self._records.clear()

    def allocate_object_id(self) -> int:
        with self._lock:
            object_id = self._next_object_id
            self._next_object_id += 1
            return object_id

    def count_active_tracking(self) -> int:
        with self._lock:
            return sum(
                1
                for record in self._records.values()
                if record.status == TrackedObject.TRACKING
            )

    def add_tracking(
        self,
        *,
        object_id: int,
        frame_name: str,
        worker_id: str,
        candidate: Candidate,
        pose: Pose,
    ):
        now_stamp = self._node.get_clock().now().to_msg()
        now_sec = time.monotonic()
        with self._lock:
            self._records[object_id] = _TrackedRecord(
                object_id=object_id,
                class_id=candidate.class_id,
                class_score=candidate.class_score,
                frame_name=frame_name,
                pose=pose,
                bbox=candidate.bbox,
                status=TrackedObject.TRACKING,
                worker_id=worker_id,
                first_seen_stamp=now_stamp,
                last_update_stamp=now_stamp,
                first_seen_monotonic_sec=now_sec,
                last_update_monotonic_sec=now_sec,
            )

    def update_tracking(
        self,
        *,
        object_id: int,
        candidate: Candidate,
        pose: Pose,
    ) -> bool:
        now_stamp = self._node.get_clock().now().to_msg()
        now_sec = time.monotonic()
        with self._lock:
            record = self._records.get(object_id)
            if record is None:
                return False
            record.pose = pose
            record.bbox = candidate.bbox
            record.class_score = candidate.class_score
            record.status = TrackedObject.TRACKING
            record.last_update_stamp = now_stamp
            record.last_update_monotonic_sec = now_sec
            return True

    def mark_lost(self, object_id: int) -> bool:
        now_stamp = self._node.get_clock().now().to_msg()
        now_sec = time.monotonic()
        with self._lock:
            record = self._records.get(object_id)
            if record is None:
                return False
            record.status = TrackedObject.LOST
            record.worker_id = ''
            record.last_update_stamp = now_stamp
            record.last_update_monotonic_sec = now_sec
            return True

    def mark_used(self, object_id: int) -> Tuple[bool, Optional[str]]:
        now_stamp = self._node.get_clock().now().to_msg()
        now_sec = time.monotonic()
        with self._lock:
            record = self._records.get(object_id)
            if record is None:
                return False, None
            worker_id = record.worker_id
            record.status = TrackedObject.USED
            record.worker_id = ''
            record.last_update_stamp = now_stamp
            record.last_update_monotonic_sec = now_sec
            return True, worker_id

    def assigned_object_ids(self) -> List[int]:
        with self._lock:
            return sorted([
                object_id
                for object_id, record in self._records.items()
                if record.status == TrackedObject.TRACKING
            ])

    def get_record(self, object_id: int) -> Optional[_TrackedRecord]:
        with self._lock:
            record = self._records.get(object_id)
            if record is None:
                return None
            return _TrackedRecord(**record.__dict__)

    def active_worker_assignments(self) -> Dict[str, int]:
        with self._lock:
            assignments: Dict[str, int] = {}
            for object_id, record in self._records.items():
                if record.status != TrackedObject.TRACKING:
                    continue
                if not record.worker_id:
                    continue
                assignments[record.worker_id] = object_id
            return assignments

    def should_refresh_tracking(
        self,
        *,
        object_id: int,
        refresh_period_sec: float,
        now_sec: float,
    ) -> bool:
        with self._lock:
            record = self._records.get(object_id)
            if record is None:
                return False
            if record.status != TrackedObject.TRACKING:
                return False
            return (now_sec - record.last_update_monotonic_sec) >= refresh_period_sec

    def prune_lost(self, now_sec: float, object_lost_ttl_sec: float):
        with self._lock:
            remove_ids: List[int] = []
            for object_id, record in self._records.items():
                if record.status in (TrackedObject.USED, TrackedObject.LOST):
                    if (now_sec - record.last_update_monotonic_sec) > object_lost_ttl_sec:
                        remove_ids.append(object_id)
                    continue
                if record.status == TrackedObject.TRACKING:
                    if (now_sec - record.last_update_monotonic_sec) > object_lost_ttl_sec:
                        record.status = TrackedObject.LOST
                        record.worker_id = ''
                        record.last_update_stamp = self._node.get_clock().now().to_msg()
                        record.last_update_monotonic_sec = now_sec
            for object_id in remove_ids:
                del self._records[object_id]

    def is_candidate_duplicate(self, candidate: Candidate) -> bool:
        with self._lock:
            for record in self._records.values():
                if record.status != TrackedObject.TRACKING:
                    continue
                if record.class_id != candidate.class_id:
                    continue
                if self._bbox_iou(candidate.detection, record.bbox) >= self._config.nms_iou_threshold:
                    return True
                if self._center_distance(candidate.detection, record.bbox) <= (
                    self._config.bbox_memory_center_distance_px
                ):
                    return True
        return False

    def snapshot_tracked_objects(self, target_count: int) -> TrackedObjectArray:
        message = TrackedObjectArray()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = self._config.output_frame_id
        message.target_count = target_count

        with self._lock:
            sorted_records = sorted(self._records.values(), key=lambda record: record.object_id)
            for record in sorted_records:
                message.objects.append(self._to_message(record))
        return message

    def summary(self) -> Dict:
        summary = {
            'output_frame_id': self._config.output_frame_id,
            'count': 0,
            'objects': [],
        }
        with self._lock:
            records = sorted(self._records.values(), key=lambda record: record.object_id)
            for record in records:
                if record.status != TrackedObject.TRACKING:
                    continue
                summary['objects'].append({
                    'object_id': record.object_id,
                    'class_id': record.class_id,
                    'class_score': record.class_score,
                    'frame_name': record.frame_name,
                    'pose': {
                        'position': [
                            float(record.pose.position.x),
                            float(record.pose.position.y),
                            float(record.pose.position.z),
                        ],
                        'orientation': [
                            float(record.pose.orientation.x),
                            float(record.pose.orientation.y),
                            float(record.pose.orientation.z),
                            float(record.pose.orientation.w),
                        ],
                    },
                })
            summary['count'] = len(summary['objects'])
        return summary

    def _to_message(self, record: _TrackedRecord) -> TrackedObject:
        message = TrackedObject()
        message.object_id = record.object_id
        message.class_id = record.class_id
        message.class_score = float(record.class_score)
        message.frame_name = record.frame_name
        message.pose = record.pose
        message.bbox = record.bbox
        message.status = record.status
        message.worker_id = record.worker_id
        message.first_seen = record.first_seen_stamp
        message.last_update = record.last_update_stamp
        return message

    def _bbox_iou(self, detection: Detection2D, bbox: BoundingBox2D) -> float:
        first = self._bbox_xyxy_detection(detection)
        second = self._bbox_xyxy_bbox(bbox)
        if first is None or second is None:
            return 0.0

        first_x1, first_y1, first_x2, first_y2 = first
        second_x1, second_y1, second_x2, second_y2 = second
        inter_x1 = max(first_x1, second_x1)
        inter_y1 = max(first_y1, second_y1)
        inter_x2 = min(first_x2, second_x2)
        inter_y2 = min(first_y2, second_y2)
        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h
        if inter_area <= 0.0:
            return 0.0

        first_area = max(0.0, first_x2 - first_x1) * max(0.0, first_y2 - first_y1)
        second_area = max(0.0, second_x2 - second_x1) * max(0.0, second_y2 - second_y1)
        union_area = first_area + second_area - inter_area
        if union_area <= 0.0:
            return 0.0
        return inter_area / union_area

    def _center_distance(self, detection: Detection2D, bbox: BoundingBox2D) -> float:
        first_x = float(detection.bbox.center.position.x)
        first_y = float(detection.bbox.center.position.y)
        second_x = float(bbox.center.position.x)
        second_y = float(bbox.center.position.y)
        delta_x = first_x - second_x
        delta_y = first_y - second_y
        return (delta_x * delta_x + delta_y * delta_y) ** 0.5

    def _bbox_xyxy_detection(
        self,
        detection: Detection2D,
    ) -> Optional[Tuple[float, float, float, float]]:
        return self._bbox_xyxy(
            center_x=float(detection.bbox.center.position.x),
            center_y=float(detection.bbox.center.position.y),
            width=float(detection.bbox.size_x),
            height=float(detection.bbox.size_y),
        )

    def _bbox_xyxy_bbox(
        self,
        bbox: BoundingBox2D,
    ) -> Optional[Tuple[float, float, float, float]]:
        return self._bbox_xyxy(
            center_x=float(bbox.center.position.x),
            center_y=float(bbox.center.position.y),
            width=float(bbox.size_x),
            height=float(bbox.size_y),
        )

    def _bbox_xyxy(
        self,
        *,
        center_x: float,
        center_y: float,
        width: float,
        height: float,
    ) -> Optional[Tuple[float, float, float, float]]:
        if width <= 0.0 or height <= 0.0:
            return None
        half_width = width / 2.0
        half_height = height / 2.0
        return (
            center_x - half_width,
            center_y - half_height,
            center_x + half_width,
            center_y + half_height,
        )
