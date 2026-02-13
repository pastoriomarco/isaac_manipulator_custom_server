import threading
import time
from typing import List, Optional, Set, Tuple

from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from vision_msgs.msg import Detection2D, Detection2DArray

from isaac_manipulator_pose_server.config import WorkflowConfig
from isaac_manipulator_pose_server.continuous.models import Candidate


class DetectorCache:
    """Stores and filters the latest detector snapshot for orchestrator scheduling."""

    def __init__(self, node: Node, config: WorkflowConfig):
        self._node = node
        self._config = config
        self._lock = threading.Lock()
        self._latest_msg: Optional[Detection2DArray] = None
        self._latest_receive_monotonic_sec = 0.0
        self._subscription = self._node.create_subscription(
            Detection2DArray,
            self._config.detections_topic_name,
            self._on_detections,
            self._resolve_qos(self._config.detections_topic_qos),
        )

    def _resolve_qos(self, qos_name: str):
        if qos_name == 'SENSOR_DATA':
            return qos_profile_sensor_data
        return QoSProfile(depth=10)

    def _on_detections(self, msg: Detection2DArray):
        with self._lock:
            self._latest_msg = msg
            self._latest_receive_monotonic_sec = time.monotonic()

    def latest_age_sec(self) -> float:
        with self._lock:
            if self._latest_msg is None:
                return float('inf')
            return max(0.0, time.monotonic() - self._latest_receive_monotonic_sec)

    def get_candidates(
        self,
        *,
        target_class_ids: Optional[List[str]],
        max_candidates: int,
        min_receive_time_sec: float,
    ) -> List[Candidate]:
        with self._lock:
            latest_msg = self._latest_msg
            receive_time_sec = self._latest_receive_monotonic_sec

        if latest_msg is None:
            return []
        if receive_time_sec <= min_receive_time_sec:
            return []

        age_sec = max(0.0, time.monotonic() - receive_time_sec)
        if (
            self._config.detections_topic_stale_sec > 0.0 and
            age_sec > self._config.detections_topic_stale_sec
        ):
            return []

        filter_ids: Set[str] = {str(class_id) for class_id in (target_class_ids or [])}
        selected: List[Candidate] = []
        for detection in list(latest_msg.detections):
            class_id, class_score = self._extract_best_class(detection)
            if filter_ids and class_id not in filter_ids:
                continue
            selected.append(
                Candidate(
                    class_id=class_id,
                    class_score=class_score,
                    detection=detection,
                    received_monotonic_sec=receive_time_sec,
                )
            )

        selected.sort(key=lambda candidate: candidate.class_score, reverse=True)

        deduped: List[Candidate] = []
        for candidate in selected:
            if self._suppressed(candidate, deduped):
                continue
            deduped.append(candidate)
            if len(deduped) >= max_candidates:
                break
        return deduped

    def _suppressed(self, candidate: Candidate, accepted: List[Candidate]) -> bool:
        for existing in accepted:
            if existing.class_id != candidate.class_id:
                continue
            if self._bbox_iou(candidate.detection, existing.detection) >= self._config.nms_iou_threshold:
                return True
            if self._center_distance(candidate.detection, existing.detection) <= (
                self._config.bbox_memory_center_distance_px
            ):
                return True
        return False

    def _extract_best_class(self, detection: Detection2D) -> Tuple[str, float]:
        best_class_id = 'unknown'
        best_score = -1.0
        for result in detection.results:
            hypothesis = result.hypothesis
            if hypothesis.score > best_score:
                best_score = float(hypothesis.score)
                best_class_id = str(hypothesis.class_id)
        return best_class_id, best_score

    def _center_distance(self, first: Detection2D, second: Detection2D) -> float:
        first_x = float(first.bbox.center.position.x)
        first_y = float(first.bbox.center.position.y)
        second_x = float(second.bbox.center.position.x)
        second_y = float(second.bbox.center.position.y)
        delta_x = first_x - second_x
        delta_y = first_y - second_y
        return (delta_x * delta_x + delta_y * delta_y) ** 0.5

    def _bbox_iou(self, first: Detection2D, second: Detection2D) -> float:
        first_box = self._bbox_xyxy(first)
        second_box = self._bbox_xyxy(second)
        if first_box is None or second_box is None:
            return 0.0

        first_x1, first_y1, first_x2, first_y2 = first_box
        second_x1, second_y1, second_x2, second_y2 = second_box
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

    def _bbox_xyxy(
        self,
        detection: Detection2D,
    ) -> Optional[Tuple[float, float, float, float]]:
        width = float(detection.bbox.size_x)
        height = float(detection.bbox.size_y)
        if width <= 0.0 or height <= 0.0:
            return None
        center_x = float(detection.bbox.center.position.x)
        center_y = float(detection.bbox.center.position.y)
        half_width = width / 2.0
        half_height = height / 2.0
        return (
            center_x - half_width,
            center_y - half_height,
            center_x + half_width,
            center_y + half_height,
        )
