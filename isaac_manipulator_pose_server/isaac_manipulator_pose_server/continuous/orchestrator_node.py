import argparse
import json
import os
import threading
import time
from dataclasses import dataclass, replace
from typing import Dict, List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray
from isaac_manipulator_interfaces.srv import ClearObjects
from isaac_manipulator_server_interfaces.msg import (
    PipelineCommand,
    PipelineStatus,
    TrackedObject,
    TrackedObjectArray,
)
from isaac_manipulator_server_interfaces.srv import GetLastScan, MarkObjectUsed, ScanBinObjects
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, ObjectHypothesis, ObjectHypothesisWithPose

from isaac_manipulator_pose_server.config import ObjectProfile, WorkflowConfig, load_config
from isaac_manipulator_pose_server.continuous.detector_cache import DetectorCache
from isaac_manipulator_pose_server.continuous.legacy_scan_adapter import LegacyScanAdapter
from isaac_manipulator_pose_server.continuous.models import Candidate, WorkerResult
from isaac_manipulator_pose_server.continuous.object_registry import ObjectRegistry
from isaac_manipulator_pose_server.continuous.worker_pool import FPWorkerPool


@dataclass
class _PendingJob:
    object_id: int
    worker_id: str
    frame_name: str
    candidate: Candidate
    tracking_refresh: bool


def _default_config_file() -> str:
    try:
        package_share_directory = get_package_share_directory('isaac_manipulator_pose_server')
        return os.path.join(package_share_directory, 'params', 'pose_server.yaml')
    except Exception:
        return 'pose_server.yaml'


class MultiObjectPoseOrchestratorServer:
    """Continuous multi-worker orchestration server with legacy scan compatibility wrappers."""

    def __init__(self, config_file: str, fp_worker_count_override: Optional[int] = None):
        self._config: WorkflowConfig = load_config(config_file)
        if fp_worker_count_override is not None:
            worker_count = max(1, int(fp_worker_count_override))
            worker_namespaces = [f'/fp_worker_{index}' for index in range(worker_count)]
            self._config = replace(
                self._config,
                fp_worker_count=worker_count,
                fp_worker_namespaces=worker_namespaces,
            )

        self._state_lock = threading.Lock()
        self._state_cv = threading.Condition(self._state_lock)
        self._mode = PipelineStatus.IDLE
        self._target_count = self._config.target_count_default
        self._active_target_class_ids = list(self._config.target_class_ids)
        self._active_mesh_file_path = self._config.shared_mesh_file_path
        self._active_object_key = ''
        self._last_summary: Optional[Dict] = None
        self._last_expected_count = 0
        self._last_expected_count_met = True
        self._last_error = ''
        self._last_candidate_count = 0
        self._pending_jobs: Dict[int, _PendingJob] = {}
        self._last_scheduler_debug_sec = 0.0
        self._last_cmd_debug_sec = 0.0
        self._last_no_candidates_debug_sec = 0.0

        rclpy.init()
        # Keep the legacy node name used by existing launch and tooling.
        self._node = rclpy.create_node('multi_object_pose_server_direct')
        self._callback_group = ReentrantCallbackGroup()

        self._legacy_pose_array_pub = self._node.create_publisher(
            PoseArray, self._config.output_pose_array_topic, 10
        )
        self._legacy_summary_pub = self._node.create_publisher(
            String, self._config.output_summary_topic, 10
        )
        self._tracked_objects_pub = self._node.create_publisher(
            TrackedObjectArray, self._config.tracked_objects_topic_name, 10
        )
        self._pipeline_status_pub = self._node.create_publisher(
            PipelineStatus, self._config.pipeline_status_topic_name, 10
        )

        self._clear_objects_client = self._node.create_client(
            ClearObjects, self._config.clear_objects_service_name
        )

        self._detector_cache = DetectorCache(self._node, self._config)
        self._registry = ObjectRegistry(self._node, self._config)
        self._worker_pool = FPWorkerPool(
            node=self._node,
            config=self._config,
            worker_namespaces=self._config.fp_worker_namespaces,
            result_callback=self._on_worker_result,
        )
        self._legacy_adapter = LegacyScanAdapter(self)

        self._pipeline_command_sub = self._node.create_subscription(
            PipelineCommand,
            self._config.pipeline_command_topic_name,
            self._handle_pipeline_command,
            10,
            callback_group=self._callback_group,
        )
        self._mark_object_used_service = self._node.create_service(
            MarkObjectUsed,
            self._config.mark_object_used_service_name,
            self._handle_mark_object_used,
            callback_group=self._callback_group,
        )
        self._scan_service = self._node.create_service(
            ScanBinObjects,
            '/isaac_manipulator_pose_server/scan_bin_objects',
            self._legacy_adapter.handle_scan_bin_objects,
            callback_group=self._callback_group,
        )
        self._last_scan_service = self._node.create_service(
            GetLastScan,
            '/isaac_manipulator_pose_server/get_last_scan',
            self._legacy_adapter.handle_get_last_scan,
            callback_group=self._callback_group,
        )

        self._scheduler_timer = self._node.create_timer(
            1.0 / self._config.detector_processing_hz,
            self._scheduler_tick,
            callback_group=self._callback_group,
        )
        self._tracked_publish_timer = self._node.create_timer(
            1.0 / self._config.tracked_objects_publish_hz,
            self._publish_tracked_outputs,
            callback_group=self._callback_group,
        )
        self._status_publish_timer = self._node.create_timer(
            1.0 / self._config.pipeline_status_publish_hz,
            self._publish_pipeline_status,
            callback_group=self._callback_group,
        )

        self._executor = rclpy.executors.MultiThreadedExecutor()
        self._executor.add_node(self._node)

    def _resolve_object_selection(self, object_key: str) -> Tuple[Optional[List[str]], Optional[str]]:
        if not object_key:
            return None, None

        object_profile = self._config.available_objects.get(object_key)
        if object_profile is None:
            available = ', '.join(sorted(self._config.available_objects.keys())) or '<none>'
            raise RuntimeError(
                f'Unknown object_key "{object_key}". Available object keys: {available}.'
            )
        return [object_profile.class_id], object_profile.mesh_file_path

    def _set_active_profile(self, object_key: str):
        selected_class_ids, selected_mesh = self._resolve_object_selection(object_key)
        if selected_class_ids is None:
            self._active_target_class_ids = list(self._config.target_class_ids)
            self._active_mesh_file_path = self._config.shared_mesh_file_path
            self._active_object_key = ''
            return
        self._active_target_class_ids = selected_class_ids
        self._active_mesh_file_path = selected_mesh or self._config.shared_mesh_file_path
        self._active_object_key = object_key

    def _handle_pipeline_command(self, msg: PipelineCommand):
        try:
            self._node.get_logger().info(
                f'[orchestrator] pipeline_command: command={int(msg.command)} '
                f'target_count={int(msg.target_count)} object_key={str(msg.object_key)!r} '
                f'clear_before_start={bool(msg.clear_before_start)} '
                f'clear_after_stop={bool(msg.clear_after_stop)}'
            )
            if msg.command == PipelineCommand.START:
                if msg.object_key:
                    self._set_active_profile(str(msg.object_key).strip())
                if msg.target_count >= 0:
                    self._target_count = int(msg.target_count)
                if msg.clear_before_start:
                    self._reset_pipeline(clear_cache=True)
                self._mode = PipelineStatus.RUNNING
            elif msg.command == PipelineCommand.STOP:
                self._mode = PipelineStatus.STOPPED
                self._worker_pool.cancel_all()
                if msg.clear_after_stop:
                    self._reset_pipeline(clear_cache=True)
            elif msg.command == PipelineCommand.RESET:
                self._reset_pipeline(clear_cache=True)
                self._mode = PipelineStatus.IDLE
            elif msg.command == PipelineCommand.SET_TARGET:
                if msg.target_count >= 0:
                    self._target_count = int(msg.target_count)
                if msg.object_key:
                    self._set_active_profile(str(msg.object_key).strip())
            else:
                self._last_error = f'Unknown pipeline command: {msg.command}'
                self._node.get_logger().error(self._last_error)
            self._notify_waiters()
        except RuntimeError as exception:
            self._last_error = str(exception)
            self._mode = PipelineStatus.ERROR
            self._node.get_logger().error(self._last_error)
            self._notify_waiters()

    def _handle_mark_object_used(
        self,
        request: MarkObjectUsed.Request,
        response: MarkObjectUsed.Response,
    ) -> MarkObjectUsed.Response:
        object_id = int(request.object_id)
        record = self._registry.get_record(object_id)
        if record is None:
            response.success = False
            response.object_was_active = False
            response.message = f'Object {object_id} not found.'
            return response

        if request.frame_name and str(request.frame_name) != record.frame_name:
            response.success = False
            response.object_was_active = False
            response.message = (
                f'Frame mismatch for object {object_id}: '
                f'expected "{record.frame_name}", got "{request.frame_name}".'
            )
            return response

        was_active = record.status == TrackedObject.TRACKING
        success, worker_id = self._registry.mark_used(object_id)
        if not success:
            response.success = False
            response.object_was_active = False
            response.message = f'Failed to mark object {object_id} as used.'
            return response
        if worker_id:
            self._worker_pool.cancel_worker(worker_id)
        response.success = True
        response.object_was_active = was_active
        response.message = f'Object {object_id} marked used.'
        self._notify_waiters()
        return response

    def _notify_waiters(self):
        with self._state_cv:
            self._state_cv.notify_all()

    def _scheduler_tick(self):
        if self._mode != PipelineStatus.RUNNING:
            return

        now_sec = time.monotonic()
        if now_sec - self._last_scheduler_debug_sec >= 5.0:
            self._last_scheduler_debug_sec = now_sec
            self._node.get_logger().info(
                f'[orchestrator] scheduler tick: target_count={self._target_count} '
                f'active={self._registry.count_active_tracking()} '
                f'queued={self._last_candidate_count} '
                f'workers_available={len(self._worker_pool.available_worker_ids(now_sec))} '
                f'workers_busy={self._worker_pool.busy_count()} '
                f'detector_topic="{self._config.detections_topic_name}"'
            )
        self._worker_pool.check_timeouts(now_sec)
        self._registry.prune_lost(now_sec, self._config.object_lost_ttl_sec)

        self._dispatch_tracking_refresh(now_sec)
        self._dispatch_new_candidates(now_sec)

    def _dispatch_tracking_refresh(self, now_sec: float):
        refresh_period_sec = max(0.2, self._config.tracking_update_timeout_sec * 0.5)
        assignments = self._registry.active_worker_assignments()
        available = set(self._worker_pool.available_worker_ids(now_sec))
        for worker_id, object_id in assignments.items():
            if worker_id not in available:
                continue
            if not self._registry.should_refresh_tracking(
                object_id=object_id,
                refresh_period_sec=refresh_period_sec,
                now_sec=now_sec,
            ):
                continue
            record = self._registry.get_record(object_id)
            if record is None:
                continue
            candidate = self._candidate_from_bbox(
                class_id=record.class_id,
                class_score=record.class_score,
                bbox=record.bbox,
                receive_time_sec=now_sec,
            )
            dispatched = self._worker_pool.dispatch(
                worker_id=worker_id,
                object_id=object_id,
                frame_name=record.frame_name,
                candidate=candidate,
                mesh_file_path=self._active_mesh_file_path,
                timeout_sec=self._config.tracking_update_timeout_sec,
                tracking_refresh=True,
            )
            if dispatched:
                with self._state_lock:
                    self._pending_jobs[object_id] = _PendingJob(
                        object_id=object_id,
                        worker_id=worker_id,
                        frame_name=record.frame_name,
                        candidate=candidate,
                        tracking_refresh=True,
                    )

    def _dispatch_new_candidates(self, now_sec: float):
        active_count = self._registry.count_active_tracking()
        if self._target_count > 0 and active_count >= self._target_count:
            return

        available_workers = self._worker_pool.available_worker_ids(now_sec)
        if not available_workers:
            return

        candidates = self._detector_cache.get_candidates(
            target_class_ids=self._active_target_class_ids,
            max_candidates=self._config.max_candidates_per_cycle,
            min_receive_time_sec=0.0,
        )
        self._last_candidate_count = len(candidates)
        self._node.get_logger().debug(
            f'[orchestrator] candidate scan: mode=RUNNING target_count={self._target_count} '
            f'active_count={active_count} candidate_count={self._last_candidate_count}'
        )
        if not candidates:
            if now_sec - self._last_no_candidates_debug_sec >= 2.0:
                self._last_no_candidates_debug_sec = now_sec
                self._node.get_logger().warning(
                    '[orchestrator] no candidates available from detection cache. '
                    'Check detections topic is publishing on '
                    f'{self._config.detections_topic_name}.'
                )
            return

        needed = len(available_workers)
        if self._target_count > 0:
            needed = min(needed, max(0, self._target_count - active_count))
        if needed <= 0:
            return

        scheduled = 0
        for candidate in candidates:
            if scheduled >= needed:
                break
            if self._registry.is_candidate_duplicate(candidate):
                continue
            if self._pending_duplicate(candidate):
                continue
            if scheduled >= len(available_workers):
                break

            worker_id = available_workers[scheduled]
            object_id = self._registry.allocate_object_id()
            frame_name = f'{self._config.object_frame_prefix}_{object_id}'
            dispatched = self._worker_pool.dispatch(
                worker_id=worker_id,
                object_id=object_id,
                frame_name=frame_name,
                candidate=candidate,
                mesh_file_path=self._active_mesh_file_path,
                timeout_sec=self._config.first_pose_timeout_sec,
                tracking_refresh=False,
            )
            if not dispatched:
                continue
            with self._state_lock:
                self._pending_jobs[object_id] = _PendingJob(
                    object_id=object_id,
                    worker_id=worker_id,
                    frame_name=frame_name,
                    candidate=candidate,
                    tracking_refresh=False,
                )
            scheduled += 1

    def _pending_duplicate(self, candidate: Candidate) -> bool:
        with self._state_lock:
            for job in self._pending_jobs.values():
                if job.candidate.class_id != candidate.class_id:
                    continue
                if self._bbox_iou(candidate.detection, job.candidate.detection) >= (
                    self._config.nms_iou_threshold
                ):
                    return True
        return False

    def _on_worker_result(self, result: WorkerResult):
        with self._state_lock:
            self._pending_jobs.pop(result.object_id, None)
        if result.success and result.pose is not None:
            if result.was_tracking_refresh:
                updated = self._registry.update_tracking(
                    object_id=result.object_id,
                    candidate=result.candidate,
                    pose=result.pose,
                )
                if not updated:
                    self._registry.add_tracking(
                        object_id=result.object_id,
                        frame_name=result.frame_name,
                        worker_id=result.worker_id,
                        candidate=result.candidate,
                        pose=result.pose,
                    )
            else:
                self._registry.add_tracking(
                    object_id=result.object_id,
                    frame_name=result.frame_name,
                    worker_id=result.worker_id,
                    candidate=result.candidate,
                    pose=result.pose,
                )
            self._last_error = ''
        else:
            if result.was_tracking_refresh:
                self._registry.mark_lost(result.object_id)
            if result.error:
                self._last_error = result.error
                self._node.get_logger().warning(
                    f'[orchestrator] worker={result.worker_id} object_id={result.object_id}: '
                    f'{result.error}'
                )
        self._notify_waiters()

    def _publish_tracked_outputs(self):
        tracked_objects = self._registry.snapshot_tracked_objects(self._target_count)
        self._tracked_objects_pub.publish(tracked_objects)

        summary = self._registry.summary()
        with self._state_lock:
            self._last_summary = summary

        pose_array = PoseArray()
        pose_array.header.stamp = self._node.get_clock().now().to_msg()
        pose_array.header.frame_id = self._config.output_frame_id
        for record in summary['objects']:
            pose = Pose()
            position = record.get('pose', {}).get('position', [0.0, 0.0, 0.0])
            orientation = record.get('pose', {}).get('orientation', [0.0, 0.0, 0.0, 1.0])
            pose.position.x = float(position[0])
            pose.position.y = float(position[1])
            pose.position.z = float(position[2])
            pose.orientation.x = float(orientation[0])
            pose.orientation.y = float(orientation[1])
            pose.orientation.z = float(orientation[2])
            pose.orientation.w = float(orientation[3])
            pose_array.poses.append(pose)
        self._legacy_pose_array_pub.publish(pose_array)

        summary_msg = String()
        summary_msg.data = json.dumps(summary, sort_keys=True)
        self._legacy_summary_pub.publish(summary_msg)

    def _publish_pipeline_status(self):
        now_sec = time.monotonic()
        status = PipelineStatus()
        status.header.stamp = self._node.get_clock().now().to_msg()
        status.header.frame_id = self._config.output_frame_id
        status.mode = int(self._mode)
        status.target_count = int(self._target_count)
        status.active_count = int(self._registry.count_active_tracking())
        status.queued_candidates = int(self._last_candidate_count)
        status.available_workers = int(len(self._worker_pool.available_worker_ids(now_sec)))
        status.busy_workers = int(self._worker_pool.busy_count())
        status.dropped_workers = self._worker_pool.dropped_worker_ids()
        status.last_error = self._last_error
        self._pipeline_status_pub.publish(status)

    def _reset_pipeline(self, clear_cache: bool):
        self._worker_pool.cancel_all()
        with self._state_lock:
            self._pending_jobs.clear()
        self._registry.clear()
        if clear_cache:
            self._try_clear_objects()

    def _try_clear_objects(self):
        probe_timeout_sec = min(0.2, max(0.0, self._config.service_timeout_sec))
        if not self._clear_objects_client.wait_for_service(timeout_sec=probe_timeout_sec):
            return
        request = ClearObjects.Request()
        future = self._clear_objects_client.call_async(request)
        deadline = time.monotonic() + self._config.service_timeout_sec
        while rclpy.ok():
            if future.done():
                return
            if time.monotonic() >= deadline:
                return
            time.sleep(0.01)

    def run_blocking_scan(
        self,
        *,
        max_objects: int,
        expected_count: int,
        object_key: str,
        clear_before: bool,
        clear_after: bool,
    ) -> Tuple[Dict, int, bool, str]:
        try:
            if max_objects < 0:
                target_count = int(self._config.max_objects)
            else:
                target_count = int(max_objects)

            if object_key:
                self._set_active_profile(object_key)
            else:
                self._set_active_profile('')

            self._target_count = max(0, target_count)
            if clear_before:
                self._reset_pipeline(clear_cache=True)
            self._mode = PipelineStatus.RUNNING

            deadline = time.monotonic() + self._config.scan_service_wait_timeout_sec
            with self._state_cv:
                while time.monotonic() < deadline:
                    count = self._registry.count_active_tracking()
                    if expected_count > 0 and count >= expected_count:
                        break
                    if expected_count <= 0:
                        if self._target_count > 0 and count >= self._target_count:
                            break
                        if self._target_count == 0 and count > 0:
                            break
                    self._state_cv.wait(timeout=1.0 / self._config.scan_service_poll_hz)

            summary = self._registry.summary()
            expected_count_met = True
            if expected_count > 0:
                expected_count_met = summary['count'] >= expected_count
            message = (
                f'Scan complete: {summary["count"]} object(s) found.'
                if expected_count_met else
                f'Scan complete but expected_count={expected_count} '
                f'was not met (found {summary["count"]}).'
            )
            if object_key:
                message = f'{message} object_key={object_key}.'

            with self._state_lock:
                self._last_summary = summary
                self._last_expected_count = expected_count
                self._last_expected_count_met = expected_count_met

            if clear_after:
                self._reset_pipeline(clear_cache=True)
            self._notify_waiters()
            return summary, expected_count, expected_count_met, message
        except RuntimeError as exception:
            summary = self._registry.summary()
            with self._state_lock:
                self._last_summary = summary
                self._last_expected_count = expected_count
                self._last_expected_count_met = False
            return summary, expected_count, False, str(exception)

    def get_last_scan_state(self) -> Tuple[Optional[Dict], int, bool]:
        with self._state_lock:
            return self._last_summary, self._last_expected_count, self._last_expected_count_met

    def _candidate_from_bbox(
        self,
        *,
        class_id: str,
        class_score: float,
        bbox,
        receive_time_sec: float,
    ) -> Candidate:
        detection = Detection2D()
        detection.bbox = bbox
        hypothesis = ObjectHypothesis()
        hypothesis.class_id = class_id
        hypothesis.score = float(class_score)
        hypothesis_with_pose = ObjectHypothesisWithPose()
        hypothesis_with_pose.hypothesis = hypothesis
        detection.results = [hypothesis_with_pose]
        return Candidate(
            class_id=class_id,
            class_score=float(class_score),
            detection=detection,
            received_monotonic_sec=receive_time_sec,
        )

    def _bbox_iou(self, first: Detection2D, second: Detection2D) -> float:
        first_box = self._bbox_to_xyxy(first)
        second_box = self._bbox_to_xyxy(second)
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

    def _bbox_to_xyxy(self, detection: Detection2D):
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

    def spin(self):
        self._node.get_logger().info('multi_object_pose_orchestrator is ready.')
        self._executor.spin()

    def shutdown(self):
        self._worker_pool.cancel_all()
        self._executor.shutdown()
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Continuous multi-worker pose orchestrator with legacy scan adapters.'
    )
    parser.add_argument(
        '--config-file',
        type=str,
        default=_default_config_file(),
        help='Path to workflow config YAML.',
    )
    parser.add_argument(
        '--fp-worker-count',
        type=int,
        default=None,
        help='Override worker count and /fp_worker_i namespace list from config.',
    )
    return parser


def main() -> int:
    args, _ = _build_arg_parser().parse_known_args()
    server = MultiObjectPoseOrchestratorServer(
        config_file=args.config_file,
        fp_worker_count_override=args.fp_worker_count,
    )
    try:
        server.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
    return 0
