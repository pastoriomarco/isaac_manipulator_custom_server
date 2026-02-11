import json
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray
from isaac_manipulator_interfaces.action import DetectObjects, EstimatePoseFoundationPose
from isaac_manipulator_interfaces.srv import ClearObjects
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2D

from isaac_manipulator_pose_server.config import WorkflowConfig
from isaac_manipulator_pose_server.services.pipeline import ScanResult, ScanRunConfig


class DirectBinObjectPosePipeline:
    """Direct pipeline that uses detect_objects + estimate_pose_foundation_pose actions."""

    def __init__(self, node: Node, config: WorkflowConfig):
        self._node = node
        self._config = config
        self._clear_objects_service_available = True
        self._clear_objects_service_warned_unavailable = False
        self._detect_objects_client = ActionClient(
            self._node, DetectObjects, self._config.detect_objects_action_name)
        self._estimate_pose_client = ActionClient(
            self._node, EstimatePoseFoundationPose, self._config.estimate_pose_action_name)
        self._clear_objects_client = self._node.create_client(
            ClearObjects, self._config.clear_objects_service_name)

        self._pose_array_pub = self._node.create_publisher(
            PoseArray, self._config.output_pose_array_topic, 10)
        self._summary_pub = self._node.create_publisher(
            String, self._config.output_summary_topic, 10)

    def run_scan(self, run_config: ScanRunConfig) -> ScanResult:
        self._log_state(
            'SCAN_START',
            'Direct pipeline scan with '
            f'max_objects={run_config.max_objects}, '
            f'clear_before={run_config.clear_objects_before_run}, '
            f'clear_after={run_config.clear_objects_after_run}.'
        )
        if run_config.clear_objects_before_run:
            self._try_clear_objects()

        object_pose_records = self._run_pipeline(run_config)
        pose_array, summary = self._build_outputs(object_pose_records)

        self._pose_array_pub.publish(pose_array)
        summary_msg = String()
        summary_msg.data = json.dumps(summary, sort_keys=True)
        self._summary_pub.publish(summary_msg)

        if run_config.clear_objects_after_run:
            self._try_clear_objects()

        self._log_state('SCAN_DONE', f'Finished direct scan with {summary["count"]} object(s).')
        return ScanResult(pose_array=pose_array, summary=summary)

    def _run_pipeline(self, run_config: ScanRunConfig) -> List[Dict]:
        self._wait_for_action_server(
            self._detect_objects_client,
            self._config.detect_objects_action_name,
            self._config.action_timeout_sec
        )
        detect_objects_result = self._send_action_goal_and_wait_result(
            client=self._detect_objects_client,
            goal=DetectObjects.Goal(),
            timeout_sec=self._config.action_timeout_sec,
            action_label='DetectObjects'
        )

        detections = list(detect_objects_result.detections.detections)
        self._log_state('DETECTION', f'DetectObjects returned {len(detections)} detection(s).')
        if not detections:
            raise RuntimeError('DetectObjects returned no detections.')

        effective_target_class_ids = (
            run_config.target_class_ids
            if run_config.target_class_ids is not None
            else self._config.target_class_ids
        )
        candidate_records = self._select_detections(
            detections,
            0,
            effective_target_class_ids,
        )
        target_pose_count = (
            run_config.max_objects if run_config.max_objects > 0 else len(candidate_records)
        )
        self._log_state(
            'SELECTION',
            f'Candidate detections after class/NMS: {len(candidate_records)}. '
            f'Target pose count: {target_pose_count}.'
        )
        for candidate in candidate_records:
            bbox = candidate['detection'].bbox
            self._log_state(
                'CANDIDATE',
                f'id={candidate["object_id"]} class={candidate["class_id"]} '
                f'score={candidate["class_score"]:.4f} '
                f'bbox=({float(bbox.center.position.x):.1f},'
                f'{float(bbox.center.position.y):.1f},'
                f'{float(bbox.size_x):.1f},'
                f'{float(bbox.size_y):.1f})'
            )

        if not candidate_records:
            raise RuntimeError('No detections left after applying class-id and NMS filters.')

        effective_shared_mesh_file_path = (
            run_config.shared_mesh_file_path
            if run_config.shared_mesh_file_path is not None
            else self._config.shared_mesh_file_path
        )

        self._wait_for_action_server(
            self._estimate_pose_client,
            self._config.estimate_pose_action_name,
            self._config.action_timeout_sec
        )
        object_pose_records: List[Dict] = []
        for record in candidate_records:
            if len(object_pose_records) >= target_pose_count:
                break

            goal = EstimatePoseFoundationPose.Goal()
            goal.roi = record['detection']
            goal.use_segmentation_mask = False
            goal.mesh_file_path = effective_shared_mesh_file_path
            goal.object_frame_name = record['frame_name']
            try:
                pose_result = self._send_action_goal_and_wait_result(
                    client=self._estimate_pose_client,
                    goal=goal,
                    timeout_sec=self._config.action_timeout_sec,
                    action_label=f'EstimatePoseFoundationPose({record["object_id"]})',
                    retry_count=self._config.estimate_pose_retry_count,
                )
                record['pose'] = self._extract_pose_from_result(
                    pose_result, record['object_id'])
                object_pose_records.append(record)
            except RuntimeError as exception:
                self._node.get_logger().warning(
                    f'[scan_state] POSE_SKIP: object_id={record["object_id"]} '
                    f'class={record["class_id"]} score={record["class_score"]:.4f} '
                    f'failed: {exception}'
                )

        if not object_pose_records:
            raise RuntimeError(
                'Pose estimation failed for all selected detections.'
            )

        if len(object_pose_records) < target_pose_count:
            self._node.get_logger().warning(
                f'[scan_state] POSE_PARTIAL: requested {target_pose_count}, '
                f'but only {len(object_pose_records)} pose(s) succeeded.'
            )

        return object_pose_records

    def _select_detections(
        self,
        detections: List[Detection2D],
        max_objects: int,
        target_class_ids_filter: List[str],
    ) -> List[Dict]:
        target_class_ids = {str(class_id) for class_id in target_class_ids_filter}
        candidates = []

        for detection_index, detection in enumerate(detections):
            class_id, class_score = self._extract_best_class(detection)
            if target_class_ids and class_id not in target_class_ids:
                continue

            candidates.append({
                'object_id': detection_index,
                'class_id': class_id,
                'class_score': class_score,
                'frame_name': f'{self._config.object_frame_prefix}_{detection_index}',
                'detection': detection,
            })

        candidates.sort(key=lambda record: record['class_score'], reverse=True)
        selected_records = []
        for candidate in candidates:
            if self._is_suppressed_by_nms(candidate, selected_records):
                continue
            selected_records.append(candidate)

        if max_objects > 0:
            selected_records = selected_records[:max_objects]

        return selected_records

    def _is_suppressed_by_nms(self, candidate: Dict, accepted: List[Dict]) -> bool:
        for accepted_record in accepted:
            if accepted_record['class_id'] != candidate['class_id']:
                continue
            iou = self._bbox_iou(candidate['detection'], accepted_record['detection'])
            if iou >= self._config.nms_iou_threshold:
                return True
        return False

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

        first_area = max(0.0, (first_x2 - first_x1)) * max(0.0, (first_y2 - first_y1))
        second_area = max(0.0, (second_x2 - second_x1)) * max(0.0, (second_y2 - second_y1))
        union_area = first_area + second_area - inter_area
        if union_area <= 0.0:
            return 0.0

        return inter_area / union_area

    def _bbox_to_xyxy(self, detection: Detection2D) -> Optional[Tuple[float, float, float, float]]:
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

    def _extract_best_class(self, detection: Detection2D) -> Tuple[str, float]:
        best_class_id = 'unknown'
        best_score = -1.0

        for result in detection.results:
            hypothesis = result.hypothesis
            if hypothesis.score > best_score:
                best_score = float(hypothesis.score)
                best_class_id = str(hypothesis.class_id)

        return best_class_id, best_score

    def _extract_pose_from_result(self, result, object_id: int) -> Pose:
        detections = list(result.poses.detections)
        if not detections:
            raise RuntimeError(
                f'EstimatePoseFoundationPose({object_id}) returned empty Detection3DArray.')
        results = list(detections[0].results)
        if not results:
            raise RuntimeError(
                f'EstimatePoseFoundationPose({object_id}) returned Detection3D with no results.')
        return results[0].pose.pose

    def _build_outputs(self, object_pose_records: List[Dict]) -> Tuple[PoseArray, Dict]:
        pose_array = PoseArray()
        pose_array.header.stamp = self._node.get_clock().now().to_msg()
        pose_array.header.frame_id = self._config.output_frame_id

        summary = {
            'output_frame_id': self._config.output_frame_id,
            'count': len(object_pose_records),
            'objects': []
        }

        for record in object_pose_records:
            pose: Pose = record['pose']
            pose_array.poses.append(pose)
            summary['objects'].append({
                'object_id': record['object_id'],
                'class_id': record['class_id'],
                'class_score': record['class_score'],
                'frame_name': record['frame_name'],
                'pose': {
                    'position': [
                        float(pose.position.x),
                        float(pose.position.y),
                        float(pose.position.z)
                    ],
                    'orientation': [
                        float(pose.orientation.x),
                        float(pose.orientation.y),
                        float(pose.orientation.z),
                        float(pose.orientation.w)
                    ]
                }
            })

        return pose_array, summary

    def _try_clear_objects(self):
        if not self._clear_objects_service_available:
            return

        # In direct mode this service is typically absent, so use a short probe to
        # avoid paying the full service timeout on every scan.
        probe_timeout_sec = min(0.2, max(0.0, self._config.service_timeout_sec))
        if not self._clear_objects_client.wait_for_service(timeout_sec=probe_timeout_sec):
            self._clear_objects_service_available = False
            if not self._clear_objects_service_warned_unavailable:
                self._node.get_logger().warning(
                    'Skipping ClearObjects because the service is unavailable in direct mode.')
                self._clear_objects_service_warned_unavailable = True
            return

        clear_request = ClearObjects.Request()
        clear_response = self._call_service_and_wait_response(
            client=self._clear_objects_client,
            request=clear_request,
            timeout_sec=self._config.service_timeout_sec,
            service_label='ClearObjects'
        )
        self._node.get_logger().info(
            f'ClearObjects removed {clear_response.count} object(s) from cache.')

    def _send_action_goal_and_wait_result_once(
        self,
        client: ActionClient,
        goal,
        timeout_sec: float,
        action_label: str
    ):
        send_goal_future = client.send_goal_async(goal)
        goal_handle = self._wait_for_future(
            send_goal_future, timeout_sec, f'{action_label} goal response')
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'{action_label} goal was rejected.')

        result_future = goal_handle.get_result_async()
        wrapped_result = self._wait_for_action_result_or_cancel(
            goal_handle=goal_handle,
            result_future=result_future,
            timeout_sec=timeout_sec,
            action_label=action_label,
        )
        if wrapped_result is None:
            raise RuntimeError(f'{action_label} returned no result.')

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f'{action_label} failed with status code {wrapped_result.status}.')

        return wrapped_result.result

    def _wait_for_action_result_or_cancel(
        self,
        *,
        goal_handle,
        result_future,
        timeout_sec: float,
        action_label: str
    ):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok():
            if result_future.done():
                try:
                    return result_future.result()
                except Exception as exception:
                    raise RuntimeError(f'{action_label} result failed: {exception}') from exception

            if time.monotonic() >= deadline:
                self._node.get_logger().warning(
                    f'[scan_state] ACTION: {action_label} timed out after '
                    f'{timeout_sec:.1f}s; canceling goal before retry.'
                )
                try:
                    cancel_future = goal_handle.cancel_goal_async()
                    self._wait_for_future(
                        cancel_future,
                        min(2.0, max(0.1, timeout_sec)),
                        f'{action_label} cancel response'
                    )
                except RuntimeError as exception:
                    self._node.get_logger().warning(
                        f'[scan_state] ACTION: {action_label} cancel failed: {exception}'
                    )
                raise RuntimeError(f'{action_label} result timed out after {timeout_sec:.1f} seconds.')

            time.sleep(0.01)

        raise RuntimeError(f'{action_label} result interrupted by ROS shutdown.')

    def _send_action_goal_and_wait_result(
        self,
        client: ActionClient,
        goal,
        timeout_sec: float,
        action_label: str,
        retry_count: Optional[int] = None
    ):
        effective_retry_count = (
            self._config.action_retry_count
            if retry_count is None
            else max(0, int(retry_count))
        )
        return self._run_with_retries(
            state='ACTION',
            operation_label=action_label,
            retry_count=effective_retry_count,
            operation=lambda: self._send_action_goal_and_wait_result_once(
                client=client,
                goal=goal,
                timeout_sec=timeout_sec,
                action_label=action_label,
            )
        )

    def _call_service_and_wait_response(
        self,
        client,
        request,
        timeout_sec: float,
        service_label: str
    ):
        service_future = client.call_async(request)
        response = self._wait_for_future(service_future, timeout_sec, service_label)
        if response is None:
            raise RuntimeError(f'{service_label} returned no response.')
        return response

    def _wait_for_action_server(self, client: ActionClient, action_name: str, timeout_sec: float):
        self._run_with_retries(
            state='WAIT_ACTION_SERVER',
            operation_label=f'wait_for_server({action_name})',
            retry_count=self._config.action_retry_count,
            operation=lambda: self._wait_for_action_server_once(client, action_name, timeout_sec),
        )

    def _wait_for_action_server_once(
        self,
        client: ActionClient,
        action_name: str,
        timeout_sec: float
    ):
        if not client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(
                f'Action server "{action_name}" unavailable after {timeout_sec:.1f} seconds.'
            )

    def _wait_for_future(self, future, timeout_sec: float, label: str):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok():
            if future.done():
                try:
                    return future.result()
                except Exception as exception:
                    raise RuntimeError(f'{label} failed: {exception}') from exception

            if time.monotonic() >= deadline:
                raise RuntimeError(f'{label} timed out after {timeout_sec:.1f} seconds.')

            time.sleep(0.01)

        raise RuntimeError(f'{label} interrupted by ROS shutdown.')

    def _run_with_retries(
        self,
        *,
        state: str,
        operation_label: str,
        retry_count: int,
        operation: Callable[[], Any],
    ) -> Any:
        total_attempts = max(1, retry_count + 1)
        final_error: Optional[Exception] = None

        for attempt in range(1, total_attempts + 1):
            self._log_state(
                state,
                f'{operation_label}: attempt {attempt}/{total_attempts}.'
            )
            try:
                return operation()
            except RuntimeError as exception:
                final_error = exception
                if attempt >= total_attempts:
                    break
                self._node.get_logger().warning(
                    f'[scan_state] {state}: {operation_label} failed on attempt '
                    f'{attempt}/{total_attempts}: {exception}. '
                    f'Retrying in {self._config.retry_backoff_sec:.2f}s.'
                )
                if self._config.retry_backoff_sec > 0.0:
                    time.sleep(self._config.retry_backoff_sec)

        raise RuntimeError(
            f'{operation_label} failed after {total_attempts} attempt(s): {final_error}'
        )

    def _log_state(self, state: str, message: str):
        self._node.get_logger().info(f'[scan_state] {state}: {message}')
