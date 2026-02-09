import json
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray
from isaac_manipulator_interfaces.action import GetObjectPose, GetObjects
from isaac_manipulator_interfaces.msg import ObjectInfo
from isaac_manipulator_interfaces.srv import AddMeshToObject, AssignNameToObject, ClearObjects
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from isaac_manipulator_pose_server.config import WorkflowConfig


@dataclass
class ScanRunConfig:
    max_objects: int
    clear_objects_before_run: bool
    clear_objects_after_run: bool


@dataclass
class ScanResult:
    pose_array: PoseArray
    summary: Dict


class BinObjectPosePipeline:
    """Pipeline that queries Isaac Manipulator servers and aggregates object poses."""

    def __init__(self, node: Node, config: WorkflowConfig):
        self._node = node
        self._config = config

        self._get_objects_client = ActionClient(
            self._node, GetObjects, self._config.get_objects_action_name)
        self._get_object_pose_client = ActionClient(
            self._node, GetObjectPose, self._config.get_object_pose_action_name)
        self._add_mesh_client = self._node.create_client(
            AddMeshToObject, self._config.add_mesh_service_name)
        self._assign_name_client = self._node.create_client(
            AssignNameToObject, self._config.assign_name_service_name)
        self._clear_objects_client = self._node.create_client(
            ClearObjects, self._config.clear_objects_service_name)

        self._pose_array_pub = self._node.create_publisher(
            PoseArray, self._config.output_pose_array_topic, 10)
        self._summary_pub = self._node.create_publisher(
            String, self._config.output_summary_topic, 10)

    def run_scan(self, run_config: ScanRunConfig) -> ScanResult:
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

        return ScanResult(pose_array=pose_array, summary=summary)

    def _run_pipeline(self, run_config: ScanRunConfig) -> List[Dict]:
        self._wait_for_action_server(
            self._get_objects_client,
            self._config.get_objects_action_name,
            self._config.action_timeout_sec)

        get_objects_result = self._send_action_goal_and_wait_result(
            client=self._get_objects_client,
            goal=GetObjects.Goal(),
            timeout_sec=self._config.action_timeout_sec,
            action_label='GetObjects'
        )

        detected_objects = list(get_objects_result.objects)
        if not detected_objects:
            raise RuntimeError('GetObjects returned no detections.')

        selected_records = self._select_objects(detected_objects, run_config.max_objects)
        if not selected_records:
            raise RuntimeError('No objects left after applying class-id filter.')

        if self._config.shared_mesh_file_path:
            self._wait_for_service(
                self._add_mesh_client,
                self._config.add_mesh_service_name,
                self._config.service_timeout_sec
            )
            add_mesh_request = AddMeshToObject.Request()
            add_mesh_request.object_ids = [record['object_id'] for record in selected_records]
            add_mesh_request.mesh_file_paths = [
                self._config.shared_mesh_file_path for _ in selected_records
            ]
            add_mesh_response = self._call_service_and_wait_response(
                client=self._add_mesh_client,
                request=add_mesh_request,
                timeout_sec=self._config.service_timeout_sec,
                service_label='AddMeshToObject'
            )
            if not add_mesh_response.success:
                raise RuntimeError(
                    'AddMeshToObject failed: '
                    f'{add_mesh_response.message} (failed_ids={list(add_mesh_response.failed_ids)})'
                )

        self._wait_for_service(
            self._assign_name_client,
            self._config.assign_name_service_name,
            self._config.service_timeout_sec
        )
        for record in selected_records:
            assign_name_request = AssignNameToObject.Request()
            assign_name_request.object_id = record['object_id']
            assign_name_request.name = record['frame_name']
            assign_name_response = self._call_service_and_wait_response(
                client=self._assign_name_client,
                request=assign_name_request,
                timeout_sec=self._config.service_timeout_sec,
                service_label=f'AssignNameToObject({record["object_id"]})'
            )
            if not assign_name_response.result:
                raise RuntimeError(
                    f'AssignNameToObject failed for object_id={record["object_id"]}.')

        self._wait_for_action_server(
            self._get_object_pose_client,
            self._config.get_object_pose_action_name,
            self._config.action_timeout_sec
        )
        for record in selected_records:
            get_object_pose_goal = GetObjectPose.Goal()
            get_object_pose_goal.object_id = record['object_id']
            get_object_pose_goal.class_id = record['class_id']

            get_object_pose_result = self._send_action_goal_and_wait_result(
                client=self._get_object_pose_client,
                goal=get_object_pose_goal,
                timeout_sec=self._config.action_timeout_sec,
                action_label=f'GetObjectPose({record["object_id"]})'
            )
            record['pose'] = get_object_pose_result.object_pose

        return selected_records

    def _select_objects(self, detected_objects: List[ObjectInfo], max_objects: int) -> List[Dict]:
        target_class_ids = {str(class_id) for class_id in self._config.target_class_ids}
        selected_records = []

        for detected_object in detected_objects:
            class_id, class_score = self._extract_best_class(detected_object)
            if target_class_ids and class_id not in target_class_ids:
                continue

            selected_records.append({
                'object_id': int(detected_object.object_id),
                'class_id': class_id,
                'class_score': class_score,
                'frame_name': f'{self._config.object_frame_prefix}_{detected_object.object_id}'
            })

        if max_objects > 0:
            selected_records = selected_records[:max_objects]

        return selected_records

    def _extract_best_class(self, detected_object: ObjectInfo) -> Tuple[str, float]:
        best_class_id = 'unknown'
        best_score = -1.0

        if detected_object.detection_2d and detected_object.detection_2d.results:
            for result in detected_object.detection_2d.results:
                hypothesis = result.hypothesis
                if hypothesis.score > best_score:
                    best_score = float(hypothesis.score)
                    best_class_id = str(hypothesis.class_id)

        return best_class_id, best_score

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
        if not self._clear_objects_client.wait_for_service(
                timeout_sec=self._config.service_timeout_sec):
            self._node.get_logger().warning(
                'Skipping ClearObjects because the service is unavailable.')
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

    def _send_action_goal_and_wait_result(
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
        wrapped_result = self._wait_for_future(
            result_future, timeout_sec, f'{action_label} result')
        if wrapped_result is None:
            raise RuntimeError(f'{action_label} returned no result.')

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f'{action_label} failed with status code {wrapped_result.status}.')

        return wrapped_result.result

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
        if not client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(
                f'Action server "{action_name}" unavailable after {timeout_sec:.1f} seconds.'
            )

    def _wait_for_service(self, client, service_name: str, timeout_sec: float):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(
                f'Service "{service_name}" unavailable after {timeout_sec:.1f} seconds.'
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


def summary_to_response_fields(summary: Optional[Dict]) -> Tuple[str, int, List[int], List[str],
                                                           List[float], List[str], List[Pose], str]:
    if not summary:
        return '', 0, [], [], [], [], [], ''

    output_frame_id = str(summary.get('output_frame_id', ''))
    count = int(summary.get('count', 0))

    object_ids: List[int] = []
    class_ids: List[str] = []
    class_scores: List[float] = []
    frame_names: List[str] = []
    poses: List[Pose] = []

    for obj in summary.get('objects', []):
        object_ids.append(int(obj.get('object_id', -1)))
        class_ids.append(str(obj.get('class_id', '')))
        class_scores.append(float(obj.get('class_score', 0.0)))
        frame_names.append(str(obj.get('frame_name', '')))

        pose = Pose()
        position = obj.get('pose', {}).get('position', [0.0, 0.0, 0.0])
        orientation = obj.get('pose', {}).get('orientation', [0.0, 0.0, 0.0, 1.0])
        if len(position) >= 3:
            pose.position.x = float(position[0])
            pose.position.y = float(position[1])
            pose.position.z = float(position[2])
        if len(orientation) >= 4:
            pose.orientation.x = float(orientation[0])
            pose.orientation.y = float(orientation[1])
            pose.orientation.z = float(orientation[2])
            pose.orientation.w = float(orientation[3])
        poses.append(pose)

    return (
        output_frame_id,
        count,
        object_ids,
        class_ids,
        class_scores,
        frame_names,
        poses,
        json.dumps(summary, sort_keys=True),
    )
