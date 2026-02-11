import argparse
import os
import threading
from typing import List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from isaac_manipulator_pose_server.config import load_config
from isaac_manipulator_pose_server.services import (
    DirectBinObjectPosePipeline,
    ScanRunConfig,
    summary_to_response_fields,
)
from isaac_manipulator_server_interfaces.srv import GetLastScan, ScanBinObjects
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup


def _default_config_file() -> str:
    try:
        package_share_directory = get_package_share_directory(
            'isaac_manipulator_pose_server')
        return os.path.join(package_share_directory, 'params', 'pose_server.yaml')
    except Exception:
        return 'pose_server.yaml'


class MultiObjectPoseServerDirect:
    """Persistent service server for direct multi-object scan orchestration."""

    def __init__(self, config_file: str):
        self._config = load_config(config_file)
        self._scan_lock = threading.Lock()
        self._last_summary = None
        self._last_expected_count = 0
        self._last_expected_count_met = True

        rclpy.init()
        self._node = rclpy.create_node('multi_object_pose_server_direct')
        self._pipeline = DirectBinObjectPosePipeline(node=self._node, config=self._config)

        callback_group = ReentrantCallbackGroup()
        self._scan_service = self._node.create_service(
            ScanBinObjects,
            '/isaac_manipulator_pose_server/scan_bin_objects',
            self._handle_scan_bin_objects,
            callback_group=callback_group,
        )
        self._last_scan_service = self._node.create_service(
            GetLastScan,
            '/isaac_manipulator_pose_server/get_last_scan',
            self._handle_get_last_scan,
            callback_group=callback_group,
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

    def _handle_scan_bin_objects(
        self,
        request: ScanBinObjects.Request,
        response: ScanBinObjects.Response,
    ) -> ScanBinObjects.Response:
        if not self._scan_lock.acquire(blocking=False):
            response.success = False
            response.message = 'Scan already in progress.'
            return response

        try:
            max_objects = int(request.max_objects)
            if max_objects < 0:
                max_objects = int(self._config.max_objects)

            object_key = str(request.object_key).strip()
            selected_class_ids, selected_mesh_file_path = self._resolve_object_selection(object_key)

            run_config = ScanRunConfig(
                max_objects=max_objects,
                target_class_ids=selected_class_ids,
                shared_mesh_file_path=selected_mesh_file_path,
                clear_objects_before_run=bool(request.clear_objects_before_run),
                clear_objects_after_run=bool(request.clear_objects_after_run),
            )

            result = self._pipeline.run_scan(run_config=run_config)

            expected_count = int(request.expected_count)
            expected_count_met = True
            if expected_count > 0:
                expected_count_met = int(result.summary['count']) >= expected_count

            self._last_summary = result.summary
            self._last_expected_count = expected_count
            self._last_expected_count_met = expected_count_met

            (
                output_frame_id,
                count,
                object_ids,
                class_ids,
                class_scores,
                frame_names,
                poses,
                summary_json,
            ) = summary_to_response_fields(result.summary)

            response.success = expected_count_met
            response.message = (
                f'Scan complete: {count} object(s) found.'
                if expected_count_met else
                f'Scan complete but expected_count={expected_count} was not met (found {count}).'
            )
            if object_key:
                response.message = f'{response.message} object_key={object_key}.'
            response.output_frame_id = output_frame_id
            response.count = count
            response.expected_count = expected_count
            response.expected_count_met = expected_count_met
            response.object_ids = object_ids
            response.class_ids = class_ids
            response.class_scores = class_scores
            response.frame_names = frame_names
            response.poses = poses
            response.summary_json = summary_json

            self._node.get_logger().info(response.message)
            return response
        except RuntimeError as exception:
            response.success = False
            response.message = str(exception)
            self._node.get_logger().error(f'[scan_bin_objects] {exception}')
            return response
        finally:
            self._scan_lock.release()

    def _handle_get_last_scan(
        self,
        request: GetLastScan.Request,
        response: GetLastScan.Response,
    ) -> GetLastScan.Response:
        del request

        if self._last_summary is None:
            response.success = False
            response.message = 'No scan has been run yet.'
            return response

        (
            output_frame_id,
            count,
            object_ids,
            class_ids,
            class_scores,
            frame_names,
            poses,
            summary_json,
        ) = summary_to_response_fields(self._last_summary)

        response.success = True
        response.message = f'Returned last scan with {count} object(s).'
        response.output_frame_id = output_frame_id
        response.count = count
        response.expected_count = self._last_expected_count
        response.expected_count_met = self._last_expected_count_met
        response.object_ids = object_ids
        response.class_ids = class_ids
        response.class_scores = class_scores
        response.frame_names = frame_names
        response.poses = poses
        response.summary_json = summary_json
        return response

    def spin(self):
        self._node.get_logger().info('multi_object_pose_server_direct is ready.')
        self._executor.spin()

    def shutdown(self):
        self._executor.shutdown()
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Persistent direct server for multi-object bin pose scans.')
    parser.add_argument(
        '--config-file',
        type=str,
        default=_default_config_file(),
        help='Path to workflow config YAML.'
    )
    return parser


def main():
    args, _ = _build_arg_parser().parse_known_args()
    server = MultiObjectPoseServerDirect(config_file=args.config_file)
    try:
        server.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
