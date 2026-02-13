from typing import Optional, Tuple

from isaac_manipulator_server_interfaces.srv import GetLastScan, ScanBinObjects

from isaac_manipulator_pose_server.services.pipeline import summary_to_response_fields


class LegacyScanAdapter:
    """Compatibility wrapper that maps legacy scan services onto continuous mode."""

    def __init__(self, orchestrator):
        self._orchestrator = orchestrator

    def handle_scan_bin_objects(
        self,
        request: ScanBinObjects.Request,
        response: ScanBinObjects.Response,
    ) -> ScanBinObjects.Response:
        summary, expected_count, expected_count_met, message = self._orchestrator.run_blocking_scan(
            max_objects=int(request.max_objects),
            expected_count=int(request.expected_count),
            object_key=str(request.object_key).strip(),
            clear_before=bool(request.clear_objects_before_run),
            clear_after=bool(request.clear_objects_after_run),
        )

        (
            output_frame_id,
            count,
            object_ids,
            class_ids,
            class_scores,
            frame_names,
            poses,
            summary_json,
        ) = summary_to_response_fields(summary)

        response.success = expected_count_met
        response.message = message
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
        return response

    def handle_get_last_scan(
        self,
        request: GetLastScan.Request,
        response: GetLastScan.Response,
    ) -> GetLastScan.Response:
        del request
        summary, expected_count, expected_count_met = self._orchestrator.get_last_scan_state()
        if summary is None:
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
        ) = summary_to_response_fields(summary)

        response.success = True
        response.message = f'Returned last scan with {count} object(s).'
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
        return response
