import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from isaac_manipulator_interfaces.action import EstimatePoseFoundationPose
from rclpy.action import ActionClient
from rclpy.node import Node

from isaac_manipulator_pose_server.config import WorkflowConfig
from isaac_manipulator_pose_server.continuous.models import Candidate, WorkerResult


@dataclass
class _WorkerState:
    worker_id: str
    namespace: str
    action_name: str
    client: ActionClient
    busy: bool = False
    dropped: bool = False
    failure_count: int = 0
    last_dispatch_monotonic_sec: float = 0.0
    dispatch_timeout_sec: float = 0.0
    active_object_id: int = -1
    active_frame_name: str = ''
    active_candidate: Optional[Candidate] = None
    active_refresh: bool = False
    goal_handle: Optional[object] = None
    goal_send_future: Optional[object] = None
    result_future: Optional[object] = None
    cooldown_until_monotonic_sec: float = 0.0

    def clear_active(self):
        self.busy = False
        self.active_object_id = -1
        self.active_frame_name = ''
        self.active_candidate = None
        self.active_refresh = False
        self.dispatch_timeout_sec = 0.0
        self.goal_handle = None
        self.goal_send_future = None
        self.result_future = None


@dataclass
class _TimedOutJob:
    worker_id: str
    object_id: int
    frame_name: str
    candidate: Optional[Candidate]
    was_tracking_refresh: bool
    goal_handle: Optional[object]


class FPWorkerPool:
    """Manages independent FoundationPose action workers."""

    def __init__(
        self,
        *,
        node: Node,
        config: WorkflowConfig,
        worker_namespaces: List[str],
        result_callback: Callable[[WorkerResult], None],
    ):
        self._node = node
        self._config = config
        self._lock = threading.Lock()
        self._result_callback = result_callback
        self._workers: Dict[str, _WorkerState] = {}

        for worker_index, raw_namespace in enumerate(worker_namespaces):
            namespace = self._normalize_namespace(raw_namespace)
            worker_id = f'fp_worker_{worker_index}'
            action_name = self._resolve_action_name(namespace, self._config.estimate_pose_action_name)
            self._workers[worker_id] = _WorkerState(
                worker_id=worker_id,
                namespace=namespace,
                action_name=action_name,
                client=ActionClient(self._node, EstimatePoseFoundationPose, action_name),
            )

    def available_worker_ids(self, now_sec: float) -> List[str]:
        with self._lock:
            available_ids = []
            for worker_id, state in self._workers.items():
                if state.busy:
                    continue
                if state.dropped:
                    # Soft recovery probe.
                    if state.client.wait_for_server(timeout_sec=0.0):
                        state.dropped = False
                        state.failure_count = 0
                    else:
                        continue
                if now_sec < state.cooldown_until_monotonic_sec:
                    continue
                available_ids.append(worker_id)
            return available_ids

    def busy_count(self) -> int:
        with self._lock:
            return sum(1 for state in self._workers.values() if state.busy)

    def dropped_worker_ids(self) -> List[str]:
        with self._lock:
            return sorted([
                state.worker_id for state in self._workers.values() if state.dropped
            ])

    def dispatch(
        self,
        *,
        worker_id: str,
        object_id: int,
        frame_name: str,
        candidate: Candidate,
        mesh_file_path: str,
        timeout_sec: float,
        tracking_refresh: bool,
    ) -> bool:
        with self._lock:
            state = self._workers.get(worker_id)
            if state is None or state.busy or state.dropped:
                return False

            if not state.client.wait_for_server(timeout_sec=0.0):
                state.failure_count += 1
                self._node.get_logger().warning(
                    f'[worker_pool] Action server unavailable for {worker_id} at '
                    f'"{state.action_name}" (failure_count={state.failure_count}).'
                )
                if state.failure_count >= 3:
                    state.dropped = True
                    self._node.get_logger().error(
                        f'[worker_pool] Marking {worker_id} dropped after repeated action '
                        f'server discovery failures for "{state.action_name}".'
                    )
                return False

            goal = EstimatePoseFoundationPose.Goal()
            goal.roi = candidate.detection
            goal.use_segmentation_mask = False
            goal.mesh_file_path = mesh_file_path
            goal.object_frame_name = frame_name

            state.busy = True
            state.last_dispatch_monotonic_sec = time.monotonic()
            state.dispatch_timeout_sec = max(0.1, timeout_sec)
            state.active_object_id = object_id
            state.active_frame_name = frame_name
            state.active_candidate = candidate
            state.active_refresh = tracking_refresh
            send_future = state.client.send_goal_async(goal)
            state.goal_send_future = send_future
            send_future.add_done_callback(
                lambda future, local_worker_id=worker_id: self._on_goal_response(
                    local_worker_id, future
                )
            )
            return True

    def cancel_all(self):
        with self._lock:
            workers = list(self._workers.values())
            now_sec = time.monotonic()
            for state in workers:
                state.cooldown_until_monotonic_sec = now_sec + self._config.dispatch_cooldown_sec
                self._cancel_worker_goal(state)
                state.clear_active()

    def cancel_worker(self, worker_id: str):
        with self._lock:
            state = self._workers.get(worker_id)
            if state is None:
                return
            state.cooldown_until_monotonic_sec = (
                time.monotonic() + self._config.dispatch_cooldown_sec
            )
            self._cancel_worker_goal(state)
            state.clear_active()

    def check_timeouts(self, now_sec: float):
        timed_out: List[_TimedOutJob] = []
        with self._lock:
            for state in self._workers.values():
                if not state.busy:
                    continue
                elapsed = now_sec - state.last_dispatch_monotonic_sec
                if elapsed <= state.dispatch_timeout_sec:
                    continue
                timed_out.append(
                    _TimedOutJob(
                        worker_id=state.worker_id,
                        object_id=state.active_object_id,
                        frame_name=state.active_frame_name,
                        candidate=state.active_candidate,
                        was_tracking_refresh=state.active_refresh,
                        goal_handle=state.goal_handle,
                    )
                )
                state.failure_count += 1
                if state.failure_count >= 3:
                    state.dropped = True
                state.cooldown_until_monotonic_sec = now_sec + self._config.dispatch_cooldown_sec
                state.clear_active()

        for job in timed_out:
            self._cancel_goal_handle(job.goal_handle)
            if job.candidate is None:
                continue
            self._result_callback(
                WorkerResult(
                    worker_id=job.worker_id,
                    success=False,
                    object_id=job.object_id,
                    frame_name=job.frame_name,
                    candidate=job.candidate,
                    error=f'Worker {job.worker_id} timed out.',
                    timed_out=True,
                    was_tracking_refresh=job.was_tracking_refresh,
                )
            )

    def _on_goal_response(self, worker_id: str, future):
        with self._lock:
            state = self._workers.get(worker_id)
            if state is None:
                return
        try:
            goal_handle = future.result()
        except Exception as exception:  # noqa: BLE001
            self._finalize_failure(worker_id, f'Goal send failed: {exception}')
            return

        if goal_handle is None or not goal_handle.accepted:
            self._finalize_failure(worker_id, 'Goal rejected by FoundationPose action server.')
            return

        with self._lock:
            state = self._workers.get(worker_id)
            if state is None:
                return
            state.goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            state.result_future = result_future
            result_future.add_done_callback(
                lambda result_done_future, local_worker_id=worker_id: self._on_result(
                    local_worker_id, result_done_future
                )
            )

    def _on_result(self, worker_id: str, future):
        with self._lock:
            state = self._workers.get(worker_id)
            if state is None:
                return
            object_id = state.active_object_id
            frame_name = state.active_frame_name
            candidate = state.active_candidate
            was_refresh = state.active_refresh

        if candidate is None:
            self._finalize_failure(worker_id, 'Worker result missing candidate context.')
            return

        try:
            wrapped_result = future.result()
        except Exception as exception:  # noqa: BLE001
            self._finalize_failure(worker_id, f'Result future failed: {exception}')
            return

        if wrapped_result is None:
            self._finalize_failure(worker_id, 'FoundationPose result was empty.')
            return

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self._finalize_failure(
                worker_id,
                f'FoundationPose action failed with status={wrapped_result.status}.'
            )
            return

        try:
            pose = self._extract_pose(wrapped_result.result)
        except RuntimeError as exception:
            self._finalize_failure(worker_id, str(exception))
            return

        with self._lock:
            state = self._workers.get(worker_id)
            if state is None:
                return
            state.failure_count = 0
            state.dropped = False
            state.cooldown_until_monotonic_sec = (
                time.monotonic() + self._config.dispatch_cooldown_sec
            )
            state.clear_active()

        self._result_callback(
            WorkerResult(
                worker_id=worker_id,
                success=True,
                object_id=object_id,
                frame_name=frame_name,
                candidate=candidate,
                pose=pose,
                was_tracking_refresh=was_refresh,
            )
        )

    def _finalize_failure(self, worker_id: str, error: str):
        with self._lock:
            state = self._workers.get(worker_id)
            if state is None:
                return
            candidate = state.active_candidate
            object_id = state.active_object_id
            frame_name = state.active_frame_name
            was_refresh = state.active_refresh
            state.failure_count += 1
            if state.failure_count >= 3:
                state.dropped = True
            state.cooldown_until_monotonic_sec = (
                time.monotonic() + self._config.dispatch_cooldown_sec
            )
            state.clear_active()

        if candidate is None:
            return
        self._result_callback(
            WorkerResult(
                worker_id=worker_id,
                success=False,
                object_id=object_id,
                frame_name=frame_name,
                candidate=candidate,
                error=error,
                was_tracking_refresh=was_refresh,
            )
        )

    def _extract_pose(self, action_result) -> Pose:
        detections = list(action_result.poses.detections)
        if not detections:
            raise RuntimeError('FoundationPose returned empty Detection3DArray.')
        results = list(detections[0].results)
        if not results:
            raise RuntimeError('FoundationPose returned Detection3D without results.')
        return results[0].pose.pose

    def _cancel_worker_goal(self, state: _WorkerState):
        self._cancel_goal_handle(state.goal_handle)

    def _cancel_goal_handle(self, goal_handle: Optional[object]):
        if goal_handle is None:
            return
        try:
            goal_handle.cancel_goal_async()
        except Exception:  # noqa: BLE001
            pass

    def _normalize_namespace(self, namespace: str) -> str:
        stripped = str(namespace).strip().strip('/')
        if not stripped:
            return ''
        return f'/{stripped}'

    def _resolve_action_name(self, namespace: str, action_name: str) -> str:
        action_base = str(action_name).strip().strip('/')
        if not action_base:
            action_base = 'estimate_pose_foundation_pose'
        if namespace:
            return f'{namespace}/{action_base}'
        return f'/{action_base}'
