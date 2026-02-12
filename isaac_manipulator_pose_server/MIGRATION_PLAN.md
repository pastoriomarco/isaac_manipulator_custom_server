# Migration Plan: Continuous Multi-Worker Pose Pipeline

## 1) Why migrate

Current direct scan flow is request-driven and bursty:

- A service call triggers detection, then serial FoundationPose calls.
- FoundationPose latency is bimodal: fast when tracking seed is good, slow (up to ~10s+) on cold start.
- Aggressive cancel/retry under load can destabilize the process.
- One slow/failed pose attempt can block progress for other candidates.

Goal: move to a continuous, event-driven architecture that is reliable for multi-object scenes and scales by machine capacity.

## 2) Target outcomes

1. Configurable number of FoundationPose workers (`fp_worker_count`), not fixed to 1 or 2.
2. Topic-triggered pipeline control (start/stop/reset, requested object count).
3. Continuous run mode after trigger, trying to maintain requested number of tracked poses.
4. Continuous publication (`2-4 Hz`) of current tracked objects (poses + IDs + state).
5. Client can mark an object as used via service; pipeline frees that slot and reacquires a new object.
6. Fault isolation: one FP worker crash should not kill the full pipeline.

## 3) Scope and non-goals

In scope:

- New orchestration layer for continuous operation.
- Multi-worker scheduling and dispatch.
- Object lifecycle state management (`candidate -> tracking -> used/lost`).
- Interface additions (topic(s), service(s), status outputs).

Out of scope (initial migration):

- Replacing NVIDIA FoundationPose internals.
- Rewriting detector models/pipelines.
- Full object re-identification across major scene changes (only practical bbox/IoU tracking initially).

## 4) High-level architecture

### 4.1 Core components

1. **Pipeline Orchestrator Node** (`multi_object_pose_orchestrator`)
   - Owns object registry/state machine.
   - Subscribes detector output continuously.
   - Assigns work to FP workers.
   - Publishes current tracked objects at fixed rate.
   - Handles trigger/control topic and mark-used service.

2. **Detector Cache**
   - Stores latest detection snapshots with timestamps.
   - Performs class filtering + NMS + de-dup.
   - Provides candidate queue to orchestrator.

3. **FP Worker Pool (N workers)**
   - Each worker has dedicated action endpoint.
   - Worker processes one target at a time (estimate + tracking refresh loop).
   - Reports success/failure/health back to orchestrator.

4. **Object Registry**
   - Global map of active logical objects with stable IDs.
   - Tracks per-object status, bbox, pose, confidence, timestamps, assigned worker.

### 4.2 Process model recommendation

Recommended: **one ROS component container process per FP worker group**, not one shared process for all FP workers.

- Use multiple `component_container_mt` processes inside same Docker/runtime.
- Reason: crash containment for native memory issues (`malloc corruption` style failures).
- Tradeoff: more launch complexity, slightly higher overhead.

Fallback option:

- Multiple FP workers in one process is simpler but has poor fault isolation.

## 5) Adopted design choices

### Continuous detector topic subscription + cached snapshots

- Detector runs continuously at low/moderate rate (`2-4 Hz` target).
- Orchestrator consumes the freshest cached snapshot instead of blocking each cycle waiting for new camera messages.
- Detect action calls remain fallback/recovery only, not the primary detection path.

### Parallel FP with configurable worker pool

- Use a configurable `fp_worker_count` to scale by machine capacity.
- Multiple FP workers run in parallel and independently process assigned candidates.
- Worker count can be tuned per deployment to balance throughput and GPU contention.

### Control topic trigger interface

- Pipeline lifecycle is controlled by a dedicated topic command interface (`START`, `STOP`, `RESET`, `SET_TARGET`).
- Once started, the orchestrator stays in continuous mode and maintains the requested active object count.
- Legacy service triggers remain only for compatibility, not as the main control path.

### Service-based "object used" signaling

- Clients call `mark_object_used` with an object identifier to retire consumed objects.
- The orchestrator acknowledges deterministically and frees the assigned worker immediately.
- Freed capacity is reused to acquire and track a new candidate object.

## 6) Proposed external interfaces

## 6.1 Control topic (new)

Topic: `/isaac_manipulator_pose_server/pipeline_command`  
Type (recommended): `isaac_manipulator_server_interfaces/msg/PipelineCommand`

Suggested fields:

- `string command` (`START`, `STOP`, `RESET`, `SET_TARGET`)
- `int32 target_count` (`0` means “as many as available”)
- `string object_key` (optional class/mesh profile selector)
- `bool clear_before_start`
- `bool clear_after_stop`

Minimal alternative: `std_msgs/String` + JSON payload (faster to prototype, weaker typing).

## 6.2 Object-used service (new)

Service: `/isaac_manipulator_pose_server/mark_object_used`  
Type: `isaac_manipulator_server_interfaces/srv/MarkObjectUsed`

Request:

- `int32 object_id`
- `string frame_name` (optional cross-check)
- `string reason` (optional; e.g. `picked`)

Response:

- `bool success`
- `string message`
- `bool object_was_active`

Behavior:

- Mark object as `USED`.
- Unassign its worker.
- Trigger immediate candidate reacquire for freed worker.

## 6.3 Continuous output topic (new)

Topic: `/isaac_manipulator_pose_server/tracked_objects`  
Type: `isaac_manipulator_server_interfaces/msg/TrackedObjectArray`

Publish rate:

- Parameterized `tracked_objects_publish_hz` (default `3.0`, allowed `2.0-4.0` as requested).

Per object (`TrackedObject`):

- `int32 object_id` (stable logical id)
- `string class_id`
- `float32 class_score`
- `string frame_name`
- `geometry_msgs/Pose pose`
- `vision_msgs/BoundingBox2D bbox`
- `uint8 status` (`TRACKING`, `CANDIDATE`, `LOST`, `USED`)
- `string worker_id`
- `builtin_interfaces/Time last_update`

## 6.4 Optional health topic (new)

Topic: `/isaac_manipulator_pose_server/pipeline_status`  
Type: `isaac_manipulator_server_interfaces/msg/PipelineStatus`

Includes:

- mode (`IDLE`, `RUNNING`, `STOPPED`, `ERROR`)
- target_count
- active_count
- queued_candidates
- available_workers / busy_workers
- dropped_workers
- last_error

## 7) Internal orchestration behavior

## 7.1 State machine

Pipeline states:

- `IDLE`: not consuming/dispatching work.
- `RUNNING`: continuously detecting, assigning, tracking.
- `PAUSED`: optional future state.
- `ERROR`: unrecoverable control-path failure.

Object states:

- `NEW_CANDIDATE`
- `ASSIGNED`
- `TRACKING`
- `LOST`
- `USED`
- `RETIRED`

## 7.2 Scheduling loop

1. Detector callback updates candidate snapshot cache.
2. Orchestrator tick (e.g. `10 Hz` internal):
   - Refresh active tracking objects.
   - Reconcile candidates vs active objects (IoU/center-distance gating).
   - Assign free workers to best unassigned candidates.
   - Respect `target_count` ceiling unless `target_count=0`.
3. Publisher timer (`2-4 Hz`) emits current `TrackedObjectArray`.

## 7.3 Tracking strategy

- Worker with solved pose remains bound to object until:
  - object marked used, or
  - tracking invalidated (stale updates, low confidence, bbox mismatch), or
  - explicit reset.
- Reacquire interval can force periodic detector-based correction.

## 7.4 Timeout and retry policy

Recommended defaults:

- `first_pose_timeout_sec`: `18-25` (cold solve)
- `tracking_update_timeout_sec`: `4-8`
- `estimate_pose_retry_count`: `0-1` (avoid cancel storms)
- `dispatch_cooldown_sec`: `0.2-0.5` between worker reassignments

## 8) Configuration changes (planned)

Add to `pose_server.yaml`:

- `continuous_mode_enabled` (bool)
- `fp_worker_count` (int, >=1)
- `fp_worker_namespaces` (optional list; auto-generated if empty)
- `tracked_objects_publish_hz` (float, default 3.0)
- `detector_processing_hz` (float, default 3.0)
- `target_count_default` (int)
- `first_pose_timeout_sec`
- `tracking_update_timeout_sec`
- `object_lost_ttl_sec`
- `dispatch_cooldown_sec`
- `max_candidates_per_cycle`

## 9) Launch architecture migration

Current:

- One manipulator container with detector server + foundation pose server + FP node path.

Target:

- Keep detector path shared.
- Spawn `N` FP worker server stacks with unique namespaces:
  - `/fp_worker_0/...`
  - `/fp_worker_1/...`
  - ...
- Orchestrator dispatches to `/fp_worker_i/estimate_pose_foundation_pose`.

Implementation detail:

- Use launch loop to instantiate composable groups by `fp_worker_count`.
- Prefer one `component_container_mt` process per worker group for isolation.

## 10) Backward compatibility

Keep existing APIs during migration:

- `/scan_bin_objects` and `/get_last_scan` remain supported.
- In continuous mode, `scan_bin_objects` can map to:
  - set target + start, wait until count/timeout, then return snapshot.

Deprecation path:

- mark legacy one-shot scan mode as supported but non-default after stabilization.

## 11) Phased implementation plan

### Phase 0: Instrument and baseline

- Add precise metrics (detection latency, FP start latency, FP result latency, success ratio).
- Capture baseline on current setup for 1/3/5 object targets.

Deliverable:

- Benchmark table in README with reproducible launch/config.

### Phase 1: Interface additions

- Add new messages/services:
  - `PipelineCommand.msg`
  - `TrackedObject.msg`
  - `TrackedObjectArray.msg`
  - `PipelineStatus.msg` (optional first pass)
  - `MarkObjectUsed.srv`
- Keep compile and package exports clean.

Deliverable:

- Interface package builds and docs with `ros2 interface show` examples.

### Phase 2: Continuous orchestrator skeleton

- New orchestrator node with:
  - command topic subscription
  - object-used service
  - periodic tracked-object publisher
  - basic candidate registry

Deliverable:

- Start/stop/reset pipeline without multi-worker dispatch yet.

### Phase 3: Detection cache integration

- Continuous detector topic consumption.
- Candidate de-dup + class filter + memory.
- Remove detector blocking behavior from per-request path.

Deliverable:

- No detector starvation timeouts in normal operation.

### Phase 4: Configurable FP worker pool

- Launch `fp_worker_count` workers.
- Worker allocator + queue dispatch.
- Isolation strategy (preferred: one process per worker).

Deliverable:

- Parallel pose throughput improvement for multi-object scenes.

### Phase 5: Persistent tracking + used-object lifecycle

- Keep successful workers tracking object until invalid/used/reset.
- Implement `mark_object_used` flow and immediate replacement search.

Deliverable:

- Stable active set maintained near target count over time.

### Phase 6: Hardening and fallback

- Worker health watchdog/restart policy.
- Guardrails against cancel/retry storms.
- Tune defaults for slow-cold-start FP models.

Deliverable:

- Long-run stress test passes without container-wide crash.

## 12) Acceptance criteria

Functional:

1. `fp_worker_count` configurable at launch/config.
2. Topic trigger can start/stop/reset and set target count.
3. Continuous publication at configured `2-4 Hz`.
4. `mark_object_used` removes object and reacquires replacement.
5. Backward compatibility with `scan_bin_objects`.

Performance/reliability:

1. For static scene of 5 same-class objects:
   - eventually tracks >= target count with no manual re-trigger loop.
2. Detect path should not repeatedly fail on topic starvation in steady state.
3. Single worker crash does not terminate all workers/orchestrator process.
4. No repeated cancel-retry loops dominating runtime under normal load.

## 13) Risks and mitigations

Risk: GPU contention with too many workers.

- Mitigation: cap `fp_worker_count` by profiling; expose config guidance.

Risk: Duplicate tracking of same physical object.

- Mitigation: IoU + center-distance + temporal gating + per-class suppression.

Risk: Native instability under heavy cancel/retry.

- Mitigation: reduce retries, adaptive timeout policy, worker cooldown, process isolation.

Risk: Interface churn for clients.

- Mitigation: keep old services and provide migration examples.

## 14) Expected outputs after migration

At runtime, clients should rely on:

1. `pipeline_command` topic to control run mode.
2. `tracked_objects` topic for continuously updated object poses and IDs.
3. `mark_object_used` service to free an object slot and trigger replacement.
4. `pipeline_status` topic (optional but recommended) for observability.

Legacy one-shot service (`scan_bin_objects`) remains available as compatibility wrapper.
