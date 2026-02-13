# Tracked object status constants.
uint8 CANDIDATE=1
uint8 TRACKING=2
uint8 LOST=3
uint8 USED=4

# Stable orchestrator-assigned object id.
int32 object_id

# Detector class metadata.
string class_id
float32 class_score

# Frame name used for FoundationPose output.
string frame_name

# Current estimated pose.
geometry_msgs/Pose pose

# Latest 2D ROI.
vision_msgs/BoundingBox2D bbox

# Current lifecycle status.
uint8 status

# Owning worker identifier (for example fp_worker_0).
string worker_id

# Lifecycle timestamps.
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_update
