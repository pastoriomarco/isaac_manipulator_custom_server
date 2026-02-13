# Pipeline mode constants.
uint8 IDLE=1
uint8 RUNNING=2
uint8 STOPPED=3
uint8 ERROR=4

# Status snapshot header.
std_msgs/Header header

# Current mode.
uint8 mode

# Requested tracked object target.
int32 target_count

# Runtime counters.
uint32 active_count
uint32 queued_candidates
uint32 available_workers
uint32 busy_workers

# Workers currently marked unhealthy.
string[] dropped_workers

# Last terminal or warning error string.
string last_error
