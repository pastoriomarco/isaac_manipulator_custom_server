# Pipeline command constants.
uint8 START=1
uint8 STOP=2
uint8 RESET=3
uint8 SET_TARGET=4

# Command to execute.
uint8 command

# Target tracked object count. 0 means "as many as available".
int32 target_count

# Optional object profile key from pose_server.available_objects.
string object_key

# Optionally clear object cache before start.
bool clear_before_start

# Optionally clear object cache after stop.
bool clear_after_stop
