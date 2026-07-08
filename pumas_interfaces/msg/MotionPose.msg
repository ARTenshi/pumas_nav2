# Atomic motion goal published by motion_synth: a full Joints snapshot + the
# time_from_start to use when the hardware controllers forward the trajectory.
# arm_controller picks up the lift/arm fields and head_controller picks up the
# pan/tilt fields; motion_execution_time applies to both.

pumas_interfaces/Joints joints
# time_from_start (sec). 0.0 → controller default.
float32 motion_execution_time
