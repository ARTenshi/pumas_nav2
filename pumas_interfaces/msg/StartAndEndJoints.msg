std_msgs/Header header
bool has_arm_start_pose
pumas_interfaces/Joints start_pose
bool has_arm_end_pose
pumas_interfaces/Joints end_pose
# time_from_start (sec) for arm/head trajectory commands during this motion.
# Leave 0.0 to use the motion_synth_server default (0.5 s).
float32 motion_execution_time

