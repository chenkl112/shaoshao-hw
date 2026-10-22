# Samples are measurements at header.stamp, synchronized into the ROS clock.
# frame_id: navcar_center; axes follow the lidar, origin is the yaw axis.
# Never publish UART V1 target echoes as this message.
std_msgs/Header header
uint64 seq
# Changes on every hardware/manual takeover, including disable -> enable.
uint64 authority_epoch
bool control_allowed
float64 vx_mps
float64 vy_mps
# Actual CHASSIS angular velocity, not lidar angular velocity.
float64 chassis_omega_radps
# Lidar/gimbal heading minus chassis heading, positive counterclockwise.
float64 gimbal_relative_yaw_rad
