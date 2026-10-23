# UART1 V2 wheel-speed-derived body motion. This is NOT ChassisFeedback.
# header.stamp is host reception time; firmware provides no sample timestamp.
# frame_id = navcar_chassis: body x forward, y left, z up.
std_msgs/Header header
# Host reception counter; NOT an MCU sampling sequence.
uint64 reception_seq
# Most recently accepted command, can repeat at 100 Hz. NOT a measurement ID.
uint16 last_command_seq
float64 forward_mps
float64 left_mps
float64 chassis_omega_radps
# IMU gimbal yaw relative to startup; NOT gimbal angle relative to chassis.
float64 gimbal_yaw_start_rad
bool velocity_valid
bool yaw_valid
