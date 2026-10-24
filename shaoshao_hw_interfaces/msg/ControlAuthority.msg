# Hardware control permission only. No wheel speed or gimbal angle is required.
# A real DR16/mode adapter must publish this; UART1 V2 ACK cannot infer it.
std_msgs/Header header
uint64 seq
uint64 authority_epoch
bool control_allowed
