# Independent of navigation translation. The adapter checks its own lease,
# sequence, command age and watchdog. The navigation gateway must revoke the
# lease on verified hardware takeover; UART1 V2 itself does not report DR16 mode.
std_msgs/Header header
string lease
uint64 seq
float64 omega_radps
