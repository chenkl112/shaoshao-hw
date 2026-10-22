# Independent of navigation translation. The adapter checks its own lease,
# sequence, command age, hardware authority and watchdog.
std_msgs/Header header
string lease
uint64 seq
float64 omega_radps
