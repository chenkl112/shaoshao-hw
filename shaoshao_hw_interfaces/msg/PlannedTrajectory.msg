# Immutable HW MINCO geometry and arc-length speed profile. The embedded path
# header identifies the exact Nav2 plan; stale snapshots cannot arm control.
nav_msgs/Path path
float64[] durations
# Segment-major, power-major, then x/y (12 coefficients per quintic segment).
float64[] coefficients
float64[] profile_s
float64[] profile_time
float64[] profile_speed
