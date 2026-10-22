uint64 id
# Offsets from each predicted center, in header.frame_id axes. Convex hull.
geometry_msgs/Polygon footprint
# Index 0 is at the parent header.stamp, index k is at k * prediction_dt.
geometry_msgs/Point[] centers
bool confirmed
