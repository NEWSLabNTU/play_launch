# A minimal stamped payload.
#
# The header is the point: play_launch's interception layer correlates a
# publish with the take that receives it via header.stamp, which is what lets
# the exported Chrome trace draw chain arrows. std_msgs/String has no header,
# so a workload built on it produces a timeline with no end-to-end linking —
# that is exactly what the existing rt_demo fixture suffers from.
std_msgs/Header header
uint32 seq
string payload
