byte GO_TO_WAYPOINT=0
byte DRIVE_TO_POINT_IN_IMAGE=1
byte FOLLOW_ME=2

byte NAV_SUCCESS=0
byte NAV_PLANNING=1
byte NAV_MOVING=2
byte NAV_ERROR=3
byte NAV_CANCELED=4

# NAV_ status
byte status
# Type of navigation
byte nav_type
# Request ID if from client
string request_id
# error string if NAV_ERROR
string error
# waypoint UUID if GO_TO_WAYPOINT
string waypoint_uuid
# Goal pose
geometry_msgs/Pose goal
