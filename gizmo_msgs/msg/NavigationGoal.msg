# This is a message covering all types of navigation
byte GO_TO_WAYPOINT=0
byte DRIVE_TO_POINT_IN_IMAGE=1
byte FOLLOW_ME=2

# Type of navigation
byte nav_type
# Goal of navigation
geometry_msgs/Pose pose
# Waypoint UUID for GO_TO_WAYPOINT
string waypoint_uuid
# Head pose for GO_TO_WAYPOINT
gizmo_msgs/HeadPose head_pose
# Request ID (if from client)
string request_id
