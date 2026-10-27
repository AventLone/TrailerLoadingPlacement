uint8 STATUS_IDLE=0
uint8 STATUS_PROCESSING=1
uint8 STATUS_SUCCEEDED=2
uint8 STATUS_FAILED=3

# Input cloud acquisition time and the coordinate frame for poses and positions.
std_msgs/Header header
uint8 status_code 0

# True if and only if status_code is STATUS_SUCCEEDED.
bool success false

# Geometry is valid only when success is true.
# Length (x), width (y), and height (z) along trailer-local axes, in meters.
geometry_msgs/Vector3 trailer_dimensions

# Trailer bounding-box center and orientation in header.frame_id.
geometry_msgs/Pose trailer_pose

# Center of the selected slot footprint on the trailer floor, in header.frame_id.
geometry_msgs/Point slot_position
