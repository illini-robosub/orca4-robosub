# Header for timestamp and frame info
std_msgs/Header header

# Class name and confidence
string class_id
float32 confidence

# 2D bounding box in image space
float32 bbox_x        # center x in pixels
float32 bbox_y        # center y in pixels
float32 bbox_width
float32 bbox_height

# 3D position relative to camera
float32 depth         # meters, -1.0 if unavailable
float32 bearing_x     # horizontal angle to object in radians
float32 bearing_y     # vertical angle to object in radians

# Whether depth is valid
bool depth_valid

