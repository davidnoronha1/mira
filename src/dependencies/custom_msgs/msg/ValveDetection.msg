std_msgs/Header header

string  class_name    # detected class label (e.g. "valve")
float32 confidence    # YOLO detection confidence [0, 1]

# 3-D centroid of the fitted AABB in camera frame [m]
# ZED RIGHT_HANDED_Y_UP: X=right, Y=up, Z=toward camera (negative = in front)
float32 x
float32 y
float32 z

# Axis-aligned bounding box dimensions [m]
float32 length        # extent along X
float32 breadth       # extent along Y
float32 height        # extent along Z (depth)
