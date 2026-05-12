# Single 2D object detection produced by the vision pipeline.
# Coordinate system:
# - x_min, y_min: top-left corner of bounding box in image pixels
# - x_max, y_max: bottom-right corner of bounding box in image pixels
# - center_x, center_y: bounding box center in image pixels

std_msgs/Header header

string class_name
float32 confidence

int32 x_min
int32 y_min
int32 x_max
int32 y_max

int32 center_x
int32 center_y
