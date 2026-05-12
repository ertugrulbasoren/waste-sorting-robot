# Multiple 2D detections from a single image frame.
# This message is used by the YOLO detector to publish all detections in one frame.

std_msgs/Header header
Detection2D[] detections
