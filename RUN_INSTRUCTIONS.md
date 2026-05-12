# Waste Sorting Robot - Run Instructions

## 1. Project Overview

This project is a ROS Noetic and Gazebo based intelligent waste sorting robot simulation.

The system detects realistic waste objects moving on a conveyor belt using a top camera and YOLO object detection. Detected objects are classified into four classes:

- glass
- metal
- paper
- plastic

The detected object is tracked, converted into a pick event, resolved to a Gazebo model name, and then sorted into the correct bin.

Final pipeline:

```text
Realistic 3D waste asset
    ↓
Conveyor motion
    ↓
Top camera image
    ↓
YOLO detector
    ↓
Tracker
    ↓
Pick event
    ↓
Pick event model resolver
    ↓
Sort executor
    ↓
KPI logger and analyzer
