# Waste Sorting Robot - Run Instructions

This project is an autonomous waste sorting robot simulation built with ROS Noetic, Gazebo Classic, YOLOv8, OpenCV, and Python.

The system detects waste objects from a dataset-driven camera stream, tracks them, generates pick events, and moves the corresponding Gazebo object to the correct recycling bin.

Supported waste classes:

- glass
- metal
- paper
- plastic

---

## 1. Tested Environment

This project was tested with:

- Ubuntu 20.04.6 LTS
- ROS Noetic
- Gazebo Classic 11
- Python 3
- OpenCV
- cv_bridge
- Ultralytics YOLOv8

---

## 2. Repository Structure

Important folders:

```text
waste-sorting-robot-project/
├── datasets/
│   └── waste_4class/
│       └── waste_dataset/
│           ├── data.yaml
│           ├── train/
│           │   ├── images/
│           │   └── labels/
│           └── valid/
│               ├── images/
│               └── labels/
├── src/
│   └── waste_sorting_gazebo/
│       ├── launch/
│       ├── msg/
│       ├── scripts/
│       ├── models_yolo/
│       ├── worlds/
│       └── plugins/
├── build/
├── devel/
└── RUN_INSTRUCTIONS.md
