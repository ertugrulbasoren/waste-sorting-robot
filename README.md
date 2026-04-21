# Waste Sorting Robot (ROS Noetic + Gazebo + UR5)

This project simulates a waste sorting robotic system using:

- ROS Noetic
- Gazebo Classic
- UR5 robotic arm
- MoveIt motion planning
- Vision-based detection pipeline (planned)

---

## 📦 Requirements

Ubuntu 20.04 + ROS Noetic

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-moveit \
  ros-noetic-gazebo-ros \
  ros-noetic-gazebo-ros-control \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  python3-rosdep
