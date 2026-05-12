cmake_minimum_required(VERSION 3.0.2)
project(waste_sorting_gazebo)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  gazebo_msgs
  message_generation
  cv_bridge
  image_transport
)

find_package(gazebo REQUIRED)

################################################
## ROS messages
################################################

add_message_files(
  FILES
  Detection2D.msg
  Detection2DArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Catkin package
################################################

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    gazebo_msgs
    message_runtime
    cv_bridge
    image_transport
  INCLUDE_DIRS
  LIBRARIES conveyor_motion_plugin
)

################################################
## Include directories
################################################

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

################################################
## Gazebo conveyor plugin
################################################

add_library(conveyor_motion_plugin
  plugins/conveyor_motion_plugin.cpp
)

add_dependencies(conveyor_motion_plugin
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(conveyor_motion_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

################################################
## Python scripts
################################################

catkin_install_python(
  PROGRAMS
    scripts/yolo_detector.py
    scripts/tracker_node.py
    scripts/sort_executor.py
    scripts/dataset_camera_feeder.py
    scripts/spawn_trash.py
    scripts/spawn_sequence.py
    scripts/arm_pick_place.py
    scripts/sim_detector.py
    scripts/trial_logger.py
    scripts/analyze_trial_results.py
    scripts/generate_realistic_waste_meshes.py
    scripts/prepare_realistic_assets.py
    scripts/spawn_realistic_waste.py
    scripts/run_realistic_asset_trials.py
    scripts/realistic_conveyor_feeder.py
    scripts/pick_event_model_resolver.py
    scripts/realistic_yolo_kpi_logger.py
    scripts/realistic_synthetic_dataset_collector.py
    scripts/analyze_realistic_yolo_kpi.py
    scripts/spawn_fake_robot_arm_visual.py
    scripts/fake_robot_arm_executor.py
    scripts/robot_arm_result_logger.py
    scripts/analyze_robot_arm_results.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

################################################
## Install folders
################################################

install(
  DIRECTORY launch worlds models models_yolo
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
