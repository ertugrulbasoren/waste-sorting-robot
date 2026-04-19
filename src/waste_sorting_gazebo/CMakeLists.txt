cmake_minimum_required(VERSION 3.0.2)
project(waste_sorting_gazebo)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  gazebo_ros
  gazebo_msgs
  cv_bridge
  message_generation
)

find_package(gazebo REQUIRED)

add_message_files(
  FILES
  Detection2D.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    gazebo_ros
    gazebo_msgs
    cv_bridge
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(conveyor_motion_plugin SHARED
  plugins/conveyor_motion_plugin.cpp
)

add_dependencies(conveyor_motion_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(conveyor_motion_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/spawn_trash.py
  scripts/spawn_sequence.py
  scripts/sim_detector.py
  scripts/yolo_detector.py
  scripts/tracker_node.py
  scripts/sort_executor.py
  scripts/spawn_simple_arm.py
  scripts/arm_pick_place.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch models worlds msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
