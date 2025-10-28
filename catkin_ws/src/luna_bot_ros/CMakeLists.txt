cmake_minimum_required(VERSION 3.0.2)
project(luna_bot_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  Hazard.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs sensor_msgs message_runtime
)

catkin_install_python(PROGRAMS
  nodes/navigation_node.py
  nodes/sensor_node.py
  nodes/mapping_node.py
  nodes/object_detection_node.py
  nodes/unity_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
