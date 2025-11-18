cmake_minimum_required(VERSION 3.0.2)
project(luna_bot_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
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
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    message_runtime
)

catkin_install_python(PROGRAMS
  nodes/avoider_large_rover.py
  nodes/battery_simulator.py
  nodes/camera_tf_publisher.py
  nodes/goal_based_avoider.py
  nodes/laser_scan_accumulator.py
  nodes/mapping_node.py
  nodes/mission_manager_node.py
  nodes/object_detection_node.py
  nodes/reset_nav.py
  nodes/semantic_annotator.py
  nodes/yolo_lunar_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/odom_to_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
