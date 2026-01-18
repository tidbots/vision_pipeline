cmake_minimum_required(VERSION 3.0.2)
project(yolo26_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  image_transport
  vision_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/yolo26_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
