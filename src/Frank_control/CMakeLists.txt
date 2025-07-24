cmake_minimum_required(VERSION 3.0.2)
project(Frank_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  GetHandAngles.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

catkin_install_python(PROGRAMS
  src/visual_hand_angle_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 