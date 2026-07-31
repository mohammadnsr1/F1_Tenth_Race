cmake_minimum_required(VERSION 3.0.2)

set (CMAKE_CXX_STANDARD 17)

project(car)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_runtime geometry_msgs sensor_msgs OpenCV cv_bridge image_transport message_generation)

catkin_python_setup()


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp rospy message_runtime
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/car/pid.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

