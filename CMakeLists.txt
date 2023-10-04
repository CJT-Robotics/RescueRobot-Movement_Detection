cmake_minimum_required(VERSION 3.0.2)
project(CJT-RoboCupGermany-MovementDetection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  joy_relay
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(movement_detection src/movement_detection.cpp)
target_link_libraries(movement_detection ${catkin_LIBRARIES} ${OpenCV_LIBS})