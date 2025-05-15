cmake_minimum_required(VERSION 3.0.2)
project(joint_state_reader)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)