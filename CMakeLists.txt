cmake_minimum_required(VERSION 2.8.3)
project(rover_locomotion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  rostest
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  add_rostest(test/rover_locomotion_test.launch)
endif()
