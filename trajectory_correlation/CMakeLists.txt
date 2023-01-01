cmake_minimum_required(VERSION 3.0.2)
project(trajectory_correlation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(trajectory_correlation src/TrajectoryCorrelationNode.cpp)
target_include_directories(trajectory_correlation PUBLIC include/*.h)
target_link_libraries(trajectory_correlation ${catkin_LIBRARIES})

add_executable(fake_vo_node src/FakeVo.cpp)
target_link_libraries(fake_vo_node ${catkin_LIBRARIES})

add_executable(fake_gps_node src/FakeGps.cpp)
target_link_libraries(fake_gps_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/TrajectoryCorrelationNode.py 
                     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})