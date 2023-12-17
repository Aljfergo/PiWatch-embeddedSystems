cmake_minimum_required(VERSION 3.0.2)
project(my_robot_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(obstacle_avoidance_node src/scripts/obstacle_avoidance_node.py)
target_link_libraries(obstacle_avoidance_node ${catkin_LIBRARIES})