cmake_minimum_required(VERSION 3.0.2)
project(test_model_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_model_node src/scripts/test_model.py)
target_link_libraries(test_model_node ${catkin_LIBRARIES})