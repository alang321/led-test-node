cmake_minimum_required(VERSION 3.0.2)
project(led_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslaunch
  led_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs roslaunch led_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include
)


add_executable(led_test_node src/led_test_node.cpp)
target_link_libraries(led_test_node ${catkin_LIBRARIES})


add_dependencies(led_test_node ${catkin_EXPORTED_TARGETS})
