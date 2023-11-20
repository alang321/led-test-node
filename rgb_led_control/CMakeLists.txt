cmake_minimum_required(VERSION 3.0.2)
project(rgb_led_control)

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


add_executable(rgb_led_control_node src/led_test_node.cpp)
target_link_libraries(rgb_led_control_node ${catkin_LIBRARIES})


add_dependencies(rgb_led_control_node ${catkin_EXPORTED_TARGETS})
