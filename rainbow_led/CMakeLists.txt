cmake_minimum_required(VERSION 3.0.2)
project(rainbow_led)

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


add_executable(rainbow_led_node src/rainbow_led_node.cpp)
target_link_libraries(rainbow_led_node ${catkin_LIBRARIES})


add_dependencies(rainbow_led_node ${catkin_EXPORTED_TARGETS})
