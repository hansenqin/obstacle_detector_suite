cmake_minimum_required(VERSION 3.0.2)
project(obstacle_converter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs std_srvs geometry_msgs message_filters obstacle_detector message_generation)

add_message_files(FILES Zonotope.msg Zonotopes.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs geometry_msgs message_runtime
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/obstacle_converter_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(distance_calculator_node src/distance_calculator_node.cpp)

add_dependencies(distance_calculator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

target_link_libraries(distance_calculator_node
  ${catkin_LIBRARIES}
)

add_executable(synchronizer_node src/synchronizer_node.cpp)

add_dependencies(synchronizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

target_link_libraries(synchronizer_node
  ${catkin_LIBRARIES}
)


