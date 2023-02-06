cmake_minimum_required(VERSION 2.8.3)
project(aerial_motion_planning_msgs)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  bspline_ros
  geometry_msgs
  sensor_msgs
  moveit_msgs
  tf_conversions
  aerial_robot_model
  dragon
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multilink_state continuous_path_generator
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs moveit_msgs
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

Add_library (multilink_state src/multilink_state.cpp )
target_link_libraries (multilink_state ${catkin_LIBRARIES})
add_dependencies(multilink_state ${catkin_EXPORTED_TARGETS})

add_library(continuous_path_generator
  src/continuous_path_generator.cpp)
target_link_libraries(continuous_path_generator ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


install(TARGETS multilink_state continuous_path_generator
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
