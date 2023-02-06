cmake_minimum_required(VERSION 3.0.2)
project(bspline_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs)

include(ExternalProject)

# tinyspline for B-Spline
set(TINYSPLINE_INSTALL_DIR ${PROJECT_SOURCE_DIR}/tinyspline)
ExternalProject_Add(tinyspline
  GIT_REPOSITORY https://github.com/msteinbeck/tinyspline.git
  GIT_TAG d536e842f71f997ddb0f11d1d7a1a0d8d3e557d0
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${TINYSPLINE_INSTALL_DIR}
  TIMEOUT 30
  PATCH_COMMAND
    COMMAND sed -i -e "s/VERSION\ 3.4/VERSION\ 3.0/g" CMakeLists.txt
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
  BUILD_COMMAND make
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND make install
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  )
include_directories(${TINYSPLINE_INSTALL_DIR}/include)
set(TINYSPLINE_LIB ${TINYSPLINE_INSTALL_DIR}/lib64/libtinysplinecxx.so) # link_directories is strongly discouraged in catkin_lint

MESSAGE(WARNING "${TINYSPLINE_LIB}")

if (NOT EXISTS ${TINYSPLINE_INSTALL_DIR})
  #This has be done in the very first build for
  # catkin_package to be able to find the include folder
  file(MAKE_DIRECTORY ${TINYSPLINE_INSTALL_DIR}/include)
endif()

catkin_package(
  INCLUDE_DIRS include ${TINYSPLINE_INSTALL_DIR}/include
  LIBRARIES bspline_ros ${TINYSPLINE_LIB}
  CATKIN_DEPENDS roscpp visualization_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(bspline_ros src/bspline_ros.cpp)
add_dependencies(bspline_ros tinyspline)
target_link_libraries(bspline_ros ${catkin_LIBRARIES} ${TINYSPLINE_LIB})

install(DIRECTORY include/${PROJECT_NAME} ${TINYSPLINE_INSTALL_DIR}/include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS bspline_ros ${TINYSPLINE_LIB}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
