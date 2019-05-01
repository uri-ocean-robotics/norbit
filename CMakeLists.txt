cmake_minimum_required(VERSION 2.8.3)
project(norbit)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES norbit
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

file(GLOB SRCS
    "include/norbit/*.h"
    "include/norbit/*.cpp"
    "include/norbit/*.hpp"
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(norbit_node nodes/norbit_node ${SRCS} ${UI_HDRS} )
target_link_libraries (norbit_node ${catkin_LIBRARIES}  )

