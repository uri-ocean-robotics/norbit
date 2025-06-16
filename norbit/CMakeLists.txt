cmake_minimum_required(VERSION 3.5)

project(norbit)

####################################################################
## setup compiler
####################################################################

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

####################################################################
## setup packages
####################################################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(norbit_msgs REQUIRED)

# find_package(acoustic_msgs REQUIRED)

####################################################################
## setup dependencies
####################################################################

set(library_name norbit_lib)
set(executable_name norbit_mbes_node)
set(dependencies
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  pcl_ros
  pcl_conversions
  norbit_msgs
)

####################################################################
## setup compiling
####################################################################

include_directories(
  include
)

# Library
add_library(${library_name} SHARED
  src/ros_wrapper.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)
target_link_libraries( 
  ${library_name}
)

# Executable
add_executable(${executable_name}
  nodes/norbit_node.cpp
)
ament_target_dependencies(${executable_name}
  ${dependencies}
)
target_link_libraries(${executable_name} 
  ${library_name}
)

####################################################################
## setup install 
####################################################################

# Install
install(TARGETS ${library_name} ${executable_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
