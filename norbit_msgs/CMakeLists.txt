cmake_minimum_required(VERSION 3.8)

project(norbit_msgs)

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
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

####################################################################
## setup files                    
####################################################################

file(GLOB MSG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/**/*.msg)
file(GLOB SRV_FILES ${CMAKE_CURRENT_SOURCE_DIR}/**/*.srv)

set(INTERFACE_FILES "")

foreach(I IN ITEMS ${MSG_FILES})
  get_filename_component(ITEM_NAME ${I} NAME)
  list(APPEND INTERFACE_FILES msg/${ITEM_NAME})
endforeach()

foreach(I IN ITEMS ${SRV_FILES})
  get_filename_component(ITEM_NAME ${I} NAME)
  list(APPEND INTERFACE_FILES srv/${ITEM_NAME})
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${INTERFACE_FILES}
  DEPENDENCIES
    builtin_interfaces
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()