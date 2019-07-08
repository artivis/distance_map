cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)
project(distance_map_tools)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no "
                      "C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  backward_ros
  distance_map_core
  interactive_markers
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    backward_ros
    distance_map_core
    interactive_markers
    visualization_msgs
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

###############
##   Build   ##
###############

add_executable(distance_map_imarker src/distance_map_imarker.cpp)
target_link_libraries(distance_map_imarker ${catkin_LIBRARIES})

# Require C++11
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(distance_map_imarker PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET distance_map_imarker PROPERTY CXX_STANDARD 11)
  set_property(TARGET distance_map_imarker PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET distance_map_imarker PROPERTY CXX_EXTENSIONS OFF)
endif()

################
##  Install   ##
################

install(TARGETS distance_map_imarker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch) #config
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
    PATTERN *.example EXCLUDE)
endforeach(dir)
