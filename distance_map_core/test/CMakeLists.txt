enable_testing()

find_package(rostest REQUIRED)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

catkin_add_gtest(gtest_grid gtest_grid.cpp)
target_link_libraries(gtest_grid ${PROJECT_NAME} ${catkin_LIBRARIES})

# Require C++11
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(gtest_grid PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET gtest_grid PROPERTY CXX_STANDARD 11)
  set_property(TARGET gtest_grid PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET gtest_grid PROPERTY CXX_EXTENSIONS OFF)
endif()
