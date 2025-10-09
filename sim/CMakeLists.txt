cmake_minimum_required(VERSION 3.16)
project(aoa_sim CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(gazebo REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}/../include
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(aoa_plugin SHARED src/aoa_plugin.cpp)
target_link_libraries(aoa_plugin ${GAZEBO_LIBRARIES})
