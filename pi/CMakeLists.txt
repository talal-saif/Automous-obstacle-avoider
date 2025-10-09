cmake_minimum_required(VERSION 3.16)
project(aoa_pi CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Threads REQUIRED)
find_path(PIGPIO_INCLUDE_DIR pigpio.h)
find_library(PIGPIO_LIBRARY pigpio)

if (PIGPIO_INCLUDE_DIR AND PIGPIO_LIBRARY)
  message(STATUS "Found pigpio: ${PIGPIO_LIBRARY}")
  add_executable(aoa_pi src/main.cpp)
  target_include_directories(aoa_pi PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../include
    ${PIGPIO_INCLUDE_DIR}
  )
  target_link_libraries(aoa_pi PRIVATE ${PIGPIO_LIBRARY} Threads::Threads)
else()
  message(WARNING "pigpio not found → skipping aoa_pi build on this machine")
endif()
