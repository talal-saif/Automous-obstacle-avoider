cmake_minimum_required(VERSION 3.16)
project(obstacle_avoider CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(yaml-cpp QUIET)
if (NOT yaml-cpp_FOUND)
  message(FATAL_ERROR "yaml-cpp not found. Install libyaml-cpp-dev.")
endif()

find_library(PIGPIO_LIB pigpio)
if (PIGPIO_LIB)
  add_compile_definitions(HAVE_PIGPIO=1)
endif()

# ===== logic + logger =====
add_library(core
  src/navigator.cpp
  src/logger.cpp
)
target_include_directories(core PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(core PUBLIC yaml-cpp)

# ===== sim_main =====
add_executable(sim_main
  sim/sim_main.cpp
  src/motors_sim.cpp
  src/ultrasonic_sim.cpp
)
target_include_directories(sim_main PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(sim_main PRIVATE core)

# ===== pi_main (optional) =====
if (PIGPIO_LIB)
  add_executable(pi_main
    pi/pi_main.cpp
    src/motors_pi.cpp
    src/ultrasonic_pi.cpp
  )
  target_include_directories(pi_main PRIVATE ${CMAKE_SOURCE_DIR}/include)
  target_link_libraries(pi_main PRIVATE core ${PIGPIO_LIB})
else()
  message(STATUS "pi_main skipped (pigpio not found).")
endif()

# ===== Gazebo plugin toggle =====
option(BUILD_GAZEBO_PLUGIN "Build Gazebo Classic plugin" OFF)

if (BUILD_GAZEBO_PLUGIN)
  find_package(gazebo REQUIRED)   # لو مش مثبت، هيفشل بوضوح
  add_library(avoider_plugin SHARED sim/gazebo/plugins/avoider_plugin.cpp)
  target_include_directories(avoider_plugin PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${gazebo_INCLUDE_DIRS})
  target_link_libraries(avoider_plugin ${gazebo_LIBRARIES})
  set_target_properties(avoider_plugin PROPERTIES OUTPUT_NAME "avoider_plugin")
  message(STATUS "Gazebo plugin will be built.")
else()
  message(STATUS "Gazebo plugin disabled (BUILD_GAZEBO_PLUGIN=OFF).")
endif()
