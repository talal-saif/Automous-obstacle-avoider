cmake_minimum_required(VERSION 3.16)
project(autonomous_obstacle_avoider CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# تحذيرات قوية + سانيتايزر في Debug
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-fsanitize=address,undefined)
    add_link_options(-fsanitize=address,undefined)
  endif()
endif()

# مسار الهيدرز
include_directories(${CMAKE_SOURCE_DIR}/include)

add_subdirectory(sim)
add_subdirectory(pi)
