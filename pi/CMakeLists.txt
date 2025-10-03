add_executable(autonomous_obstacle_avoider_pi
  src/main.cpp
)
# pthread مفيد لبعض البيئات
target_link_libraries(autonomous_obstacle_avoider_pi pigpio rt pthread)
