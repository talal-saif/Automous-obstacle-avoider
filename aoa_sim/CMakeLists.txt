cmake_minimum_required(VERSION 3.8)
project(aoa_sim)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

# تثبيت سكربتات بايثون كنود قابلة للتشغيل
install(PROGRAMS
  scripts/avoider.py
  DESTINATION lib/${PROJECT_NAME}
)

# نشر ملفات التشغيل والنماذج والعوالم
install(DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
