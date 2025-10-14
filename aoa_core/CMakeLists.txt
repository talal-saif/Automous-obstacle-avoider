cmake_minimum_required(VERSION 3.8)
project(aoa_core)

find_package(ament_cmake REQUIRED)

# انشر ملفات الإعدادات
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
