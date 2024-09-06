cmake_minimum_required(VERSION 3.8)
project(robosense_m1plus_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robosense_m1plus_interfaces REQUIRED)

include_directories(include)

add_executable(robosense_m1plus_driver src/robosense_m1plus_driver.cpp)

ament_target_dependencies(robosense_m1plus_driver
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  robosense_m1plus_interfaces
)

install(TARGETS robosense_m1plus_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
