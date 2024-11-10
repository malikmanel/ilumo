cmake_minimum_required(VERSION 3.8)
project(sensor_publishers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(blickfeld REQUIRED)

add_executable(lidar_interface src/lidar_interface.cpp)
ament_target_dependencies(lidar_interface rclcpp std_msgs blickfeld)

add_executable(stereo_camera_interface src/stereo_camera_interface.cpp)
ament_target_dependencies(stereo_camera_interface rclcpp std_msgs)

add_executable(thermal_camera_interface src/thermal_camera_interface.cpp)
ament_target_dependencies(thermal_camera_interface rclcpp std_msgs)

install(TARGETS
  lidar_interface
  stereo_camera_interface
  thermal_camera_interface
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
