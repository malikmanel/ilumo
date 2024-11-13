cmake_minimum_required(VERSION 3.8)
project(sensor_publishers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(blickfeld REQUIRED)

include_directories(include)

add_executable(lidar_publisher src/lidar_publisher.cpp)
ament_target_dependencies(lidar_publisher rclcpp std_msgs sensor_msgs)

add_executable(stereo_camera_publisher src/stereo_camera_publisher.cpp)
ament_target_dependencies(stereo_camera_publisher rclcpp std_msgs sensor_msgs)

add_executable(thermal_camera_publisher src/thermal_camera_publisher.cpp)
ament_target_dependencies(thermal_camera_publisher rclcpp std_msgs sensor_msgs)

install(TARGETS
  lidar_publisher
  stereo_camera_publisher
  thermal_camera_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
