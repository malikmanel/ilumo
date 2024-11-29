cmake_minimum_required(VERSION 3.8)
project(ilumo_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_FIND_DEBUG_MODE 1)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(zed2_interface REQUIRED)
find_package(blickfeld-scanner 2.18.2 REQUIRED full)

find_library(BLICKFELD_LIBRARY
  NAMES blickfeld-scanner
  PATH_SUFFIXES "lib" "lib32" "lib64")

include_directories(
  include
)

add_executable(lidar_publisher src/lidar_publisher.cpp)
target_link_libraries(lidar_publisher
  ${BLICKFELD_LIBRARY})
ament_target_dependencies(lidar_publisher 
  rclcpp std_msgs sensor_msgs blickfeld-scanner Protobuf)

add_executable(stereo_camera_publisher src/stereo_camera_publisher.cpp)
ament_target_dependencies(stereo_camera_publisher 
  rclcpp std_msgs sensor_msgs zed2_interface)

add_executable(thermal_camera_publisher src/thermal_camera_publisher.cpp)
ament_target_dependencies(thermal_camera_publisher 
  rclcpp std_msgs sensor_msgs)

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

# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#     message("${_variableName}=${${_variableName}}")
# endforeach()

ament_package()
