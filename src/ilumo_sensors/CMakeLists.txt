cmake_minimum_required(VERSION 3.8)
project(ilumo_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(CMAKE_FIND_DEBUG_MODE 1)

add_definitions(-DSENSORS_MOD_AVAILABLE -DVIDEO_MOD_AVAILABLE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(zed_open_capture REQUIRED)
find_package(blickfeld REQUIRED)
find_package(ilumo_interfaces REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(
  include
)

add_executable(lidar_publisher src/lidar_publisher.cpp)
ament_target_dependencies(lidar_publisher 
  rclcpp std_msgs sensor_msgs blickfeld ilumo_interfaces Protobuf)

add_executable(stereo_camera_publisher src/stereo_camera_publisher.cpp)
ament_target_dependencies(stereo_camera_publisher 
  rclcpp std_msgs sensor_msgs zed_open_capture OpenCV cv_bridge)

install(TARGETS
  lidar_publisher
  stereo_camera_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
ilumo_sensors/thermal_camera_publisher.py
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
