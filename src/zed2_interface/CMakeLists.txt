cmake_minimum_required(VERSION 3.8)
project(zed2_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(libusb 1 REQUIRED)
find_package(hidapi REQUIRED)

include_directories(
  include
)

install(
  DIRECTORY include/
  DESTINATION include
)

add_executable(videocapture src/videocapture.cpp)
ament_target_dependencies(videocapture)

add_executable(sensorcapture src/sensorcapture.cpp)
ament_target_dependencies(sensorcapture hidapi)

install(TARGETS
sensorcapture
  videocapture
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
