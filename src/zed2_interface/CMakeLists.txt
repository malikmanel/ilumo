cmake_minimum_required(VERSION 3.8)
project(zed2_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_path(HIDAPI_INCLUDE_DIR
  NAMES hidapi.h
  PATH_SUFFIXES "include" "hidapi" "hidapi")
find_library(HIDAPI_LIBRARY
  NAMES hidapi hidapi-libusb
  PATH_SUFFIXES "lib" "lib32" "lib64")

find_path(LIBUSB_INCLUDE_DIR
  NAMES libusb.h
  PATH_SUFFIXES "include" "libusb" "libusb-1.0")
find_library(LIBUSB_LIBRARY
  NAMES usb-1.0 libusb usb
  PATH_SUFFIXES "lib" "lib32" "lib64")

find_package(ament_cmake REQUIRED)

include_directories(
  include
  ${HIDAPI_INCLUDE_DIR}
  ${LIBUSB_INCLUDE_DIR}
)

link_libraries(
  ${HIDAPI_LIBRARY}
  ${LIBUSB_LIBRARY}
)

install(
  DIRECTORY include/
  DESTINATION include
)

add_executable(videocapture src/videocapture.cpp)
ament_target_dependencies(videocapture)

add_executable(sensorcapture src/sensorcapture.cpp)
ament_target_dependencies(sensorcapture)

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
