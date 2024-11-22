cmake_minimum_required(VERSION 3.8)
project(zed2_interface)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

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
  NAMES usb
  PATH_SUFFIXES "lib" "lib32" "lib64")

find_package(ament_cmake REQUIRED)

include_directories(
  include
  ${HIDAPI_INCLUDE_DIR}
  ${HIDAPI_INCLUDE_DIR}
)

link_libraries(
  ${HIDAPI_LIBRARY}
  ${HIDAPI_LIBRARY}
)

add_library(
    ${PROJECT_NAME} 

    src/sensorcapture.cpp
    src/videocapture.cpp
)

target_include_directories(zed2_interface
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# ament_target_dependencies(zed2_interface PUBLIC some_dependency)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS zed2_interface
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies(some_dependency)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
