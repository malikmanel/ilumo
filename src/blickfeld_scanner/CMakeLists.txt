##
## Copyright (c) 2020 Blickfeld GmbH.
## All rights reserved.
##
## This source code is licensed under the BSD-style license found in the
## LICENSE.md file in the root directory of this source tree.
##
cmake_minimum_required (VERSION 3.8)
project (blickfeld_scanner)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

include(GNUInstallDirs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEFAULT_LOG_MSGS_KEPT_IN_MEM 50)

# Set lib include directories
set (LIBINCLUDEDIR "include/blickfeld")
set (LIBINCLUDEDIR_TOP "${PROJECT_BINARY_DIR}/include")

find_package(ament_cmake REQUIRED)
find_package(Protobuf 3.0.0 REQUIRED)
find_package(ZLIB REQUIRED)
find_package(OpenSSL REQUIRED)

configure_file("${PROJECT_SOURCE_DIR}/include/blickfeld/scanner-config.h.in" "${PROJECT_BINARY_DIR}/include/blickfeld/scanner-config.h")

# Sources and includes
set(BLICKFELD_SCANNER_SOURCES
  src/string_utils.cpp
  src/exception.cpp
  src/scanner.cpp
  src/utils.cpp
  src/logged_object.cpp
  src/scanner_connection.cpp
  src/connection.cpp
  src/discover.cpp
  src/point_cloud_record.cpp
  src/point_cloud_stream.cpp
  src/mockup_server.cpp
  src/mockup_session.cpp
  src/simulated_connection.cpp
  src/imu_stream.cpp
)

include_directories(
  ${Protobuf_INCLUDE_DIRS}
  include
  src
  thirdparty/portable_endian
  thirdparty/asio/include
  ${PROJECT_BINARY_DIR}/include
)

add_subdirectory(protocol)

add_library (blickfeld-scanner SHARED 
  ${BLICKFELD_SCANNER_SOURCES} $<TARGET_OBJECTS:blickfeld-scanner-protocol-static>)
add_library (blickfeld-scanner-static STATIC EXCLUDE_FROM_ALL 
  ${BLICKFELD_SCANNER_SOURCES} $<TARGET_OBJECTS:blickfeld-scanner-protocol-static>)
target_include_directories(blickfeld-scanner-static PUBLIC ${BLICKFELD_SCANNER_INCLUDES})

add_dependencies(blickfeld-scanner blickfeld-scanner-protocol)
add_dependencies(blickfeld-scanner-static blickfeld-scanner-protocol)

target_link_libraries(blickfeld-scanner PRIVATE ${Protobuf_LIBRARY})
target_include_directories(blickfeld-scanner PRIVATE ${Protobuf_INCLUDE_DIRS})

target_link_libraries(blickfeld-scanner-static PUBLIC ${Protobuf_LIBRARY})
target_include_directories(blickfeld-scanner-static PUBLIC ${Protobuf_INCLUDE_DIRS})

target_link_libraries(blickfeld-scanner PUBLIC OpenSSL::SSL OpenSSL::Crypto)
target_link_libraries(blickfeld-scanner-static PUBLIC OpenSSL::SSL OpenSSL::Crypto)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(blickfeld-scanner INTERFACE ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(blickfeld-scanner-static INTERFACE ${CMAKE_THREAD_LIBS_INIT})

target_include_directories(blickfeld-scanner INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS blickfeld-scanner EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )

install(
  FILES LICENSE.md
  DESTINATION include/${PROJECT_NAME}
)
# Generated headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)
# Protocol file
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/protocol/blickfeld/
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/blickfeld-scanner/protocol/blickfeld
  FILES_MATCHING PATTERN "*.proto"
)

set(CPACK_PACKAGE_NAME "${PROJECT_NAME}-dev")
if(CPACK_DEBIAN_PACKAGE_DEPENDS)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS},")
endif()
set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}libprotobuf-dev (>= ${Protobuf_VERSION})")
if(${Protobuf_VERSION} VERSION_GREATER_EQUAL "3.11.4")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS},libprotobuf23")
  set(CPACK_PACKAGE_NAME "${PROJECT_NAME}-dev-testing")
elseif(${Protobuf_VERSION} VERSION_GREATER_EQUAL "3.6.1")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS},libprotobuf17")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies(some_dependency)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()