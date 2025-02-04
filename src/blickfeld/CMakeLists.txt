##
## Copyright (c) 2020 Blickfeld GmbH.
## All rights reserved.
##
## This source code is licensed under the BSD-style license found in the
## LICENSE.md file in the root directory of this source tree.
##
cmake_minimum_required (VERSION 3.8)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Get version of BSL
set(PROJECT_VERSION v2.20.6-30)
set(PROJECT_GIT_DESCRIBE 2.20.6-30)

string(REGEX REPLACE "^v([0-9.]+).*" "\\1" PROJECT_VERSION "${PROJECT_VERSION}")
string(REGEX REPLACE "^v" "" PROJECT_GIT_DESCRIBE "${PROJECT_GIT_DESCRIBE}")
string(REPLACE "." ";" VERSION_LIST ${PROJECT_VERSION})
list(GET VERSION_LIST 0 BSL_VERSION_MAJOR)
list(GET VERSION_LIST 1 BSL_VERSION_MINOR)
list(GET VERSION_LIST 2 BSL_VERSION_PATCH)
set(BSL_VERSION ${PROJECT_GIT_DESCRIBE})
message("BSL Version ${PROJECT_GIT_DESCRIBE}")
project (blickfeld
	VERSION ${PROJECT_VERSION} 
	DESCRIPTION "Blickfeld Scanner Library for controlling LiDAR devices of the Blickfeld GmbH"
	HOMEPAGE_URL https://www.blickfeld.com
)
include(GNUInstallDirs)

set(SERVER_PORT 8000)

include (CheckIncludeFiles)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

check_include_files (windows.h HAVE_WINDOWS_H)
check_include_files (sys/socket.h HAVE_SYS_TYPES_H)
check_include_files (sys/socket.h HAVE_SYS_SOCKET_H)
check_include_files (netinet/in.h HAVE_NETINET_IN_H)
check_include_files (netinet/ip.h HAVE_NETINET_IP_H)
check_include_files (arpa/inet.h HAVE_ARPA_INET_H)
check_include_files (netdb.h HAVE_NETDB_H)
check_include_files (unistd.h HAVE_UNISTD_H)
check_include_files (syslog.h HAVE_SYSLOG_H)
check_include_files (time.h HAVE_TIME_H)
check_include_files (inttypes.h HAVE_INTTYPES_H)
check_include_files (winsock.h HAVE_WINSOCK_H)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include (CheckSymbolExists)
check_symbol_exists(clock_gettime time.h HAVE_CLOCK_GETTIME)
check_symbol_exists(MSG_NOSIGNAL "sys/types.h;sys/socket.h" HAVE_MSG_NOSIGNAL)

set(DEFAULT_LOG_MSGS_KEPT_IN_MEM 50)

# Set lib include directories
set (LIBINCLUDEDIR "include/blickfeld")
set (LIBINCLUDEDIR_TOP "${PROJECT_BINARY_DIR}/include") # TODO: Issue here

# Protobuf
find_package(Protobuf 3.0.0 REQUIRED)

# Check version of protobuf
if(PROTOBUF_FOUND AND Protobuf_PROTOC_EXECUTABLE)
  execute_process(COMMAND ${Protobuf_PROTOC_EXECUTABLE} --version OUTPUT_VARIABLE Protobuf_Compiler_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
  string(REGEX REPLACE "^libprotoc " "" Protobuf_Compiler_VERSION "${Protobuf_Compiler_VERSION}")

  if(NOT Protobuf_VERSION STREQUAL Protobuf_Compiler_VERSION)
    message( WARNING "Mismatch of protoc version (is ${Protobuf_Compiler_VERSION}) and libprotobuf version (is ${Protobuf_VERSION}). This might cause severe build issues.")
  endif()
endif()

# Only pass if we found protobuf with its executable and library
if(NOT (PROTOBUF_FOUND AND Protobuf_PROTOC_EXECUTABLE AND Protobuf_LIBRARY))
  set(PROTOBUF_FOUND FALSE)
endif()

if(PROTOBUF_FOUND)
  set(BSL_STANDALONE OFF)  # Set BSL standalone build on off. This will activate more features.
else()
  message(FATAL_ERROR "Could not find suitable protobuf on this system.")
endif()

find_package(ament_cmake REQUIRED)

# ZLIB
find_package(ZLIB)
if (ZLIB_FOUND)
	set(BSL_RECORDING ON)
else()
	set(BSL_RECORDING OFF)
	message("Did not find ZLIB. Recording and reading is disabled.")
endif()

# OpenSSL
if(BF_ENABLE_SSL)
  set(HAVE_OPENSSL ON)
  find_package(OpenSSL REQUIRED)
else()
  set(HAVE_OPENSSL OFF)
endif()

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
set(BLICKFELD_SCANNER_INCLUDES
  ${Protobuf_INCLUDE_DIRS}
  include
  src
  thirdparty/portable_endian
  thirdparty/asio/include
)

include_directories(${BLICKFELD_SCANNER_INCLUDES})

add_subdirectory(protocol)

add_library (${PROJECT_NAME} 
  ${BLICKFELD_SCANNER_SOURCES} $<TARGET_OBJECTS:blickfeld-scanner-protocol-static>)
add_library (${PROJECT_NAME}-static 
  STATIC EXCLUDE_FROM_ALL ${BLICKFELD_SCANNER_SOURCES} $<TARGET_OBJECTS:blickfeld-scanner-protocol-static>)
target_include_directories(${PROJECT_NAME}-static 
  PUBLIC 
    ${BLICKFELD_SCANNER_INCLUDES})

add_dependencies(${PROJECT_NAME} blickfeld-scanner-protocol)
add_dependencies(${PROJECT_NAME}-static blickfeld-scanner-protocol)

target_link_libraries(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${Protobuf_LIBRARY}>)
target_include_directories(${PROJECT_NAME} 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${Protobuf_INCLUDE_DIRS}> 
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)


target_link_libraries(${PROJECT_NAME}-static PUBLIC ${Protobuf_LIBRARY})
target_include_directories(${PROJECT_NAME}-static PUBLIC ${Protobuf_INCLUDE_DIRS})


if(HAVE_OPENSSL)
  target_link_libraries(${PROJECT_NAME} PUBLIC OpenSSL::SSL OpenSSL::Crypto)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} INTERFACE ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${PROJECT_NAME}-static INTERFACE ${CMAKE_THREAD_LIBS_INIT})

# Install license
install(
  FILES LICENSE.md
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/blickfeld
)

# Generated headers
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/blickfeld
  FILES_MATCHING PATTERN "*.h"
)

# Protocol file
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/protocol/blickfeld/
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/protocol/blickfeld
  FILES_MATCHING PATTERN "*.proto"
)

# Install BSL
install(
  TARGETS ${PROJECT_NAME} 
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
message("${BSL_VERSION}")