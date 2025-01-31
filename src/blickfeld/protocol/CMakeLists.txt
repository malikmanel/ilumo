##
## Copyright (c) 2020 Blickfeld GmbH.
## All rights reserved.
##
## This source code is licensed under the BSD-style license found in the
## LICENSE.md file in the root directory of this source tree.
##
cmake_minimum_required (VERSION 3.9)
include(ExternalProject)

project(blickfeld-protocol)

# NPM
find_program(NODEJS_EXECUTABLE NAMES node)
	
## Protocol Buffers
message("Existing protobuf compiler is used: ${Protobuf_PROTOC_EXECUTABLE}")
set(Protobuf_PROTOC_EXECUTABLE ${Protobuf_PROTOC_EXECUTABLE} PARENT_SCOPE)

# list .proto files here
set(PROTOBUF_DEFS_DIR ${PROJECT_SOURCE_DIR})
set(PROTOBUF_SOURCES	
	blickfeld/config/algorithm.proto
	blickfeld/config/advanced.proto
	blickfeld/config/product.proto
	blickfeld/config/generate.proto
	blickfeld/config/scan_pattern.proto
	blickfeld/config/secure.proto
	blickfeld/config/named_scan_pattern.proto
	
	blickfeld/status/main.proto
	blickfeld/status/scanner.proto
	blickfeld/status/temperature.proto
	blickfeld/status/server.proto
	blickfeld/status/time_synchronization.proto
	blickfeld/status/imu.proto
	
	blickfeld/data/algorithm.proto
	blickfeld/data/frame.proto
	blickfeld/data/imu.proto
	blickfeld/data/point_cloud.proto
	blickfeld/data/point.proto
	blickfeld/data/scanline.proto
	
	blickfeld/file/general.proto
	blickfeld/file/point_cloud.proto
	
	blickfeld/stream/connection.proto
	blickfeld/stream/event.proto

	blickfeld/update/hardware.proto
	blickfeld/update/manifest.proto
	
	blickfeld/common.proto
	blickfeld/options.proto
	blickfeld/error.proto
	blickfeld/connection.proto
)

set(PROTOBUF_SOURCES_CUSTOM
	blickfeld/error.proto
)

set(PROTOBUF_GENERATED_PY_DIR ${PROJECT_BINARY_DIR}/python)

set(PROTOBUF_GENERATED_CPP)
set(PROTOBUF_GENERATED_HEADER)
set(PROTOBUF_GENERATED_PY)
set(PROTOBUF_GENERATED_JS)

set(PROTOBUF_SOURCES_WITH_ABSOLUTE_PATH "")

foreach(FILE ${PROTOBUF_SOURCES})
	list(APPEND PROTOBUF_SOURCES_WITH_ABSOLUTE_PATH "${PROTOBUF_DEFS_DIR}/${FILE}")

	string(REPLACE ".proto" ".pb.cc" FILE_CC ${FILE})
	string(REPLACE ".proto" ".pb.h" FILE_H ${FILE})
	
	# Cpp Build
	add_custom_command(
		OUTPUT ${PROJECT_BINARY_DIR}/${FILE_CC}
		OUTPUT ${LIBINCLUDEDIR_TOP}/${FILE_H}
		
		#COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}
		COMMAND ${Protobuf_PROTOC_EXECUTABLE} ${PROTOBUF_DEFS_DIR}/${FILE} --cpp_out=${DLL_EXPORT}${PROJECT_BINARY_DIR} --proto_path=${PROTOBUF_DEFS_DIR}
		COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_BINARY_DIR}/${FILE_H} ${LIBINCLUDEDIR_TOP}/${FILE_H}
		COMMAND ${CMAKE_COMMAND} -E remove ${PROJECT_BINARY_DIR}/${FILE_H}
		
		DEPENDS $<TARGET_NAME_IF_EXISTS:blickfeld-protobuf>
		DEPENDS ${FILE}
		WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
	)
	list(APPEND PROTOBUF_GENERATED_CPP ${PROJECT_BINARY_DIR}/${FILE_CC})
	list(APPEND PROTOBUF_GENERATED_HEADER ${LIBINCLUDEDIR_TOP}/${FILE_H})
endforeach()

# Custom plugin protobuf build
set(PROTOBUF_GENERATED_CUSTOM)
foreach(FILE ${PROTOBUF_SOURCES_CUSTOM})
	string(REPLACE ".proto" ".custom.pb.h" FILE_H ${FILE})
	add_custom_command(
		OUTPUT ${LIBINCLUDEDIR_TOP}/${FILE_H}
		COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/plugin/${FILE_H} ${LIBINCLUDEDIR_TOP}/${FILE_H}
		DEPENDS ${FILE}
		WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
	)
	list(APPEND PROTOBUF_GENERATED_CUSTOM ${LIBINCLUDEDIR_TOP}/${FILE_H})
endforeach()

add_library(blickfeld-scanner-protocol-static OBJECT ${PROTOBUF_GENERATED_CUSTOM} ${PROTOBUF_GENERATED_CPP})
target_include_directories(blickfeld-scanner-protocol-static PRIVATE ${LIBINCLUDEDIR_TOP})
target_link_libraries(blickfeld-scanner-protocol-static PUBLIC ${Protobuf_LIBRARY})
set_property(TARGET blickfeld-scanner-protocol-static PROPERTY POSITION_INDEPENDENT_CODE ON)

set(PROTOBUF_GENERATED_CPP ${PROTOBUF_GENERATED_CPP} PARENT_SCOPE)
set(PROTOBUF_GENERATED_PY_DIR ${PROTOBUF_GENERATED_PY_DIR} PARENT_SCOPE)
set(PROTOBUF_GENERATED_PY ${PROTOBUF_GENERATED_PY} PARENT_SCOPE)
set(PROTOBUF_GENERATED_JS ${PROTOBUF_GENERATED_JS} PARENT_SCOPE)
set(PROTOBUF_GENERATED_JS_DIR ${PROJECT_BINARY_DIR}/npm PARENT_SCOPE)

add_custom_target(blickfeld-scanner-protocol 
	DEPENDS blickfeld-scanner-protocol-static
	DEPENDS ${PROTOBUF_GENERATED_CPP}
	DEPENDS ${PROTOBUF_GENERATED_PY})

add_custom_target(blickfeld-scanner-protocol-js
	DEPENDS ${PROTOBUF_GENERATED_JS})
