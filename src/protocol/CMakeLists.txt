# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 wentywenty
file(GLOB_RECURSE PROTOCOL_SRCS *.cpp)

add_library(bms_protocol
    STATIC
    ${PROTOCOL_SRCS}
)
target_include_directories(bms_protocol
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/can>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/serial>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(bms_protocol PUBLIC ${PUBLIC_DEPENDENCIES})
