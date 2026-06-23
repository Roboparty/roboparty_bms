# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 wentywenty
# GF RS485 Modbus protocol (used by daemon)
add_library(gf485_bms_protocol STATIC
    protocol.cpp
)
target_include_directories(gf485_bms_protocol
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gf485_bms_protocol PUBLIC ${PUBLIC_DEPENDENCIES} bms_protocol)

# GF RS485 BMS client driver (used by client library)
add_library(gf485_bms STATIC
    gf_bms_driver.cpp
)
target_include_directories(gf485_bms
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gf485_bms PUBLIC ${PUBLIC_DEPENDENCIES})
