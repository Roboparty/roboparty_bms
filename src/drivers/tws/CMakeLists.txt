# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 wentywenty
# TWS BMS Modbus protocol (used by daemon)
add_library(tws_bms_protocol STATIC
    protocol.cpp
)
target_include_directories(tws_bms_protocol
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(tws_bms_protocol PUBLIC ${PUBLIC_DEPENDENCIES} bms_protocol)

# TWS BMS client driver (used by client library)
add_library(tws_bms STATIC
    tws_bms_driver.cpp
)
target_include_directories(tws_bms
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(tws_bms PUBLIC ${PUBLIC_DEPENDENCIES})
