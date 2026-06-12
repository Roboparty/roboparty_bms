# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 wentywenty
# GF BMS Modbus protocol (used by daemon)
add_library(gf_bms_protocol STATIC
    protocol.cpp
)
target_include_directories(gf_bms_protocol
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gf_bms_protocol PUBLIC ${PUBLIC_DEPENDENCIES})

# GF BMS client driver (used by client library)
add_library(gf_bms STATIC
    gf_bms_driver.cpp
)
target_include_directories(gf_bms
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gf_bms PUBLIC ${PUBLIC_DEPENDENCIES})
