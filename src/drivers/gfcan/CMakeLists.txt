# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 wentywenty
add_library(gfcan_bms STATIC
    gfcan_bms_driver.cpp
)
target_include_directories(gfcan_bms
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gfcan_bms PUBLIC ${PUBLIC_DEPENDENCIES} PRIVATE bms_protocol)
