# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 wentywenty
add_library(nrf_pmic STATIC
    nrf_pmic_driver.cpp
)
target_include_directories(nrf_pmic
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(nrf_pmic PUBLIC ${PUBLIC_DEPENDENCIES})
