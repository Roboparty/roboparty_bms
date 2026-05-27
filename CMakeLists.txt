# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 wentywenty

cmake_minimum_required(VERSION 3.12)

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/debian/changelog" _changelog_first_line LIMIT_COUNT 1)
string(REGEX MATCH "\\(([0-9]+\\.[0-9]+\\.[0-9]+)" _ "${_changelog_first_line}")
set(ROBOPARTY_BMS_VERSION "${CMAKE_MATCH_1}")

project(roboparty_bms VERSION ${ROBOPARTY_BMS_VERSION})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
if(NOT CMAKE_CROSSCOMPILING)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()
set(CMAKE_CXX_COMPILER_LAUNCHER ccache)

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Release)

find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

set(PUBLIC_DEPENDENCIES
    fmt::fmt spdlog::spdlog pthread)

add_subdirectory(src/drivers)

# ---- Daemon executable ----
add_executable(bms_daemon
    src/bms_daemon.cpp
)
target_include_directories(bms_daemon PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(bms_daemon PRIVATE tws_bms_protocol fmt::fmt spdlog::spdlog pthread)

# ---- Client library ----
add_library(bms STATIC
    src/bms_driver.cpp
)
target_include_directories(bms
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(bms PUBLIC ${PUBLIC_DEPENDENCIES} tws_bms)

# ---- Python bindings ----
pybind11_add_module(bms_py src/pybind_module.cpp)
target_link_libraries(bms_py PUBLIC bms)

# ---- Install ----
install(DIRECTORY include/ DESTINATION include)

install(TARGETS bms_daemon DESTINATION bin)

install(TARGETS bms tws_bms
    EXPORT roboparty_bms-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(EXPORT roboparty_bms-targets
  FILE roboparty_bmsTargets.cmake
  NAMESPACE roboparty_bms::
  DESTINATION lib/cmake/roboparty_bms
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/roboparty_bmsConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/roboparty_bmsConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/roboparty_bmsConfig.cmake"
  INSTALL_DESTINATION lib/cmake/roboparty_bms
)

# Install auto-generated Config for non-ROS consumers (Deb mode)
install(FILES 
  "${CMAKE_CURRENT_BINARY_DIR}/roboparty_bmsConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/roboparty_bmsConfigVersion.cmake"
  DESTINATION lib/cmake/roboparty_bms
)

set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
install(TARGETS bms_py
    LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
    ARCHIVE DESTINATION ${PYTHON_INSTALL_DIR}
    RUNTIME DESTINATION ${PYTHON_INSTALL_DIR}
)

