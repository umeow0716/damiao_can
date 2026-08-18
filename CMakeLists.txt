# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(damiao_can VERSION 1.2.9)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Testing support
include(CTest)

include(GNUInstallDirs)

# Create the main library
add_library(
  damiao_can
  src/damiao_can/can/socket/motor_component.cpp
  src/damiao_can/can/socket/damiao_can_group.cpp
  src/damiao_can/can/socket/damiao_can.cpp
  src/damiao_can/canbus/can_device_collection.cpp
  src/damiao_can/canbus/can_helper.cpp
  src/damiao_can/canbus/can_socket.cpp
  src/damiao_can/damiao_motor/dm_motor.cpp
  src/damiao_can/damiao_motor/dm_motor_control.cpp
  src/damiao_can/damiao_motor/dm_motor_device.cpp
  src/damiao_can/damiao_motor/dm_motor_device_collection.cpp)
set_target_properties(
  damiao_can
  PROPERTIES POSITION_INDEPENDENT_CODE ON
             VERSION ${PROJECT_VERSION}
             SOVERSION ${PROJECT_VERSION_MAJOR})
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.23)
  target_sources(
    damiao_can
    PUBLIC FILE_SET
           HEADERS
           TYPE
           HEADERS
           BASE_DIRS
           include
           FILES
           include/damiao_can/can/socket/motor_component.hpp
           include/damiao_can/can/socket/damiao_can_group.hpp
           include/damiao_can/can/socket/damiao_can.hpp
           include/damiao_can/canbus/can_device.hpp
           include/damiao_can/canbus/can_device_collection.hpp
           include/damiao_can/canbus/can_helper.hpp
           include/damiao_can/canbus/can_socket.hpp
           include/damiao_can/damiao_motor/dm_motor.hpp
           include/damiao_can/damiao_motor/dm_motor_constants.hpp
           include/damiao_can/damiao_motor/dm_motor_control.hpp
           include/damiao_can/damiao_motor/dm_motor_device.hpp
           include/damiao_can/damiao_motor/dm_motor_device_collection.hpp)
  install(
    TARGETS damiao_can
    EXPORT damiao_can_export
    FILE_SET HEADERS)
else()
  target_include_directories(
    damiao_can PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include>)
  install(TARGETS damiao_can EXPORT damiao_can_export)
  install(DIRECTORY include/damiao_can TYPE INCLUDE)
endif()

# CMake package
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/DamiaoCAN)
install(
  EXPORT damiao_can_export
  DESTINATION ${INSTALL_CMAKE_DIR}
  NAMESPACE DamiaoCAN::
  FILE DamiaoCANTargets.cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  DamiaoCANConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/DamiaoCANConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKE_DIR})
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/DamiaoCANConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/DamiaoCANConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/DamiaoCANConfigVersion.cmake
        DESTINATION ${INSTALL_CMAKE_DIR})

# pkg-config package
if(IS_ABSOLUTE "${CMAKE_INSTALL_INCLUDEDIR}")
  set(PKG_CONFIG_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}")
else()
  set(PKG_CONFIG_INCLUDEDIR "\${prefix}/${CMAKE_INSTALL_INCLUDEDIR}")
endif()
if(IS_ABSOLUTE "${CMAKE_INSTALL_LIBDIR}")
  set(PKG_CONFIG_LIBDIR "${CMAKE_INSTALL_LIBDIR}")
else()
  set(PKG_CONFIG_LIBDIR "\${prefix}/${CMAKE_INSTALL_LIBDIR}")
endif()
configure_file(damiao-can.pc.in ${CMAKE_CURRENT_BINARY_DIR}/damiao-can.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/damiao-can.pc
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

# C++ demo executable. This is intentionally the only C++ command-line target.
add_executable(damiao-can-demo examples/demo.cpp)
target_link_libraries(damiao-can-demo damiao_can)
install(TARGETS damiao-can-demo DESTINATION ${CMAKE_INSTALL_BINDIR})

# Add tests
if(BUILD_TESTING)
  # add_subdirectory(test)
endif()
