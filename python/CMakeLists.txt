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
project(
  damiao_can_python
  VERSION 1.2.9
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(
  Python
  COMPONENTS Interpreter Development.Module
  REQUIRED)

include(FetchContent)
set(DCP_FETCH_CONTENT_COMMON_OPTIONS)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.28)
  list(APPEND DCP_FETCH_CONTENT_COMMON_OPTIONS EXCLUDE_FROM_ALL TRUE)
endif()
macro(dcp_prepare_fetchcontent)
  set(BUILD_SHARED_LIBS OFF)
  set(BUILD_TESTING OFF)
  set(DAMIAO_CAN_BUILD_CLI OFF CACHE BOOL "Build DamiaoCAN CLI tools" FORCE)
  set(CMAKE_COMPILE_WARNING_AS_ERROR FALSE)
  set(CMAKE_EXPORT_NO_PACKAGE_REGISTRY TRUE)
  set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
endmacro()

execute_process(
  COMMAND Python::Interpreter -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE
  OUTPUT_VARIABLE nanobind_ROOT)
function(dcp_ensure_nanobind)
  # We can use FIND_PACKAGE_ARGS in fetchcontent_declare() instead of
  # explicit find_package() when we require CMake 3.24 or later.
  find_package(nanobind)
  if(nanobind_FOUND)
    return()
  endif()

  set(NANOBIND_BUNDLED_VERSION "2.10.2")
  set(NANOBIND_SOURCE_BASE_NAME "nanobind-${NANOBIND_BUNDLED_VERSION}.tar.gz")
  set(NANOBIND_SOURCE_LOCAL_PATH
      "${CMAKE_CURRENT_SOURCE_DIR}/vendor/${NANOBIND_SOURCE_BASE_NAME}")
  if(EXISTS ${NANOBIND_SOURCE_LOCAL_PATH})
    set(NANOBIND_SOURCE_ARGS URL ${NANOBIND_SOURCE_LOCAL_PATH})
  else()
    set(NANOBIND_SOURCE_ARGS
        GIT_REPOSITORY "https://github.com/wjakob/nanobind.git" GIT_TAG
        "v${NANOBIND_BUNDLED_VERSION}")
  endif()
  fetchcontent_declare(
    nanobind
    ${DCP_FETCH_CONTENT_COMMON_OPTIONS}
    ${NANOBIND_SOURCE_ARGS}
    # We can use FIND_PACKAGE_ARGS when we require CMake 3.24 or later.
    #
    # FIND_PACKAGE_ARGS
  )
  dcp_prepare_fetchcontent()
  fetchcontent_makeavailable(nanobind)
  if(nanobind_SOURCE_DIR)
    if(CMAKE_VERSION VERSION_LESS 3.28)
      set_property(DIRECTORY "${nanobind_SOURCE_DIR}" PROPERTY EXCLUDE_FROM_ALL
                                                               TRUE)
    endif()
  endif()
endfunction()
dcp_ensure_nanobind()

function(dcp_ensure_damiao_can)
  # Prefer the parent repository when installing from this Git checkout.
  set(DAMIAO_CAN_LOCAL_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
  if(EXISTS "${DAMIAO_CAN_LOCAL_SOURCE_DIR}/include/damiao_can/can/socket/damiao_can.hpp"
     AND EXISTS "${DAMIAO_CAN_LOCAL_SOURCE_DIR}/CMakeLists.txt")
    fetchcontent_declare(
      damiao_can
      ${DCP_FETCH_CONTENT_COMMON_OPTIONS}
      SOURCE_DIR "${DAMIAO_CAN_LOCAL_SOURCE_DIR}")
    dcp_prepare_fetchcontent()
    fetchcontent_makeavailable(damiao_can)

    if(damiao_can_SOURCE_DIR AND CMAKE_VERSION VERSION_LESS 3.28)
      set_property(DIRECTORY "${damiao_can_SOURCE_DIR}" PROPERTY EXCLUDE_FROM_ALL TRUE)
    endif()

    if(TARGET damiao_can AND NOT TARGET DamiaoCAN::damiao_can)
      add_library(DamiaoCAN::damiao_can ALIAS damiao_can)
    endif()
    return()
  endif()

  find_package(DamiaoCAN ${damiao_can_python_VERSION} REQUIRED)
endfunction()
dcp_ensure_damiao_can()

nanobind_add_module(damiao_can_python src/damiao_can.cpp)
set_target_properties(damiao_can_python PROPERTIES OUTPUT_NAME "damiao_can")
target_link_libraries(damiao_can_python PRIVATE DamiaoCAN::damiao_can)
install(TARGETS damiao_can_python LIBRARY DESTINATION "damiao_can")
install(
  FILES
    damiao_can/__init__.py
    damiao_can/__init__.pyi
    damiao_can/damiao_can.pyi
    damiao_can/py.typed
  DESTINATION "damiao_can")
