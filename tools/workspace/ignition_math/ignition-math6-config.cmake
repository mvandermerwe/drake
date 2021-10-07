# TODO(jwnimmer-tri) On 2021-12-01 when the ":ignition_math" target is removed,
# we should also remove this file (and its mention in BUILD.bazel).

# Generated by cps2cmake https://github.com/mwoehlke/pycps
# and then subsequently edited by hand.

if(CMAKE_VERSION VERSION_LESS 3.9.0)
  message(FATAL_ERROR "CMake >= 3.9 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 3.0)
set(CMAKE_IMPORT_FILE_VERSION 1)

include(CMakeFindDependencyMacro)

get_filename_component(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}" PATH)
get_filename_component(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}" PATH)
get_filename_component(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}" PATH)

if(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX STREQUAL "/")
  set(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX)
endif()

set(_expectedTargets ignition-math6::ignition-math6)

set(_targetsDefined)
set(_targetsNotDefined)

foreach(_expectedTarget ${_expectedTargets})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)

set(ignition-math6_VERSION "6.8.0")

set(_apple_soname_prologue)
if(APPLE)
  set(_apple_soname_prologue "@rpath/")
endif()
add_library(ignition-math6::ignition-math6 SHARED IMPORTED)
set_target_properties(ignition-math6::ignition-math6 PROPERTIES
  IMPORTED_LOCATION "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib/libdrake_ignition_math.so"
  IMPORTED_SONAME "${_apple_soname_prologue}libdrake_ignition_math.so"
  INTERFACE_INCLUDE_DIRECTORIES "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/include/ignition-math6"
)
unset(_apple_soname_prologue)

set(ignition-math6_LIBRARIES "ignition-math6::ignition-math6")
set(ignition-math6_INCLUDE_DIRS "")

set(IGNITION-MATH_INCLUDE_DIRS "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/include")
set(IGNITION-MATH_LINK_DIRS "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib")
set(IGNITION-MATH_LIBRARY_DIRS "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib")
set(IGNITION-MATH_LIBRARIES "ignition_math")

unset(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX)
unset(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
