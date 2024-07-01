# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

# Finds the Manus Glove SDK
#
# This will define the following variables::
#
#   ManusGlove_FOUND         - True if the system has the Manus Glove SDK
#   ManusGlove               - The name of the libraries to link against.
###############################

# Check Directory of the ManusGlove_DIR
if(NOT DEFINED ENV{ManusGlove_DIR})
  message( FATAL_ERROR "Environment variable {ManusGlove_DIR} is not defined." )
else()
  message(STATUS "Environment variable {ManusGlove_DIR}: $ENV{ManusGlove_DIR}" )
endif()

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  set(BUILD_TYPE "Debug")
else()
  set(BUILD_TYPE "Release")
endif()
message(STATUS "ManusGlove is linking to BUILD_TYPE: ${BUILD_TYPE}")


if(WIN32)
  file(GLOB ManusGlove_LIB $ENV{ManusGlove_DIR}/ManusSDK/ManusSDK.lib )
  set(LIB_TYPE "STATIC")
else()
  file(GLOB ManusGlove_LIB $ENV{ManusGlove_DIR}/ManusSDK/ManusSDK.so )
  set(LIB_TYPE "SHARED")
endif()

set(ManusGlove_INCLUDE_DIRS $ENV{ManusGlove_DIR} )

message(STATUS "Variable {ManusGlove_LIB}: ${ManusGlove_LIB}" )
message(STATUS "variable {ManusGlove_INCLUDE_DIRS}: ${ManusGlove_INCLUDE_DIRS}" )

##### Find ManusGlove #####

add_library(ManusSDK ${LIB_TYPE} IMPORTED GLOBAL ${ManusGlove_LIB})
set_target_properties(ManusSDK PROPERTIES IMPORTED_LOCATION ${ManusGlove_LIB})
target_include_directories(ManusSDK INTERFACE ${ManusGlove_INCLUDE_DIRS})

set(ManusGlove_FOUND TRUE)
