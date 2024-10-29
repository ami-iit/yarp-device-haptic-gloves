# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

# Finds the Sense Glove SDK
#
# This will define the following variables::
#
#   SenseGlove_FOUND         - True if the system has the Sense Glove SDK
#   SenseGlove               - The name of the libraries to link against.
###############################

# Check Directory of the SenseGlove_DIR
if(NOT DEFINED ENV{SenseGlove_DIR})
  message( FATAL_ERROR "Environment variable {SenseGlove_DIR} is not defined." )
else()
  message(STATUS "Environment variable {SenseGlove_DIR}: $ENV{SenseGlove_DIR}" )
endif()

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  set(BUILD_TYPE "Debug")
else()
  set(BUILD_TYPE "Release")
endif()
message(STATUS "SenseGlove is linking to BUILD_TYPE: ${BUILD_TYPE}")


if(WIN32)
  if (${MSVC_TOOLSET_VERSION} STREQUAL "142")
      file(GLOB SenseGlove_LIB $ENV{SenseGlove_DIR}/lib/win64/msvc142/${BUILD_TYPE}/sgcore.lib )
  elseif (${MSVC_TOOLSET_VERSION} STREQUAL "143")
      file(GLOB SenseGlove_LIB $ENV{SenseGlove_DIR}/lib/win64/msvc143/${BUILD_TYPE}/sgcore.lib )
  else()
      message( FATAL_ERROR "SenseGlove SDK is not available for the current MSVC_TOOLSET_VERSION: ${MSVC_TOOLSET_VERSION}" )
  endif()
  set(LIB_TYPE "STATIC")
else()
  file(GLOB SenseGlove_LIB $ENV{SenseGlove_DIR}/lib/linux/v22/x86-64/${BUILD_TYPE}/libsgcore.so )
  set(LIB_TYPE "SHARED")
endif()

set(SenseGlove_INCLUDE_DIRS $ENV{SenseGlove_DIR}/include/ )

message(STATUS "Variable {SenseGlove_LIB}: ${SenseGlove_LIB}" )
message(STATUS "variable {SenseGlove_INCLUDE_DIRS}: ${SenseGlove_INCLUDE_DIRS}" )

##### Find SenseGlove #####

add_library(SenseGloveSDK ${LIB_TYPE} IMPORTED GLOBAL ${SenseGlove_LIB})
set_target_properties(SenseGloveSDK PROPERTIES IMPORTED_LOCATION ${SenseGlove_LIB})
target_include_directories(SenseGloveSDK INTERFACE ${SenseGlove_INCLUDE_DIRS})

set(SenseGloveSDK_FOUND TRUE)
