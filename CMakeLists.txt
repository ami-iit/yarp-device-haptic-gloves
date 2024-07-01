## SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
## SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.16)

set(PROJECT_VERSION "1.0.0")

set (YARP_DEVICE_HAPTIC_GLOVES_NAME YarpDeviceHapticGloves)

project(${YARP_DEVICE_HAPTIC_GLOVES_NAME} VERSION ${PROJECT_VERSION}
                  LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Control where binaries and libraries are placed in the build folder.
# This simplifies tests running in Windows.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# Enable RPATH support for installed binaries and libraries
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}" "${CMAKE_INSTALL_FULL_LIBDIR}/yarp"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

option(YARP_DEVICE_WEART_GLOVES_ENABLE "Flag that enables building WeArtGloves device" OFF)
option(YARP_DEVICE_MANUS_GLOVES_ENABLE "Flag that enables building ManusGloves device" OFF)

# Flag to enable WeArtGloves device
if(YARP_DEVICE_WEART_GLOVES_ENABLE)
    add_subdirectory(WeartGloves)
endif()

# Flag to enable ManusGloves device
if(YARP_DEVICE_MANUS_GLOVES_ENABLE)
    add_subdirectory(ManusGloves)
endif()

include(AddUninstallTarget)
