#/*
# Copyright 2024 Istituto Italiano di Tecnologia (IIT)
# @author Ehsan Ranjbari <ehsan.ranjbari@iit.it>
#* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#*/
cmake_minimum_required(VERSION 3.5)

project(YARPManusGlove VERSION "0.0.1"
                  LANGUAGES CXX C)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(ManusGloveLibName "ManusSDK")
set(ManusGloveDevName "ManusGlove")

find_package(YARP REQUIRED)
find_package(${ManusGloveLibName} REQUIRED)

find_package(IWear REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

# Export all symbols in Windows
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(YARP_FORCE_DYNAMIC_PLUGINS ON)

# Add a postfix to Windows libraries compiled in debug
if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
    
    #NOMINMAX is to avoid windows.h defining its own versions of min and max
    add_definitions(-DNOMINMAX)
endif()

# Libraries type
if(MSVC)
    option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" OFF)
else()
    option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
endif()

# Enable RPATH support for installed binaries and libraries
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}" "${CMAKE_INSTALL_FULL_LIBDIR}/yarp"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

# Compile the plugin by default
yarp_prepare_plugin(${ManusGloveDevName} TYPE wearable::devices::${ManusGloveDevName}
                    INCLUDE include/ManusGlove.h
                    CATEGORY device
                    ADVANCED
                    DEFAULT ON
                    )

yarp_add_plugin(${ManusGloveDevName}
    src/ManusGlove.cpp
    src/ManusGloveHelper.cpp
    include/ManusGlove.h
    include/ManusGloveHelper.h
    )

target_include_directories(${ManusGloveDevName} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(${ManusGloveDevName}
    PUBLIC
    YARP::YARP_dev
    IWear::IWear
    PRIVATE
    YARP::YARP_init
    ${ManusGloveLibName}
    Eigen3::Eigen
    )

yarp_install(TARGETS ${ManusGloveDevName}
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
             YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
             )

set (WEARABLES_XML_FILES conf/ManusGlove.xml
                         conf/ManusGloveRight.xml
                         conf/ManusGloveLeft.xml)
install(FILES ${WEARABLES_XML_FILES}
        DESTINATION ${CMAKE_INSTALL_DATADIR}/Wearables)
include(AddUninstallTarget)
