#/*
# Copyright 2021 Istituto Italiano di Tecnologia (IIT)
# @author Ehsan Ranjbari <ehsan.ranjbari@iit.it>
#* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#*/
cmake_minimum_required(VERSION 3.5)

project(YARPWeArtGlove VERSION "0.0.1"
                  LANGUAGES CXX C)

set(WeArtGloveLibName "WeArtGloveLib")
set(WeArtGloveDevName "WeArtGlove")

find_package(YARP REQUIRED)                
find_package(${WeArtGloveLibName} REQUIRED)
find_package(IWear REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(iDynTree REQUIRED)
find_package(BipedalLocomotionFramework 0.9.0
  COMPONENTS VectorsCollection IK ParametersHandlerYarpImplementation
             ContinuousDynamicalSystem ManifConversions
             ParametersHandlerYarpImplementation REQUIRED)
find_package(robometry)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

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
yarp_prepare_plugin(${WeArtGloveDevName} TYPE wearable::devices::${WeArtGloveDevName}
                    INCLUDE include/WeArtGlove.h
                    CATEGORY device
                    ADVANCED
                    DEFAULT ON
                    )

yarp_add_plugin(${WeArtGloveDevName}
    src/WeArtGlove.cpp
    include/WeArtGlove.h
    )

target_include_directories(${WeArtGloveDevName} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(${WeArtGloveDevName}
    PUBLIC
    YARP::YARP_dev
    IWear::IWear
    PRIVATE
    YARP::YARP_init
    ${WeArtGloveLibName}::${WeArtGloveLibName}
    Eigen3::Eigen
    ${iDynTree_LIBRARIES}
    BipedalLocomotion::IK
    BipedalLocomotion::ContinuousDynamicalSystem
    robometry::robometry
    )

yarp_install(TARGETS ${WeArtGloveDevName}
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
             YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
             )

set (WEARABLES_XML_FILES conf/WeArtGlove.xml
                         conf/WeArtGloveRight.xml
                         conf/WeArtGloveLeft.xml)
install(FILES ${WEARABLES_XML_FILES}
        DESTINATION ${CMAKE_INSTALL_DATADIR}/Wearables)

include(AddUninstallTarget)
