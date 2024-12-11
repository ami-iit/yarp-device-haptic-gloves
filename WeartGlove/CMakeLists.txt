#/*
# Copyright 2021 Istituto Italiano di Tecnologia (IIT)
# @author Ehsan Ranjbari <ehsan.ranjbari@iit.it>
#* CopyPolicy: Released under the terms of the BSD-3-Clause
#*/

project(YARPWeArtGlove VERSION "0.0.1"
                  LANGUAGES CXX C)

set(WeArtGloveLibName "WeArtGloveLib")
set(WeArtGloveDevName "WeArtGlove")

find_package(${WeArtGloveLibName} REQUIRED)
find_package(iDynTree REQUIRED)
find_package(BipedalLocomotionFramework 0.9.0
  COMPONENTS VectorsCollection IK ParametersHandlerYarpImplementation
             ContinuousDynamicalSystem ManifConversions
             ParametersHandlerYarpImplementation REQUIRED)

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
option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

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
    )

yarp_install(TARGETS ${WeArtGloveDevName}
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
             YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
             )

set (MAIN_XML_FILES conf/WeartGlove.xml)

set (HANDS_XML_FILES conf/WeartGlove/Right.xml
                     conf/WeartGlove/Left.xml)

install(FILES ${MAIN_XML_FILES}
        DESTINATION ${CMAKE_INSTALL_DATADIR}/yarp)

install(FILES ${HANDS_XML_FILES}
        DESTINATION ${CMAKE_INSTALL_DATADIR}/yarp/WeartGlove)

