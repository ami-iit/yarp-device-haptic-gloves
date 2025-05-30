# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

find_package(SenseGloveSDK REQUIRED)

# Compile the plugin by default
yarp_prepare_plugin(SenseGlove TYPE wearable::devices::HapticGlove
                    INCLUDE include/HapticGlove.h
                    CATEGORY device
                    ADVANCED
                    DEFAULT ON
                    )

yarp_add_plugin(SenseGlove
    src/HapticGlove.cpp
    src/SenseGloveHelper.cpp
    include/HapticGlove.h
    include/SenseGloveHelper.hpp
    )

target_include_directories(SenseGlove PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(SenseGlove
    PUBLIC
    YARP::YARP_dev
    IWear::IWear
    PRIVATE
    YARP::YARP_init
    SenseGloveSDK
    Eigen3::Eigen
    )

yarp_install(TARGETS SenseGlove
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
             YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
             )

set (MAIN_XML_FILES conf/SenseGlove.xml)

set (HANDS_XML_FILES conf/SenseGlove/Right.xml
                     conf/SenseGlove/Left.xml)

install(FILES ${MAIN_XML_FILES}
        DESTINATION ${CMAKE_INSTALL_DATADIR}/yarp)

install(FILES ${HANDS_XML_FILES}
        DESTINATION ${CMAKE_INSTALL_DATADIR}/yarp/SenseGlove)
