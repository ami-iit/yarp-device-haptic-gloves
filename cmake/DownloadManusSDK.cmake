# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

include(FetchContent)

# Configuration for Manus SDK download
set(MANUS_SDK_DOWNLOAD_VERSION "3.1.1" CACHE STRING "Manus SDK version to download")
set(_manus_sdk_archive_name "MANUS_Core_${MANUS_SDK_DOWNLOAD_VERSION}_SDK.zip")

if(MANUS_SDK_DOWNLOAD_VERSION VERSION_LESS "3.0.0")
  set(_manus_sdk_url "https://static.manus-meta.com/resources/manus_core_2/sdk/${_manus_sdk_archive_name}")
else()
  set(_manus_sdk_url "https://static.manus-meta.com/resources/manus_core_3/sdk/${_manus_sdk_archive_name}")
endif()

# Fetch ManusSDK from the official Manus static server
FetchContent_Declare(
    ManusSDK
    URL ${_manus_sdk_url}
    SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/manussdk-src
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_MakeAvailable(ManusSDK)

# Determine the SDK client directory based on platform
set(_MANUS_SDK_EXTRACT_ROOT ${CMAKE_BINARY_DIR}/_deps/manussdk-src)

# Debug: List the contents of the extracted directory
if(NOT EXISTS "${_MANUS_SDK_EXTRACT_ROOT}")
  message(FATAL_ERROR "ManusSDK extraction directory not found: ${_MANUS_SDK_EXTRACT_ROOT}")
endif()

file(GLOB _manus_sdk_top_level RELATIVE "${_MANUS_SDK_EXTRACT_ROOT}" "${_MANUS_SDK_EXTRACT_ROOT}/*")
message(STATUS "ManusSDK extracted contents: ${_manus_sdk_top_level}")

# The archive structure does not have a versioned subdirectory, SDKClient_* are at the root
if(UNIX)
  set(_manus_sdk_client_dir "${_MANUS_SDK_EXTRACT_ROOT}/SDKClient_Linux")
else()
  set(_manus_sdk_client_dir "${_MANUS_SDK_EXTRACT_ROOT}/SDKClient_Windows")
endif()

# Verify the client directory exists
if(NOT EXISTS "${_manus_sdk_client_dir}")
  message(FATAL_ERROR "ManusSDK client directory not found at: ${_manus_sdk_client_dir}\nPlease verify the SDK version and archive structure.")
endif()

# Create an imported target for ManusSDK since it's a prebuilt binary SDK
add_library(ManusSDK::ManusSDK SHARED IMPORTED GLOBAL)

if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(_MANUS_SDK_ARCH x64)
    else()
        set(_MANUS_SDK_ARCH x86)
    endif()
    
    set(_MANUS_SDK_DLL "${_manus_sdk_client_dir}/ManusSDK/lib/ManusSDK.dll")
    set(_MANUS_SDK_LIB "${_manus_sdk_client_dir}/ManusSDK/lib/ManusSDK.lib")
    set(_MANUS_SDK_INCLUDE "${_manus_sdk_client_dir}/ManusSDK/include")
    
    # Verify the files exist
    if(NOT EXISTS "${_MANUS_SDK_DLL}")
        message(FATAL_ERROR "ManusSDK DLL not found at: ${_MANUS_SDK_DLL}")
    endif()
    if(NOT EXISTS "${_MANUS_SDK_LIB}")
        message(FATAL_ERROR "ManusSDK LIB not found at: ${_MANUS_SDK_LIB}")
    endif()
    if(NOT EXISTS "${_MANUS_SDK_INCLUDE}")
        message(FATAL_ERROR "ManusSDK include directory not found at: ${_MANUS_SDK_INCLUDE}")
    endif()
    
    set_target_properties(ManusSDK::ManusSDK PROPERTIES
        IMPORTED_LOCATION "${_MANUS_SDK_DLL}"
        IMPORTED_IMPLIB "${_MANUS_SDK_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${_MANUS_SDK_INCLUDE}"
        IMPORTED_LOCATION_RELEASE "${_MANUS_SDK_DLL}"
        IMPORTED_IMPLIB_RELEASE "${_MANUS_SDK_LIB}"
    )
    
    # For multi-config generators, also set Release-specific properties
    set_target_properties(ManusSDK::ManusSDK PROPERTIES
        MAP_IMPORTED_CONFIG_RELEASE Release
        MAP_IMPORTED_CONFIG_DEBUG Release
    )
    
elseif(APPLE)
    set(_MANUS_SDK_LIB "${_manus_sdk_client_dir}/ManusSDK/lib/libManusSDK.dylib")
    set(_MANUS_SDK_INCLUDE "${_manus_sdk_client_dir}/ManusSDK/include")
    
    if(NOT EXISTS "${_MANUS_SDK_LIB}")
        message(FATAL_ERROR "ManusSDK dylib not found at: ${_MANUS_SDK_LIB}")
    endif()
    if(NOT EXISTS "${_MANUS_SDK_INCLUDE}")
        message(FATAL_ERROR "ManusSDK include directory not found at: ${_MANUS_SDK_INCLUDE}")
    endif()
    
    set_target_properties(ManusSDK::ManusSDK PROPERTIES
        IMPORTED_LOCATION "${_MANUS_SDK_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${_MANUS_SDK_INCLUDE}"
        IMPORTED_LOCATION_RELEASE "${_MANUS_SDK_LIB}"
    )
    
elseif(UNIX)
    set(_MANUS_SDK_LIB "${_manus_sdk_client_dir}/ManusSDK/lib/libManusSDK.so")
    set(_MANUS_SDK_INCLUDE "${_manus_sdk_client_dir}/ManusSDK/include")
    
    if(NOT EXISTS "${_MANUS_SDK_LIB}")
        message(FATAL_ERROR "ManusSDK shared library not found at: ${_MANUS_SDK_LIB}")
    endif()
    if(NOT EXISTS "${_MANUS_SDK_INCLUDE}")
        message(FATAL_ERROR "ManusSDK include directory not found at: ${_MANUS_SDK_INCLUDE}")
    endif()
    
    set_target_properties(ManusSDK::ManusSDK PROPERTIES
        IMPORTED_LOCATION "${_MANUS_SDK_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${_MANUS_SDK_INCLUDE}"
        IMPORTED_LOCATION_RELEASE "${_MANUS_SDK_LIB}"
    )
endif()

# Store the SDK root for later use in installation
set(MANUS_SDK_ROOT "${_manus_sdk_client_dir}" CACHE INTERNAL "ManusSDK root directory")

# Install the ManusSDK library
if(WIN32)
    # On Windows, install the DLL to bin directory and the import library to lib directory
    get_target_property(_manus_sdk_dll_location ManusSDK::ManusSDK IMPORTED_LOCATION)
    if(_manus_sdk_dll_location)
        install(FILES "${_manus_sdk_dll_location}"
                DESTINATION ${CMAKE_INSTALL_BINDIR}
                COMPONENT runtime)
    endif()
    
    get_target_property(_manus_sdk_implib_location ManusSDK::ManusSDK IMPORTED_IMPLIB)
    if(_manus_sdk_implib_location)
        install(FILES "${_manus_sdk_implib_location}"
                DESTINATION ${CMAKE_INSTALL_LIBDIR}
                COMPONENT runtime)
    endif()
else()
    # On Unix-like systems, install the shared library to lib directory
    get_target_property(_manus_sdk_lib_location ManusSDK::ManusSDK IMPORTED_LOCATION)
    if(_manus_sdk_lib_location)
        install(FILES "${_manus_sdk_lib_location}"
                DESTINATION ${CMAKE_INSTALL_LIBDIR}
                COMPONENT runtime)
    endif()
endif()

message(STATUS "Downloaded Manus SDK ${MANUS_SDK_DOWNLOAD_VERSION} from ${_manus_sdk_url}")
message(STATUS "Using Manus SDK ${MANUS_SDK_DOWNLOAD_VERSION} from ${MANUS_SDK_ROOT}")
