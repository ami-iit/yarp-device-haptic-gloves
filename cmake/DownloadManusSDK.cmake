# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

set(_manus_sdk_version "3.1.1")
set(_manus_sdk_archive_name "MANUS_Core_${_manus_sdk_version}_SDK.zip")
set(_manus_sdk_url "https://static.manus-meta.com/resources/manus_core_3/sdk/${_manus_sdk_archive_name}")

set(_manus_sdk_base_dir "${CMAKE_BINARY_DIR}")
set(_manus_sdk_zip_path "${_manus_sdk_base_dir}/${_manus_sdk_archive_name}")
set(_manus_sdk_extract_dir "${_manus_sdk_base_dir}")

if(UNIX)
  set(_manus_sdk_client_dir "${_manus_sdk_extract_dir}/ManusSDK_v${_manus_sdk_version}/SDKClient_Linux")
else()
  set(_manus_sdk_client_dir "${_manus_sdk_extract_dir}/ManusSDK_v${_manus_sdk_version}/SDKClient_Windows")
endif()

# Header location is used as extraction marker so repeated configure runs are idempotent.
set(_manus_sdk_header_marker "${_manus_sdk_client_dir}/ManusSDK/include/ManusSDK.h")

file(MAKE_DIRECTORY "${_manus_sdk_base_dir}")

if(NOT EXISTS "${_manus_sdk_header_marker}")
  message(STATUS "Downloading Manus SDK ${_manus_sdk_version} from ${_manus_sdk_url}")
  file(DOWNLOAD
    "${_manus_sdk_url}"
    "${_manus_sdk_zip_path}"
    SHOW_PROGRESS
    STATUS _manus_sdk_download_status
  )

  list(GET _manus_sdk_download_status 0 _manus_sdk_download_code)
  if(NOT _manus_sdk_download_code EQUAL 0)
    list(GET _manus_sdk_download_status 1 _manus_sdk_download_message)
    message(FATAL_ERROR "Failed to download Manus SDK: ${_manus_sdk_download_message}")
  endif()

  message(STATUS "Extracting Manus SDK archive ${_manus_sdk_zip_path}")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xf "${_manus_sdk_zip_path}"
    WORKING_DIRECTORY "${_manus_sdk_base_dir}"
    RESULT_VARIABLE _manus_sdk_extract_result
  )

  if(NOT _manus_sdk_extract_result EQUAL 0)
    message(FATAL_ERROR "Failed to extract Manus SDK archive: ${_manus_sdk_zip_path}")
  endif()

  file(REMOVE "${_manus_sdk_zip_path}")
endif()

set(MANUS_ROOT_DIR
    "${_manus_sdk_client_dir}"
    CACHE PATH "Folder containing the ManusSDK"
    FORCE)

message(STATUS "Using Manus SDK from ${MANUS_ROOT_DIR}")
