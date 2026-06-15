# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

include(FindPackageHandleStandardArgs)

set(MANUS_ROOT_DIR "$ENV{ManusGlove_DIR}" CACHE PATH "Folder containing the ManusSDK")

find_path(MANUS_INCLUDE_DIR ManusSDK.h PATHS ${MANUS_ROOT_DIR} PATH_SUFFIXES ManusSDK/include NO_DEFAULT_PATH)
if(UNIX)
  set(_manus_lib_name ManusSDK_Integrated)
  find_library(MANUS_LIBRARY ${_manus_lib_name} PATHS ${MANUS_ROOT_DIR} PATH_SUFFIXES ManusSDK ManusSDK/lib NO_DEFAULT_PATH)
  find_package_handle_standard_args(ManusSDK DEFAULT_MSG MANUS_INCLUDE_DIR MANUS_LIBRARY)
else()
  # On Windows the import library (.lib) is used for linking; the runtime DLL
  # must be distributed separately and installed to the bin directory.
  find_library(MANUS_LIBRARY ManusSDK PATHS ${MANUS_ROOT_DIR} PATH_SUFFIXES ManusSDK ManusSDK/lib NO_DEFAULT_PATH)
  find_file(MANUS_RUNTIME_DLL ManusSDK.dll PATHS ${MANUS_ROOT_DIR} PATH_SUFFIXES ManusSDK ManusSDK/lib ManusSDK/bin NO_DEFAULT_PATH)
  find_package_handle_standard_args(ManusSDK DEFAULT_MSG MANUS_INCLUDE_DIR MANUS_LIBRARY MANUS_RUNTIME_DLL)
endif()

if(ManusSDK_FOUND)
    if(NOT TARGET ManusSDK::ManusSDK)
      add_library(ManusSDK::ManusSDK SHARED IMPORTED)
      set_target_properties(ManusSDK::ManusSDK PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${MANUS_INCLUDE_DIR}")
      if(UNIX)
        # The prebuilt Manus SDK shared library ships without a DT_SONAME. By
        # default CMake links imported libraries by full path, which makes the
        # linker bake that (build-relative) path into the consumer's DT_NEEDED
        # entry, breaking it at runtime. IMPORTED_NO_SONAME makes CMake link via
        # -L<dir> -l<name> so the linker records only the basename
        # (libManusSDK_Integrated.so), resolved at runtime through the
        # consumer's RPATH ($ORIGIN/../lib). Note: this property is only honored
        # for targets CMake treats as shared libraries, hence SHARED IMPORTED
        # (not UNKNOWN IMPORTED).
        set_target_properties(ManusSDK::ManusSDK PROPERTIES
          IMPORTED_LOCATION "${MANUS_LIBRARY}"
          IMPORTED_NO_SONAME TRUE)
      else()
        # On Windows, IMPORTED_LOCATION is the runtime DLL and IMPORTED_IMPLIB
        # is the import library used at link time.
        set_target_properties(ManusSDK::ManusSDK PROPERTIES
          IMPORTED_LOCATION "${MANUS_RUNTIME_DLL}"
          IMPORTED_IMPLIB   "${MANUS_LIBRARY}")
      endif()
    endif()
endif()
