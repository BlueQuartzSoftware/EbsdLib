# -------------------------------------------------------------
# This function adds the necessary cmake code to find the HDF5
# shared libraries and setup custom copy commands for Linux and Windows to use
function(AddHDF5CopyRules)
  set(options )
  set(oneValueArgs LIBNAME LIBVAR)
  set(multiValueArgs TYPES)
  cmake_parse_arguments(Z "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
  set(INTER_DIR ".")

  # message(STATUS "Z_LIBVAR: ${Z_LIBVAR}")
  # message(STATUS "Z_LIBNAME: ${Z_LIBNAME}")
  # message(STATUS "Z_TYPES: ${Z_TYPES}")

  foreach(BTYPE ${Z_TYPES} )
    #message(STATUS "BTYPE: ${BTYPE}")
    string(TOUPPER ${BTYPE} TYPE)
    if(MSVC_IDE)
      set(INTER_DIR "${BTYPE}")
    endif()

    # Get the Actual Library Path and create Install and copy rules
    get_target_property(LibPath ${Z_LIBNAME} IMPORTED_LOCATION_${TYPE})
    # message(STATUS "LibPath: ${LibPath}")
    if(NOT "${LibPath}" STREQUAL "LibPath-NOTFOUND")
      # message(STATUS "Creating Copy Command for ${LibPath}")
      if(NOT TARGET ZZ_${Z_LIBVAR}_DLL_${TYPE}-Copy)
        add_custom_target(ZZ_${Z_LIBVAR}_DLL_${TYPE}-Copy ALL
                            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LibPath}
                            ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${INTER_DIR}/
                            # COMMENT "  Copy: ${LibPath} To: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${INTER_DIR}/"
        )
        set_target_properties(ZZ_${Z_LIBVAR}_DLL_${TYPE}-Copy PROPERTIES FOLDER ZZ_COPY_FILES/${BTYPE}/HDF5)
        get_property(COPY_LIBRARY_TARGETS GLOBAL PROPERTY COPY_LIBRARY_TARGETS)
        set_property(GLOBAL PROPERTY COPY_LIBRARY_TARGETS ${COPY_LIBRARY_TARGETS} ZZ_${Z_LIBVAR}_DLL_${TYPE}-Copy)
      endif()

      # Now get the path that the library is in
      get_filename_component(${Z_LIBVAR}_DIR ${LibPath} PATH)
      # message(STATUS "${Z_LIBVAR}_DIR: ${${Z_LIBVAR}_DIR}")

      # Now piece together a complete path for the symlink that Linux Needs to have
      if(WIN32)
        get_target_property(${Z_LIBVAR}_${TYPE} ${Z_LIBNAME} IMPORTED_IMPLIB_${TYPE})
      else()
        get_target_property(${Z_LIBVAR}_${TYPE} ${Z_LIBNAME} IMPORTED_SONAME_${TYPE})
      endif()

      # message(STATUS "${Z_LIBVAR}_${TYPE}: ${${Z_LIBVAR}_${TYPE}}")
      if(NOT "${${Z_LIBVAR}_${TYPE}}" STREQUAL "${Z_LIBVAR}_${TYPE}-NOTFOUND" AND NOT WIN32)
        set(SYMLINK_PATH "${${Z_LIBVAR}_DIR}/${${Z_LIBVAR}_${TYPE}}")
        # message(STATUS "Creating Copy Command for ${SYMLINK_PATH}")
        if(NOT TARGET ZZ_${Z_LIBVAR}_SYMLINK_${TYPE}-Copy)
          add_custom_target(ZZ_${Z_LIBVAR}_SYMLINK_${TYPE}-Copy ALL
                              COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SYMLINK_PATH}
                              ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${INTER_DIR}/
                              # COMMENT "  Copy: ${SYMLINK_PATH} To: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${INTER_DIR}/"
          )
          set_target_properties(ZZ_${Z_LIBVAR}_SYMLINK_${TYPE}-Copy PROPERTIES FOLDER ZZ_COPY_FILES/${BTYPE}/HDF5)
          get_property(COPY_LIBRARY_TARGETS GLOBAL PROPERTY COPY_LIBRARY_TARGETS)
          set_property(GLOBAL PROPERTY COPY_LIBRARY_TARGETS ${COPY_LIBRARY_TARGETS} ZZ_${Z_LIBVAR}_SYMLINK_${TYPE}-Copy)
        endif()
      endif()
    endif()
  endforeach()
endfunction()

# -------------------------------------------------------------
# This function adds the necessary cmake code to find the HDF5
# shared libraries and setup install rules for Linux and Windows to use
function(AddHDF5InstallRules)
  set(options )
  set(oneValueArgs LIBNAME LIBVAR)
  set(multiValueArgs TYPES)
  cmake_parse_arguments(Z "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
  set(INTER_DIR ".")

  # message(STATUS "Z_LIBVAR: ${Z_LIBVAR}")
  # message(STATUS "Z_LIBNAME: ${Z_LIBNAME}")
  # message(STATUS "Z_TYPES: ${Z_TYPES}")

  set(Z_INSTALL_DIR "lib")
  if(WIN32)
    set(Z_INSTALL_DIR ".")
  endif()

  foreach(BTYPE ${Z_TYPES} )
    #message(STATUS "BTYPE: ${BTYPE}")
    string(TOUPPER ${BTYPE} TYPE)
    if(MSVC_IDE)
      set(INTER_DIR "${BTYPE}")
    endif()

    # Get the Actual Library Path and create Install and copy rules
    get_target_property(LibPath ${Z_LIBNAME} IMPORTED_LOCATION_${TYPE})
    # message(STATUS "LibPath: ${LibPath}")
    if(NOT "${LibPath}" STREQUAL "LibPath-NOTFOUND")
      # message(STATUS "Creating Install Rule for ${LibPath}")
      install(FILES ${LibPath} DESTINATION "${Z_INSTALL_DIR}" CONFIGURATIONS ${BTYPE} COMPONENT Applications)
    endif()

    # Now get the path that the library is in
    get_filename_component(${Z_LIBVAR}_DIR ${LibPath} PATH)
    # message(STATUS "${Z_LIBVAR}_DIR: ${${Z_LIBVAR}_DIR}")

    # Now piece together a complete path for the symlink that Linux Needs to have
    if(WIN32)
      get_target_property(${Z_LIBVAR}_${TYPE} ${Z_LIBNAME} IMPORTED_IMPLIB_${TYPE})
    else()
      get_target_property(${Z_LIBVAR}_${TYPE} ${Z_LIBNAME} IMPORTED_SONAME_${TYPE})
    endif()

    # message(STATUS "${Z_LIBVAR}_${TYPE}: ${${Z_LIBVAR}_${TYPE}}")
    if(NOT "${${Z_LIBVAR}_${TYPE}}" STREQUAL "${Z_LIBVAR}_${TYPE}-NOTFOUND" AND NOT WIN32)
      set(SYMLINK_PATH "${${Z_LIBVAR}_DIR}/${${Z_LIBVAR}_${TYPE}}")
      # message(STATUS "Creating Install Rule for ${SYMLINK_PATH}")
      install(FILES ${SYMLINK_PATH} DESTINATION "${Z_INSTALL_DIR}" CONFIGURATIONS ${BTYPE} COMPONENT Applications)
    endif()
  endforeach()
endfunction()


#------------------------------------------------------------------------------
# Find HDF5 Headers/Libraries
# HDF5 now comes with everything that is needed for CMake to load
# up the targets (Exported) that it needs. We just need to find where HDF5 is installed.
#------------------------------------------------------------------------------
if("${HDF5_INSTALL}" STREQUAL "")
    set(HDF5_INSTALL  $ENV{HDF5_INSTALL})
endif()

#message(STATUS "HDF5_DIR: ${HDF5_DIR}")

set(hdf5_path_suffixes "")
if(WIN32)
  set(hdf5_path_suffixes hdf5)
endif()

if(NOT DEFINED CMP_HDF5_USE_CONFIG)
  set(CMP_HDF5_USE_CONFIG ON)
endif()

if(CMP_HDF5_USE_CONFIG)
  find_package(HDF5 NAMES hdf5 REQUIRED PATH_SUFFIXES ${hdf5_path_suffixes})
else()
  if(${CMAKE_VERSION} VERSION_LESS "3.19.0") 
    message(FATAL_ERROR "CMake 3.19 required for HDF5 targets to be created in MODULE mode")
  endif()
  find_package(HDF5 MODULE REQUIRED)
endif()

if(HDF5_ENABLE_THREADSAFE)
  find_package(Threads REQUIRED)
endif()

if(NOT HDF5_FOUND)
  message(FATAL_ERROR "HDF5 was not found on your system. Please follow any instructions given to fix the problem")
endif()

if(HDF5_FOUND)
  # Add the library directory to the file that has all the search directories stored in it.
  get_property(HDF5_STATUS_PRINTED GLOBAL PROPERTY HDF5_STATUS_PRINTED)
  if(NOT HDF5_STATUS_PRINTED)
    message(STATUS "HDF5 Location: ${HDF5_INSTALL}")
    message(STATUS "HDF5 Version: ${HDF5_VERSION_STRING}")
    set_property(GLOBAL PROPERTY HDF5_STATUS_PRINTED TRUE)

    get_filename_component(HDF5_LIBRARY_DIRS "${HDF5_INCLUDE_DIR}" PATH)
    set(HDF5_LIBRARY_DIRS ${HDF5_LIBRARY_DIRS}/lib)
    file(APPEND ${CMP_PLUGIN_SEARCHDIR_FILE} "${HDF5_LIBRARY_DIRS};")
  endif()

  if(MSVC_IDE)
    set(BUILD_TYPES Debug Release)
  else()
    set(BUILD_TYPES "${CMAKE_BUILD_TYPE}")
    if("${BUILD_TYPES}" STREQUAL "")
        set(BUILD_TYPES "Debug")
    endif()
  endif()

  if(CMP_HDF5_USE_CONFIG)
    if(TARGET hdf5::hdf5-shared) # 1.8.17 and above
      set(HDF5_C_TARGET_NAME hdf5::hdf5-shared)
    else()
      message(FATAL_ERROR "Neither target hdf5, hdf5-shared nor hdf5::hdf5-shared was found.")
    endif()

    if(TARGET hdf5::hdf5_cpp-shared) # 1.8.17 and above
      set(HDF5_CXX_TARGET_NAME hdf5::hdf5_cpp-shared)
    else()
      message(FATAL_ERROR "Neither target hdf5_cpp, hdf5_cpp-shared nor hdf5::hdf5_cpp-shared was found.")
    endif()
  else()
    set(HDF5_C_TARGET_NAME hdf5::hdf5)
    set(HDF5_CXX_TARGET_NAME hdf5::hdf5_cpp)
  endif()

  if(NOT DEFINED CMP_HDF5_ENABLE_INSTALL)
    set(CMP_HDF5_ENABLE_INSTALL ON)
  endif()

  if(NOT DEFINED CMP_HDF5_ENABLE_COPY)
    set(CMP_HDF5_ENABLE_COPY ON)
  endif()

  if(NOT APPLE)
    if(CMP_HDF5_ENABLE_COPY)
      AddHDF5CopyRules(LIBVAR HDF5_LIB
        LIBNAME ${HDF5_C_TARGET_NAME}
        TYPES ${BUILD_TYPES}
      )
      AddHDF5CopyRules(LIBVAR HDF5_CPP_LIB
        LIBNAME ${HDF5_CXX_TARGET_NAME}
        TYPES ${BUILD_TYPES}
      )
    endif()

    if(CMP_HDF5_ENABLE_INSTALL)
      AddHDF5InstallRules(LIBVAR HDF5_LIB
        LIBNAME ${HDF5_C_TARGET_NAME}
        TYPES ${BUILD_TYPES}
      )
      AddHDF5InstallRules(LIBVAR HDF5_CPP_LIB
        LIBNAME ${HDF5_CXX_TARGET_NAME}
        TYPES ${BUILD_TYPES}
      )
    endif()
  endif()

  # The next CMake variable is needed for Linux to properly generate a shell script
  # that will properly install the HDF5 files.
  if(NOT APPLE AND NOT WIN32 AND NOT DREAM3D_ANACONDA)
    string(TOUPPER "${CMAKE_BUILD_TYPE}" TYPE)
    get_target_property(HDF5_C_LIB_PATH ${HDF5_C_TARGET_NAME} IMPORTED_LOCATION_${TYPE})
    get_target_property(HDF5_CXX_LIB_PATH ${HDF5_CXX_TARGET_NAME} IMPORTED_LOCATION_${TYPE})
    set(HDF5_COMPONENTS ${HDF5_C_LIB_PATH} ${HDF5_CXX_LIB_PATH})
  endif()
else(HDF5_FOUND)
    message(FATAL_ERROR "Cannot build without HDF5.  Please set HDF5_DIR environment variable to point to your HDF5 installation.")
endif(HDF5_FOUND)
