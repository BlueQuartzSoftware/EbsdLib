cmake_minimum_required(VERSION 3.26)

cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0054 NEW)
cmake_policy(SET CMP0079 NEW)
cmake_policy(SET CMP0077 NEW)
cmake_policy(SET CMP0080 NEW) # ``BundleUtilities`` cannot be included at configure time.

# resolves symlinks before collapsing ../ components.
if(CMAKE_VERSION VERSION_GREATER "3.28.0")
  cmake_policy(SET CMP0152 NEW)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/cmake/Utility.cmake)


# ------------------------------------------------------------------------------
# is building unit tests enabled
# ------------------------------------------------------------------------------
option(EbsdLib_BUILD_TESTS "Enable building tests" ON)
enable_vcpkg_manifest_feature(TEST_VAR EbsdLib_BUILD_TESTS FEATURE "tests")

# ------------------------------------------------------------------------------
# are multithreading algorithms enabled
# ------------------------------------------------------------------------------
option(EbsdLib_ENABLE_MULTICORE "Enable multicore support" ON)
enable_vcpkg_manifest_feature(TEST_VAR EbsdLib_ENABLE_MULTICORE FEATURE "parallel")


option(EbsdLib_DISABLE_MSVC_WARNINGS ON)
option(BUILD_SHARED_LIBS "Build Shared Libs" ON)

# Extra Variables that need to be set before all the configured files are generated.
option(EbsdLib_ENABLE_HDF5 "Enable HDF5 Support in the EbsdLibProj" ON)

option(EbsdLib_BUILD_H5SUPPORT "Build H5Support Library" OFF)


# set project's name
project(EbsdLibProj VERSION 3.1.1)


# Request C++17 standard, using new CMake variables.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#--------------------------------------------------------------------------------------------------
# Always write out the compile_commands.json file to help out things like QtCreator and VS Code
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------- Setup output Directories -------------------------
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
	${PROJECT_BINARY_DIR}/Bin
	CACHE PATH
	"Single Directory for all Libraries"
)

# --------- Setup the Executable output Directory -------------
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
	${PROJECT_BINARY_DIR}/Bin
	CACHE PATH
	"Single Directory for all Executables."
)

# --------- Setup the Executable output Directory -------------
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
	${PROJECT_BINARY_DIR}/Bin
	CACHE PATH
	"Single Directory for all static libraries."
)


get_filename_component(EbsdLib_PARENT ${EbsdLibProj_SOURCE_DIR} DIRECTORY CACHE)


#-------------------------------------------------------------------------------
# Find the HDF5 Library as we need that.
#-------------------------------------------------------------------------------
if(EbsdLib_ENABLE_HDF5)
  if(EbsdLib_BUILD_H5SUPPORT)
    if(NOT TARGET H5Support::H5Support)
      #------------------------------------------------------------------------------
      # If the developer has set another H5Support directory then use that, otherwise look
      # for the H5Support directory at the same level as the EBSDLib directory
      if("${H5Support_SOURCE_DIR}" STREQUAL "")
        if(EXISTS "${EbsdLib_PARENT}/H5Support")
          set(H5Support_SOURCE_DIR "${EbsdLib_PARENT}/H5Support")
        elseif(EXISTS "${EbsdLib_SOURCE_DIR}/H5Support")
          set(H5Support_SOURCE_DIR "${EbsdLib_SOURCE_DIR}/H5Support")
        else()
          message(FATAL_ERROR "H5Support_SOURCE_DIR was not set. Where is the H5Support project directory. Please set the H5Support_SOURCE_DIR variable to the CMP directory.")
        endif()
        message(STATUS "H5Support_SOURCE_DIR: ${H5Support_SOURCE_DIR}")
      endif()
      set(H5Support_INCLUDE_QT_API OFF)
      add_subdirectory("${H5Support_SOURCE_DIR}" "${EbsdLibProj_BINARY_DIR}/H5Support")
    endif()
  else()
    find_package(H5Support REQUIRED)
  endif()
endif()
mark_as_advanced(EbsdLib_ENABLE_HDF5)

#-------------------------------------------------------------------------------
# Find the Eigen Library as we need that.
#-------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
  message(STATUS "Eigen3 Location: ${EIGEN3_ROOT_DIR}")
  message(STATUS "Eigen3 Version: ${EIGEN3_VERSION_STRING}")
  set(EBSD_USE_EIGEN "1" CACHE INTERNAL "")
else()
  message(WARNING "The Eigen Library is required for some algorithms to execute. These algorithms will be disabled.")
endif()


# -----------------------------------------------------------------------
# Find FMT library
# -----------------------------------------------------------------------
find_package(fmt CONFIG REQUIRED)


#-------------------------------------------------------------------------------
# Determine if we need the ghcFilesystem library
#-------------------------------------------------------------------------------
set(EbsdLib_USE_GHC_FILESYSTEM OFF)

if(APPLE)
  execute_process(COMMAND uname -v OUTPUT_VARIABLE DARWIN_VERSION)
  string(REGEX MATCH "[0-9]+" DARWIN_VERSION ${DARWIN_VERSION})
  if(DARWIN_VERSION VERSION_LESS 19)
    message(STATUS "The current macOS System is too old to compile and will fail. Please set the EbsdLib_USE_GHC_FILESYSTEM=ON variable to allow Ebsdlib to compile")
  endif()
  message(STATUS "EbsdLib: DARWIN_VERSION ${DARWIN_VERSION}")
  message(STATUS "EbsdLib: CMAKE_OSX_DEPLOYMENT_TARGET: ${CMAKE_OSX_DEPLOYMENT_TARGET}")
  if(NOT "${CMAKE_OSX_DEPLOYMENT_TARGET}" STREQUAL "" )
    if(CMAKE_OSX_DEPLOYMENT_TARGET VERSION_LESS 10.15)
      message(STATUS "EbsdLib: CMAKE_OSX_DEPLOYMENT_TARGET less than 10.15. Enabling EbsdLib_USE_GHC_FILESYSTEM")
      set(EbsdLib_USE_GHC_FILESYSTEM ON)
    endif()
  endif()
endif()
if("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux" AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
  set(EbsdLib_USE_GHC_FILESYSTEM ON)
endif()


message(STATUS "EbsdLib: EbsdLib_USE_GHC_FILESYSTEM: ${EbsdLib_USE_GHC_FILESYSTEM}")

if(EbsdLib_USE_GHC_FILESYSTEM)
  find_package(ghcFilesystem REQUIRED NAMES ghc_filesystem ghcFilesystem)
endif()


# ------------------------------------------------------------------------------
# are multithreading algorithms enabled
# ------------------------------------------------------------------------------
if(EbsdLib_ENABLE_MULTICORE)  # If TBB already exists no need to look for it (e.g. EbsdLib is added as a subdirectory in DREAM3D)
  find_package(TBB CONFIG REQUIRED)
  get_target_property(tbb_dll_path TBB::tbb IMPORTED_LOCATION_RELEASE)
  get_filename_component(tbb_dll_path "${tbb_dll_path}" DIRECTORY)
  get_property(EbsdLib_EXTRA_LIBRARY_DIRS GLOBAL PROPERTY EbsdLib_EXTRA_LIBRARY_DIRS)
  set_property(GLOBAL PROPERTY EbsdLib_EXTRA_LIBRARY_DIRS ${EbsdLib_EXTRA_LIBRARY_DIRS} ${tbb_dll_path})
  set(EbsdLib_USE_PARALLEL_ALGORITHMS ON)
endif()

include(${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/SourceList.cmake)



# ------------------------------------------------------------------------------
# Check if a different Data_Archive web site is being used.
#   The alternate would be for the internal build bots that can download from a
#   local webserver
# ------------------------------------------------------------------------------
if("${DATA_ARCHIVE_WEB_SITE}" STREQUAL "")
  set(DATA_ARCHIVE_WEB_SITE "https://github.com/bluequartzsoftware/ebsdlib/releases/download/Data_Archive/")
endif()

file(TO_CMAKE_PATH "${EBSDLIB_DATA_DIR}" EBSDLIB_DATA_DIR_NORM)


# -----------------------------------------------------------------------
# This section creates a global file that holds the commands to copy data
# files, test data files, pipelines into a single cmake file that gets
# generated. This reduces the many copy targets to a single target.
# -----------------------------------------------------------------------
set_property(GLOBAL PROPERTY EBSD_FETCH_FILE_INDEX 0)
set(EBSD_FETCH_FILE_PATH "${EbsdLibProj_BINARY_DIR}/EbsdLibProj_fetch_remote_files.cmake")
set_property(GLOBAL PROPERTY EBSD_FETCH_FILE_PATH "${EBSD_FETCH_FILE_PATH}")
file(WRITE ${EBSD_FETCH_FILE_PATH} "# -----------------------------------------------------------------------
# This file is programmatically generated from the CMake file ${CMAKE_CURRENT_LIST_FILE}
# This file has the commands to download each of the test files.
# The WORKING_DIRECTORY is set to the following CMake code:
# ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>
# -----------------------------------------------------------------------\n
cmake_policy(SET CMP0012 NEW)
cmake_policy(SET CMP0054 NEW)
# The ``FindPythonInterp`` and ``FindPythonLibs`` modules are removed.
# if(CMAKE_VERSION VERSION_GREATER \"3.27.0\")
#   cmake_policy(SET CMP0148 NEW)
# endif()
# resolves symlinks before collapsing ../ components.
if(CMAKE_VERSION VERSION_GREATER \"3.28.0\")
  cmake_policy(SET CMP0152 NEW)
endif()
set(CONFIG \${CMAKE_CONFIG})
set(LOG_FILE \"${EbsdLibProj_BINARY_DIR}/EbsdLibProj_fetch_remote_files.log\")\n
FILE(WRITE \"\${LOG_FILE}\" \"Featch File Log Start\n\")\n
")

message(STATUS "EBSD_FETCH_FILE_PATH: ${EBSD_FETCH_FILE_PATH}")

set(TEST_WORKING_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
if(CMAKE_GENERATOR MATCHES "Visual Studio")
  set(TEST_WORKING_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>")
endif()

if(EBSDLIB_DOWNLOAD_TEST_FILES)
  add_custom_target(EbsdLib_Fetch_Remote_Data_Files ALL
    COMMAND "${CMAKE_COMMAND}" -DCMAKE_CONFIG=$<CONFIG> -P "${EBSD_FETCH_FILE_PATH}"
    COMMENT "Downloading EbsdLib Test Data Files"
  #  WORKING_DIRECTORY "${TEST_WORKING_DIR}"
  )
  set_target_properties(EbsdLib_Fetch_Remote_Data_Files PROPERTIES FOLDER ZZ_FETCH_TEST_FILES)
endif()


# ------------------------------------------------------------------------------
# Build Unit Test
# ------------------------------------------------------------------------------
if(EbsdLib_BUILD_TESTS)
  enable_testing()
	include(${EbsdLibProj_SOURCE_DIR}/Source/Test/CMakeLists.txt)
endif()

option(EbsdLib_BUILD_TOOLS "Build example applications" ON)
if(EbsdLib_BUILD_TOOLS)
  include(${EbsdLibProj_SOURCE_DIR}/Source/Apps/SourceList.cmake)
endif()


# -----------------------------------------------------------------------
# Include the packaging Cmake codes
# -----------------------------------------------------------------------
include(${EbsdLibProj_SOURCE_DIR}/cmake/Packaging.cmake)

