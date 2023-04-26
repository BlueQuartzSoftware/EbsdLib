cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0054 NEW)
cmake_policy(SET CMP0079 NEW)
cmake_policy(SET CMP0077 NEW)

cmake_minimum_required(VERSION 3.14.0)

# Request C++17 standard, using new CMake variables.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#--------------------------------------------------------------------------------------------------
# Always write out the compile_commands.json file to help out things like QtCreator and VS Code
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set project's name
project(EbsdLibProj VERSION 1.0.17)

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

option(EbsdLib_DISABLE_MSVC_WARNINGS ON)
option(BUILD_SHARED_LIBS "Build Shared Libs" ON)


get_filename_component(EbsdLib_PARENT ${EbsdLibProj_SOURCE_DIR} DIRECTORY CACHE)

#-------------------------------------------------------------------------------
# Find the HDF5 Library as we need that.
#-------------------------------------------------------------------------------
# Extra Variables that need to be set before all the configured files are generated.
option(EbsdLib_ENABLE_HDF5 "Enable HDF5 Support in the EbsdLibProj" ON)
option(EbsdLib_BUILD_H5SUPPORT "Build H5Support Library" ON)

if(EbsdLib_ENABLE_HDF5)
  if(EbsdLib_Anaconda_Install)
    set(HDF5_USE_STATIC_LIBRARIES OFF)
    set(HDF5_FIND_COMPONENTS OFF)
    find_package(HDF5 1.10.6 EXACT MODULE REQUIRED)
    set(HDF5_C_TARGET_NAME hdf5::hdf5)
  else()
    find_package(HDF5 NAMES hdf5 REQUIRED)
  endif()

  # If H5Support already exists no need to look for it (e.g. EbsdLib is added as a subdirectory in DREAM3D)
  if(NOT TARGET H5Support::H5Support AND EbsdLib_BUILD_H5SUPPORT)
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
    add_subdirectory("${H5Support_SOURCE_DIR}" "${EbsdLibProj_BINARY_DIR}/H5Support")
    else()
      find_package(H5Support NAMES H5Support)
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

#-------------------------------------------------------------------------------
# Determine if we need the ghcFilesystem library
#-------------------------------------------------------------------------------
set(EbsdLib_USE_GHC_FILESYSTEM OFF)

if(APPLE)
  exec_program(uname ARGS -v  OUTPUT_VARIABLE DARWIN_VERSION)
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

#-------------------------------------------------------------------------------
# Are we using Parallel Algorithms.
#-------------------------------------------------------------------------------
option(EbsdLib_USE_PARALLEL_ALGORITHMS "Enable EBSDLib to use parallel algorithms" ON)
if(EbsdLib_USE_PARALLEL_ALGORITHMS)  # If TBB already exists no need to look for it (e.g. EbsdLib is added as a subdirectory in DREAM3D)
  if(NOT TARGET TBB::tbb)
    include(${EbsdLibProj_SOURCE_DIR}/cmake/TBBSupport.cmake)
  endif()
endif()

include(${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/SourceList.cmake)

option(EbsdLib_ENABLE_TESTING "Enable the unit test" ON)
if(EbsdLib_ENABLE_TESTING)
	include(${EbsdLibProj_SOURCE_DIR}/Source/Test/CMakeLists.txt)
endif()

option(EbsdLib_BUILD_TOOLS "Build example applications" ON)
if(EbsdLib_BUILD_TOOLS)
  include(${EbsdLibProj_SOURCE_DIR}/Source/Apps/SourceList.cmake)
endif()
