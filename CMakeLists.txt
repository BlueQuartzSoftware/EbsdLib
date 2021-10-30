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
project(EbsdLibProj VERSION 2.0.0)

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

if(EbsdLib_ENABLE_HDF5)

  find_package(HDF5 NAMES hdf5 REQUIRED)
  
  find_package(H5Support)

  # If H5Support already exists no need to look for it (e.g. EbsdLib is added as a subdirectory in DREAM3D)
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
    add_subdirectory("${H5Support_SOURCE_DIR}" "${EbsdLibProj_BINARY_DIR}/H5Support")
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
if(APPLE AND CMAKE_OSX_DEPLOYMENT_TARGET VERSION_LESS 10.15)
  set(EbsdLib_USE_GHC_FILESYSTEM ON)
elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" AND "${CMAKE_SYSTEM_NAME}" STREQUAL "Linux" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
  set(EbsdLib_USE_GHC_FILESYSTEM ON)
endif()

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
