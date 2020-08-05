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
	#------------------------------------------------------------------------------
	# If the developer has set another H5Support directory then use that, otherwise look
	# for the H5Support directory at the same level as the EBSDLib directory
	if("${H5Support_SOURCE_DIR}" STREQUAL "")
		if(EXISTS "${EbsdLib_PARENT}/H5Support")
			set(H5Support_SOURCE_DIR "${EbsdLib_PARENT}/H5Support")
		else()
			message(FATAL_ERROR "H5Support_SOURCE_DIR was not set. Where is the H5Support project directory. Please set the H5Support_SOURCE_DIR variable to the CMP directory.")
		endif()
		message(STATUS "H5Support_SOURCE_DIR: ${H5Support_SOURCE_DIR}")
	endif()
	# This will find the HDF5 Libraries
	include(${EbsdLibProj_SOURCE_DIR}/cmake/HDF5Support.cmake)
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

set(EbsdLib_USE_GHC_FILESYSTEM 0)
if(APPLE)
	find_package(ghcFilesystem REQUIRED)
	set(EbsdLib_USE_GHC_FILESYSTEM 1)
elseif(CMAKE_COMPILER_IS_GNUCC AND "${CMAKE_SYSTEM_NAME}" STREQUAL "Linux" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
	find_package(ghcFilesystem REQUIRED)
	set(EbsdLib_USE_GHC_FILESYSTEM 1)
endif()

option(EbsdLib_USE_PARALLEL_ALGORITHMS "Enable EBSDLib to use parallel algorithms" ON)
if(EbsdLib_USE_PARALLEL_ALGORITHMS)
  include(${EbsdLibProj_SOURCE_DIR}/cmake/TBBSupport.cmake)
endif()

include(${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/SourceList.cmake)

option(EbsdLib_ENABLE_TESTING "Enable the unit test" ON)
if(EbsdLib_ENABLE_TESTING)
	include(${EbsdLibProj_SOURCE_DIR}/Source/Test/CMakeLists.txt)
endif()

include(${EbsdLibProj_SOURCE_DIR}/Source/Apps/SourceList.cmake)
