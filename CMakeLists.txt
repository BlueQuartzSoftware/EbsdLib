
cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0054 NEW)
cmake_policy(SET CMP0079 NEW)
cmake_policy(SET CMP0077 NEW)

cmake_minimum_required(VERSION 3.14.0)

# Request C++14 standard, using new CMake variables.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS False)

#--------------------------------------------------------------------------------------------------
# Always write out the compile_commands.json file to help out things like QtCreator and VS Code
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set project's name
PROJECT( EbsdLibProj VERSION 2.0.0)

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

#-------------------------------------------------------------------------------
# Find the Qt5 Library as we need that.
#-------------------------------------------------------------------------------
set(Qt5_COMPONENTS "Core")
# On Linux we need the DBus library
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
	set(Qt5_COMPONENTS ${Qt5_COMPONENTS} DBus)
endif()

find_package(Qt5 COMPONENTS ${Qt5_COMPONENTS})
if(NOT Qt5_FOUND)
	message(FATAL_ERROR "Qt5 is Required for ${PROJECT_NAME} to build. Please install it.")
endif()

# We need the location of QMake for later on in order to find the plugins directory
get_target_property(QtQMake_location Qt5::qmake LOCATION)
get_property(Qt5_STATUS_PRINTED GLOBAL PROPERTY Qt5_STATUS_PRINTED)
execute_process(COMMAND "${QtQMake_location}" -query QT_INSTALL_PREFIX OUTPUT_VARIABLE QM_QT_INSTALL_PREFIX OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND "${QtQMake_location}" -query QT_VERSION OUTPUT_VARIABLE QM_QT_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)

#-------------------------------------------------------------------------------
# Find the HDF5 Library as we need that.
#-------------------------------------------------------------------------------
# Extra Variables that need to be set before all the configured files are generated.
option(EbsdLib_ENABLE_HDF5 "Enable HDF5 Support in the EbsdLibProj" ON)
if(EbsdLib_ENABLE_HDF5)

	include(${EbsdLibProj_SOURCE_DIR}/Source/H5Support/Support/CMP/ExtLib/HDF5Support.cmake)
	

	set(H5Support_INCLUDE_QT_API ON)
	add_subdirectory(${EbsdLibProj_SOURCE_DIR}/Source/H5Support ${EbsdLibProj_BINARY_DIR}/H5Support)

endif()
mark_as_advanced(EbsdLib_ENABLE_HDF5)

#-------------------------------------------------------------------------------
# Find the Eigen Library as we need that.
#-------------------------------------------------------------------------------
Find_Package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
  message(STATUS "Eigen3 Location: ${EIGEN3_ROOT_DIR}")
  message(STATUS "Eigen3 Version: ${EIGEN3_VERSION_STRING}")
  set(EBSD_USE_EIGEN "1" CACHE INTERNAL "")
else()
  message(WARNING "The Eigen Library is required for some algorithms to execute. These algorithms will be disabled.")
endif()



include (${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/SourceList.cmake)


option(EbsdLib_ENABLE_TESTING "Enable the unit test" ON)
if(EbsdLib_ENABLE_TESTING)
	include(${EbsdLibProj_SOURCE_DIR}/Source/Test/CMakeLists.txt)
endif()
