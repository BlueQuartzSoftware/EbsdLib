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
	message(FATAL_ERROR "Qt5 is Required for EbsdLib to build. Please install it or set the Qt5_DIR cmake variable")
endif()

#-------------------------------------------------------------------------------
# Find the Eigen Library as we need that.
#-------------------------------------------------------------------------------
Find_Package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  message(FATAL_ERROR "Eigen 3 is Required for EbsdLib to build. Please install it or set the Eigen3_DIR variable")
endif()

message(STATUS "* Qt5 (${Qt5_VERSION}) ${Qt5_DIR}")
message(STATUS "* Eigen (${EIGEN3_VERSION_STRING}) ${Eigen3_DIR}")

include("${CMAKE_CURRENT_LIST_DIR}/EbsdLibTargets.cmake")

set(EbsdLib_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include")
set(EbsdLib_LIB_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../lib;${CMAKE_CURRENT_LIST_DIR}/../../../bin")
