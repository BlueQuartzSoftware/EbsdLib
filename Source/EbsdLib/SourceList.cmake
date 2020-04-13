

set(EbsdLib_INSTALL_FILES 1)
set(EXE_DEBUG_EXTENSION _debug)
set(PROJECT_NAME EbsdLib)


set(EbsdLib_BUILT_AS_DYNAMIC_LIB)
if(BUILD_SHARED_LIBS)
	set(EbsdLib_BUILT_AS_DYNAMIC_LIB 1)
endif(BUILD_SHARED_LIBS)

include (${EbsdLibProj_SOURCE_DIR}/cmake/EbsdLibMacros.cmake)

#-------------------------------------------------------------------------------
# Core
#-------------------------------------------------------------------------------
include (${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/Core/SourceList.cmake)

#-------------------------------------------------------------------------------
# IO
#-------------------------------------------------------------------------------
include (${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/SourceList.cmake)

#-------------------------------------------------------------------------------
# LaueOps
#-------------------------------------------------------------------------------
include (${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/LaueOps/SourceList.cmake)

#-------------------------------------------------------------------------------
# Math
#-------------------------------------------------------------------------------
include (${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/Math/SourceList.cmake)

#-------------------------------------------------------------------------------
# OrientationMath
#-------------------------------------------------------------------------------
include (${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/OrientationMath/SourceList.cmake)

#-------------------------------------------------------------------------------
# Texture
#-------------------------------------------------------------------------------
include (${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/Texture/SourceList.cmake)

#-------------------------------------------------------------------------------
# Utilities
#-------------------------------------------------------------------------------
include (${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/Utilities/SourceList.cmake)


configure_file(${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/EbsdLibConfiguration.h.in
                ${EbsdLibProj_BINARY_DIR}/EbsdLib/EbsdLib.h
)

set(VERSION_GEN_NAMESPACE "EbsdLib")
set(VERSION_GEN_NAME "EBSDLIB")
set(VERSION_GEN_HEADER_FILE_NAME "EbsdLibVersion.h")
set(VERSION_GEN_VER_MAJOR ${EbsdLibProj_VERSION_MAJOR})
set(VERSION_GEN_VER_MINOR ${EbsdLibProj_VERSION_MINOR})
set(VERSION_GEN_VER_PATCH ${EbsdLibProj_VERSION_PATCH})
set(VERSION_GEN_VER_REVISION "0")
set(PROJECT_PREFIX "EbsdLib")
cmpGenerateBuildDate(PROJECT_NAME EbsdLibProj)
set(VERSION_BUILD_DATE ${EbsdLibProj_BUILD_DATE})

#------------------------------------------------------------------------------
# Find the Git Package for Versioning. It should be ok if Git is NOT found
Find_package(Git)
execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse --verify HEAD
    OUTPUT_VARIABLE GVS_GIT_HASH
    RESULT_VARIABLE did_run
    ERROR_VARIABLE git_error
    WORKING_DIRECTORY ${EbsdLibProj_SOURCE_DIR} 
)
string(REPLACE "\n" "" GVS_GIT_HASH "${GVS_GIT_HASH}")


configure_file(${EbsdLibProj_SOURCE_DIR}/cmake/cmpVersion.h.in
                ${EbsdLibProj_BINARY_DIR}/EbsdLib/EbsdLibVersion.h
)
configure_file(${EbsdLibProj_SOURCE_DIR}/cmake/cmpVersion.cpp.in
                ${EbsdLibProj_BINARY_DIR}/EbsdLib/EbsdLibVersion.cpp
)

set(EbsdLib_PROJECT_SRCS
  ${EbsdLib_Core_HDRS}
  ${EbsdLib_Core_SRCS}

  ${EbsdLib_IO_HDRS}
  ${EbsdLib_IO_SRCS}

  ${EbsdLib_LaueOps_HDRS}
  ${EbsdLib_LaueOps_SRCS}

  ${EbsdLib_Math_HDRS}
  ${EbsdLib_Math_SRCS}

  ${EbsdLib_OrientationMath_HDRS}
  ${EbsdLib_OrientationMath_SRCS}

  ${EbsdLib_Texture_HDRS}
  ${EbsdLib_Texture_SRCS}

  ${EbsdLib_Utilities_HDRS}
  ${EbsdLib_Utilities_SRCS}

  ${EbsdLibProj_BINARY_DIR}/EbsdLib/EbsdLibVersion.h
  ${EbsdLibProj_BINARY_DIR}/EbsdLib/EbsdLibVersion.cpp

  ${EbsdLibProj_BINARY_DIR}/EbsdLib/EbsdLib.h
)

if( ${EbsdLib_INSTALL_FILES} EQUAL 1 )
    INSTALL (FILES 
              ${EbsdLibProj_BINARY_DIR}/EbsdLib/EbsdLib.h
              ${EbsdLibProj_BINARY_DIR}/EbsdLib/EbsdLibVersion.h
            DESTINATION include/EbsdLib
            COMPONENT Headers   )
endif()


add_library(${PROJECT_NAME} ${LIB_TYPE} ${EbsdLib_PROJECT_SRCS})

# Start building up the list of libraries to link against
set(EBSDLib_LINK_LIBRARIES Qt5::Core)

#------------------------------------------------------------------------------
# Now add in the H5Support sources to the current target
if(EbsdLib_ENABLE_HDF5)
  target_sources(${PROJECT_NAME}  PRIVATE
    ${H5Support_SOURCE_DIR}/Source/H5Support/H5Lite.h       
    ${H5Support_SOURCE_DIR}/Source/H5Support/H5Macros.h   
    ${H5Support_SOURCE_DIR}/Source/H5Support/H5ScopedErrorHandler.h 
    ${H5Support_SOURCE_DIR}/Source/H5Support/H5ScopedSentinel.h   
    ${H5Support_SOURCE_DIR}/Source/H5Support/H5Support.h         
    ${H5Support_SOURCE_DIR}/Source/H5Support/H5SupportVersion.h   
    ${H5Support_SOURCE_DIR}/Source/H5Support/H5Utilities.h     
    ${H5Support_SOURCE_DIR}/Source/H5Support/QH5Lite.h          
    ${H5Support_SOURCE_DIR}/Source/H5Support/QH5Utilities.h
  )
   set(H5Support_USE_QT ON)
   set(EBSDLib_LINK_LIBRARIES  ${EBSDLib_LINK_LIBRARIES} hdf5::hdf5-shared)
   target_compile_definitions( ${PROJECT_NAME} PUBLIC -DH5Support_USE_QT)
endif()


CMP_AddDefinitions(TARGET ${PROJECT_NAME})

#-- Configure Target Specific Include Directories
get_filename_component(TARGET_BINARY_DIR_PARENT ${EbsdLibProj_BINARY_DIR} PATH)
get_filename_component(TARGET_SOURCE_DIR_PARENT ${EbsdLibProj_SOURCE_DIR} PATH)

target_include_directories(${PROJECT_NAME}
                            PUBLIC
                            $<BUILD_INTERFACE:${EbsdLibProj_SOURCE_DIR}/Source>
)
target_include_directories(${PROJECT_NAME}
                            PUBLIC
                            ${EIGEN3_INCLUDE_DIR}
                            ${Qt5Core_INCLUDE_DIRS}
                            ${Qt5Core_INCLUDE_DIR}
                            $<BUILD_INTERFACE:${EbsdLibProj_BINARY_DIR}>
)

if(EbsdLib_ENABLE_HDF5)
  target_include_directories(${PROJECT_NAME}
                              PUBLIC
                              ${HDF5_INCLUDE_DIRS}
                              ${HDF5_INCLUDE_DIR}
                              ${H5Support_SOURCE_DIR}/Source
                             )
endif()

if(WIN32 AND BUILD_SHARED_LIBS)
	target_compile_definitions(${PROJECT_NAME} PUBLIC "-DEbsdLib_BUILT_AS_DYNAMIC_LIB")
endif()
	
LibraryProperties( ${PROJECT_NAME} ${EXE_DEBUG_EXTENSION} )
target_link_libraries(${PROJECT_NAME} ${EBSDLib_LINK_LIBRARIES})

if(EbsdLib_USE_PARALLEL_ALGORITHMS)
  target_link_libraries(${PROJECT_NAME} TBB::tbb TBB::tbbmalloc)
endif()



set(install_dir "bin")
set(lib_install_dir "lib")
if(WIN32)
	set(install_dir "bin")
	set(lib_install_dir "lib")
endif()

INSTALL(TARGETS ${PROJECT_NAME}
  COMPONENT Applications
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION ${install_dir}
  LIBRARY DESTINATION ${lib_install_dir}
  ARCHIVE DESTINATION lib
  BUNDLE DESTINATION "."
)



# --------------------------------------------------------------------
# Allow the generation and installation of a CMake configuration file
# which makes using EBSDLib from another project easier.
# --------------------------------------------------------------------

# --------------------------------------------------------------------
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/EbsdLib/${PROJECT_NAME}TargetsConfigVersion.cmake"
  VERSION ${EbsdLib_VERSION}
  COMPATIBILITY AnyNewerVersion
)
#if(BUILD_SHARED_LIBS)
  export(EXPORT ${PROJECT_NAME}Targets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/EbsdLib/${PROJECT_NAME}Targets.cmake"
    NAMESPACE EbsdLib::
  )
#endif()

configure_file(${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/cmake/EbsdLibConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/EbsdLib/EbsdLibConfig.cmake"
  @ONLY
)

set(ConfigPackageLocation share/cmake/EbsdLib)

#if(BUILD_SHARED_LIBS)
  install(EXPORT ${PROJECT_NAME}Targets
    FILE
      ${PROJECT_NAME}Targets.cmake
    NAMESPACE
      EbsdLib::
    DESTINATION
      ${ConfigPackageLocation}
  )

#endif()

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/EbsdLib/EbsdLibConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/EbsdLib/${PROJECT_NAME}TargetsConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)





