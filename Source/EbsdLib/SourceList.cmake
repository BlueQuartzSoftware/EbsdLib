

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

  ${EbsdLibProj_BINARY_DIR}/EbsdLib/EbsdLib.h
)

if( ${EbsdLib_INSTALL_FILES} EQUAL 1 )
    INSTALL (FILES 
              ${EbsdLibProj_BINARY_DIR}/EbsdLib/EbsdLib.h
            DESTINATION include/EbsdLib
            COMPONENT Headers   )
endif()


add_library(${PROJECT_NAME} ${LIB_TYPE} ${EbsdLib_PROJECT_SRCS})

CMP_AddDefinitions(TARGET ${PROJECT_NAME})

#-- Configure Target Specific Include Directories
get_filename_component(TARGET_BINARY_DIR_PARENT ${EbsdLibProj_BINARY_DIR} PATH)
get_filename_component(TARGET_SOURCE_DIR_PARENT ${EbsdLibProj_SOURCE_DIR} PATH)

target_include_directories(${PROJECT_NAME}
                          PRIVATE
                            ${EbsdLibProj_SOURCE_DIR}/Source
                            ${EbsdLibProj_BINARY_DIR}
                          PUBLIC 
                            ${EIGEN3_INCLUDE_DIR}
                            ${HDF5_INCLUDE_DIRS}
                            ${HDF5_INCLUDE_DIR}
                            ${Qt5Core_INCLUDE_DIRS}
                            ${Qt5Core_INCLUDE_DIR}
)

# Start building up the list of libraries to link against
set(EBSDLib_LINK_LIBRARIES Qt5::Core)

# If we compiled against HDF5, we need that on the list of link libs
if(${EbsdLib_ENABLE_HDF5})
	set(EBSDLib_LINK_LIBRARIES
		${EBSDLib_LINK_LIBRARIES}
      hdf5::hdf5 H5Support
		)
  target_compile_definitions(${PROJECT_NAME} PRIVATE -DEbsdLib_HAVE_HDF5)
endif()


if(WIN32 AND BUILD_SHARED_LIBS)
	target_compile_definitions(${PROJECT_NAME} PUBLIC "-DEbsdLib_BUILT_AS_DYNAMIC_LIB")
endif()
	
LibraryProperties( ${PROJECT_NAME} ${EXE_DEBUG_EXTENSION} )
target_link_libraries(${PROJECT_NAME} ${EBSDLib_LINK_LIBRARIES})


set(install_dir "bin")
set(lib_install_dir "lib")
if(WIN32)
	set(install_dir ".")
	set(lib_install_dir "")
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






