
#-------------------------------------------------------------------------------
#
function(CMP_AddDefinitions)
  set(options)
  set(oneValueArgs TARGET)
  set(multiValueArgs)
  cmake_parse_arguments(Z "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

  # --------------------------------------------------------------------
  # Add in some compiler definitions
  # --------------------------------------------------------------------
  if( CMAKE_BUILD_TYPE MATCHES Debug )
    target_compile_definitions(${Z_TARGET} PRIVATE -DDEBUG)
  endif( CMAKE_BUILD_TYPE MATCHES Debug )

  # On linux we need to set this because some of the libraries are Static
  # and some are shared.
  if( CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" AND NOT MSVC )
    target_compile_options(${Z_TARGET} PRIVATE -fPIC)
  endif()

  # --------------------------------------------------------------------
  # If was are using GCC, make the compiler messages on a single line
  if(CMAKE_COMPILER_IS_GNUCC)
    target_compile_options(${Z_TARGET} PRIVATE -fmessage-length=0)
  endif(CMAKE_COMPILER_IS_GNUCC)
  if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(${Z_TARGET} PRIVATE -fmessage-length=0)
  endif(CMAKE_COMPILER_IS_GNUCXX)

  if(MSVC AND SIMPL_DISABLE_MSVC_WARNINGS)
    target_compile_definitions(${Z_TARGET} PRIVATE -D_CRT_SECURE_NO_WARNINGS)
    target_compile_definitions(${Z_TARGET} PRIVATE -D_SCL_SECURE_NO_WARNINGS)
  endif()

endfunction()


#-------------------------------------------------------------------------------
#
macro(LibraryProperties targetName DEBUG_EXTENSION)
    if( NOT BUILD_SHARED_LIBS AND MSVC)
      set_target_properties( ${targetName}
        PROPERTIES
        DEBUG_OUTPUT_NAME lib${targetName}
        RELEASE_OUTPUT_NAME lib${targetName}  )
    endif()

    set_target_properties( ${targetName} PROPERTIES FOLDER ${targetName}Proj)

    #-- Set the Debug and Release names for the libraries
    set_target_properties( ${targetName}
        PROPERTIES
        DEBUG_POSTFIX ${DEBUG_EXTENSION}
    )

    # enable per object parallel compilation in this large library
    if(MSVC)
        target_compile_options(${targetName} PRIVATE "/MP")
    endif()

    if(BUILD_SHARED_LIBS)
      if(APPLE)
        # use, i.e. don't skip the full RPATH for the build tree
        SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

        # when building, don't use the install RPATH already
        # (but later on when installing)
        SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

        SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

        # add the automatically determined parts of the RPATH
        # which point to directories outside the build tree to the install RPATH
        SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

      endif(APPLE)

      if(CMAKE_SYSTEM_NAME MATCHES "Linux")
        set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib")
        set_target_properties( ${targetName}
                    PROPERTIES
                    INSTALL_RPATH \$ORIGIN/../lib)
      endif()

   endif( BUILD_SHARED_LIBS)

endmacro(LibraryProperties DEBUG_EXTENSION)




#-------------------------------------------------------------------------------
# 
function(CMP_MODULE_INCLUDE_DIRS)
  set(options)
  set(oneValueArgs TARGET LIBVARS)
  set(multiValueArgs )
  cmake_parse_arguments(Z "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

  foreach(LIB ${Z_LIBVARS})
    target_include_directories(${Z_TARGET} PUBLIC ${${LIB}_INCLUDE_DIRS})
    target_include_directories(${Z_TARGET} PUBLIC ${${LIB}_INCLUDE_DIR})
  endforeach()


endfunction()


