#-------------------------------------------------------------------------------
#
#-------------------------------------------------------------------------------
function(complex_enable_warnings)
  set(optionsArgs)
  set(oneValueArgs TARGET)
  set(multiValueArgs)
  cmake_parse_arguments(ARG "${optionsArgs}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(NOT TARGET ${ARG_TARGET})
    message(FATAL_ERROR "complex_enable_warnings must be called with the argument TARGET set to a valid target")
  endif()

  if(MSVC)
    target_compile_options(${ARG_TARGET}
      PRIVATE
      # Suppressed warnings
      /wd4275 # C4275: An exported class was derived from a class that wasn't exported.
      /wd4251 # C4251: 'type' : class 'type1' needs to have dll-interface to be used by clients of class 'type2'

      # Warning to error
      /we4706 # C4706: assignment within conditional expression
      /we4715 # C4715: The specified function can potentially not return a value.
      /we4456 # C4456: declaration of 'identifier' hides previous local declaration
      /we4457 # C4457: declaration of 'identifier' hides function parameter
      /we4458 # C4458: declaration of 'identifier' hides class member
      /we4459 # C4459: declaration of 'identifier' hides global declaration
      )
  else()
    target_compile_options(${ARG_TARGET}
      PRIVATE
      # Warning to error
      -Werror=parentheses # Wparentheses: Warn if parentheses are omitted in certain contexts, such as when there is an assignment in a context where a truth value is expected, or when operators are nested whose precedence people often get confused about
      -Werror=return-type # Wreturn-type: Warn about any "return" statement with no return value in a function whose return type is not "void"
      -Werror=shadow # Wshadow: Warn whenever a local variable or type declaration shadows another variable, parameter, type, class member (in C++), or instance variable (in Objective-C) or whenever a built-in function is shadowed.
      )
  endif()
endfunction()

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

  if(MSVC AND EBSD_DISABLE_MSVC_WARNINGS)
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




#-------------------------------------------------------------------------------
# This function will attempt to generate a build date/time string.
#
#-------------------------------------------------------------------------------
function(cmpGenerateBuildDate)
  set(oneValueArgs PROJECT_NAME )
  cmake_parse_arguments(GVS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

  IF (WIN32)
      EXECUTE_PROCESS(COMMAND "cmd" " /C date /T" OUTPUT_VARIABLE RESULT)
      string(REPLACE  " " ";" RESULT ${RESULT})
      list(GET RESULT 1 RESULT)
      string(REPLACE "/" ";" RESULT ${RESULT})
      list(LENGTH RESULT LIST_LENGTH)
      if(LIST_LENGTH GREATER 2)
        list(GET RESULT 2 YEAR)
        list(GET RESULT 0 MONTH)
        list(GET RESULT 1 DAY)
        set(${GVS_PROJECT_NAME}_BUILD_DATE "${YEAR}/${MONTH}/${DAY}" PARENT_SCOPE)
      else()
        set(${GVS_PROJECT_NAME}_BUILD_DATE "0000/00/00" PARENT_SCOPE)
      endif()
      #message(STATUS "${GVS_PROJECT_NAME}_BUILD_DATE: ${${GVS_PROJECT_NAME}_BUILD_DATE}")
  ELSEIF(UNIX)
      EXECUTE_PROCESS(COMMAND "date" "+%Y/%m/%d/" OUTPUT_VARIABLE RESULT)
      string(REPLACE "/" ";" RESULT ${RESULT})
      list(LENGTH RESULT LIST_LENGTH)
      if(LIST_LENGTH GREATER 2)
        list(GET RESULT 0 YEAR)
        list(GET RESULT 1 MONTH)
        list(GET RESULT 2 DAY)
        set(${GVS_PROJECT_NAME}_BUILD_DATE "${YEAR}/${MONTH}/${DAY}" PARENT_SCOPE)
      else()
          set(${GVS_PROJECT_NAME}_BUILD_DATE "0000/00/00" PARENT_SCOPE)
      endif()
      #message(STATUS "${GVS_PROJECT_NAME}_BUILD_DATE: ${${GVS_PROJECT_NAME}_BUILD_DATE}")
  ELSE (WIN32)
      MESSAGE(SEND_ERROR "date for this operating system not implemented")
      set(${GVS_PROJECT_NAME}_BUILD_DATE "0000/00/00" PARENT_SCOPE)
  ENDIF (WIN32)

endfunction()
