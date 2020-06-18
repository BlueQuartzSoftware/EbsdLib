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


# --------------------------------------------------------------------------
# Adds a Unit Test 
function(AddSIMPLUnitTest)
    set(options)
    set(oneValueArgs TESTNAME FOLDER)
    set(multiValueArgs SOURCES LINK_LIBRARIES INCLUDE_DIRS)
    cmake_parse_arguments(Z "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

    add_executable( ${Z_TESTNAME} ${Z_SOURCES})
    if("${Z_FOLDER}" STREQUAL "")
        set(Z_FOLDER "Test")
    endif()
    set_target_properties( ${Z_TESTNAME} PROPERTIES FOLDER ${Z_FOLDER})
    CMP_AddDefinitions(TARGET ${Z_TESTNAME} )
#    cmp_IDE_SOURCE_PROPERTIES( "" "" "${Z_SOURCES}" "0")
    target_include_directories(${Z_TESTNAME} PUBLIC ${Z_INCLUDE_DIRS})
    target_link_libraries( ${Z_TESTNAME} ${Z_LINK_LIBRARIES})
    add_test(${Z_TESTNAME} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${Z_TESTNAME})

endfunction()


enable_testing()
set(TEST_TEMP_DIR ${EbsdLibProj_BINARY_DIR}/Test/Temp)
# Make sure the directory is created during CMake time
file(MAKE_DIRECTORY ${TEST_TEMP_DIR})

configure_file(${EbsdLibProj_SOURCE_DIR}/Source/Test/TestFileLocations.h.in
               ${EbsdLibProj_BINARY_DIR}/EbsdLib/Test/EbsdLibTestFileLocations.h @ONLY IMMEDIATE)

#------------------------------------------------------------------------------
# List all the source files here. They will NOT be compiled but instead
# be directly included in the main test source file. We list them here so that
# they will show up in IDEs
set(TEST_NAMES
  OrientationArrayTest
  OrientationConverterTest
  OrientationTransformsTest
  OrientationTest
  QuaternionTest

  AngImportTest
  CtfReaderTest

  ODFTest

  SO3SamplerTest
  TextureTest
)

if(EbsdLib_ENABLE_HDF5)
  set(TEST_NAMES
    ${TEST_NAMES}
    H5EspritReaderTest
    EdaxOIMReaderTest
  )

endif()


set(EbsdLibProj_TEST_SRCS )
set(FilterTestIncludes "")
set(TestMainFunctors "")

foreach(name  ${TEST_NAMES})
  set( EbsdLibProj_TEST_SRCS
    ${EbsdLibProj_TEST_SRCS}
    "${EbsdLibProj_SOURCE_DIR}/Source/Test/${name}.cpp"
    )
  string(CONCAT
    FilterTestIncludes
    ${FilterTestIncludes}
    "#include \"${name}.cpp\"\n"
    )

  string(CONCAT
    TestMainFunctors
   ${TestMainFunctors}
   "  ${name}()()|\n")
endforeach()

STRING(REPLACE "|" ";" TestMainFunctors ${TestMainFunctors}   )

set(QT_APPLICATION_CLASS_HEADER "//#include <QtCore/QCoreApplication>")
set(QT_APPLICATION_CLASS "//QCoreApplication")

configure_file(${EbsdLibProj_SOURCE_DIR}/Source/Test/TestMain.cpp.in
               ${EbsdLibProj_BINARY_DIR}/Test/EbsdLibUnitTest.cpp @ONLY)


# Set the source files properties on each source file.
foreach(f ${EbsdLibProj_TEST_SRCS})
  set_source_files_properties( ${f} PROPERTIES HEADER_FILE_ONLY TRUE)
endforeach()


AddSIMPLUnitTest(TESTNAME EbsdLibUnitTest
  SOURCES ${EbsdLibProj_BINARY_DIR}/Test/EbsdLibUnitTest.cpp ${EbsdLibProj_TEST_SRCS}
  FOLDER "EbsdLibProj/Test"
  LINK_LIBRARIES  EbsdLib
  INCLUDE_DIRS
    ${EbsdLibProj_SOURCE_DIR}/Source
    ${EbsdLibProj_SOURCE_DIR}/Source/Test
    ${EbsdLibProj_BINARY_DIR}
    ${EbsdLibProj_BINARY_DIR}/Test
    ${EIGEN3_INCLUDE_DIR}
    ${HDF5_INCLUDE_DIRS}
    ${HDF5_INCLUDE_DIR}
    ${H5Support_SOURCE_DIR}/Source
  )
target_compile_definitions( EbsdLibUnitTest PRIVATE  -DH5Support_USE_QT)


if(MSVC)
  set_source_files_properties(${EbsdLibProj_BINARY_DIR}/EbsdLibUnitTest.cpp PROPERTIES COMPILE_FLAGS /bigobj)
endif()

