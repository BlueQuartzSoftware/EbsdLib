set(UNIT_TEST_TARGET EbsdLibUnitTest)

set(EBSDLIB_TEST_DIRS_HEADER ${EbsdLibProj_BINARY_DIR}/EbsdLib/Test/EbsdLibTestFileLocations.h)
configure_file(${EbsdLibProj_SOURCE_DIR}/Source/Test/TestFileLocations.h.in ${EBSDLIB_TEST_DIRS_HEADER} @ONLY)

#------------------------------------------------------------------------------
# Find the Catch2 unit testing package
find_package(Catch2 CONFIG REQUIRED)
include(Catch)


# ------------------------------------------------------------------------------
# Define the list of unit test source files
set(EbsdLib_UnitTest_SRCS
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/AngImportTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/AngleFileLoaderTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/OrientationTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/ConvertToFundamentalZoneTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/CtfReaderTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/EdaxOIMReaderTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/H5EspritReaderTest.cpp
    # ${EbsdLibProj_SOURCE_DIR}/Source/Test/H5OINAReaderTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/IPFLegendTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/QuaternionTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/ODFTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/TextureTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/OrientationConverterTest.cpp
)


set(DISABLED_TESTS

)


#------------------------------------------------------------------------------
# Create the unit test executable
add_executable(${UNIT_TEST_TARGET}
  ${EBSDLIB_TEST_DIRS_HEADER}
  ${EbsdLibProj_SOURCE_DIR}/Source/Test/EbsdLib_test_main.cpp
  ${EbsdLib_UnitTest_SRCS}
)

target_link_libraries(${UNIT_TEST_TARGET}
  PRIVATE
    EbsdLib
    Catch2::Catch2
)


if(MSVC)
  set_source_files_properties(${EbsdLibProj_BINARY_DIR}/EbsdLibUnitTest.cpp PROPERTIES COMPILE_FLAGS /bigobj)
endif()

catch_discover_tests(${UNIT_TEST_TARGET} TEST_PREFIX "EbsdLib::")

get_property(EbsdLib_EXTRA_LIBRARY_DIRS GLOBAL PROPERTY EbsdLib_EXTRA_LIBRARY_DIRS)
