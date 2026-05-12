set(UNIT_TEST_TARGET EbsdLibUnitTest)

set(EBSDLIB_TEST_DIRS_HEADER ${EbsdLibProj_BINARY_DIR}/EbsdLib/Test/EbsdLibTestFileLocations.h)
configure_file(${EbsdLibProj_SOURCE_DIR}/Source/Test/TestFileLocations.h.in ${EBSDLIB_TEST_DIRS_HEADER} @ONLY)

file(MAKE_DIRECTORY ${EbsdLibProj_BINARY_DIR}/Testing/Temporary/)
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
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/Matrix3X1Test.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/Matrix3X3Test.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/ArrayHelpersTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/EbsdDataArrayTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/EbsdStringUtilsTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/EbsdTransformTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/EbsdLibRandomTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/OrientationMathTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/ColorTableTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/LambertUtilitiesTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/StereographicProjectionTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/TexturePresetTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/PhaseTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/LaueOpsTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/ModifiedLambertProjection3DTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/ModifiedLambertProjectionTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/OrientationTransformationTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/ToolTipGeneratorTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/PoleFigureDataTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/CanvasUtilitiesTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/ModifiedLambertProjectionArrayTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/PoleFigureUtilitiesTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/PoleFigureCompositorTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/PoleFigureLaueComparisonTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/PoleFigurePositionTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/RenderEbsdSmokeTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/ImageCropTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Apps/render_ebsd.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/InversePoleFigureTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/TiffWriterTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/DirectionalStatsTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/UnitTestCommon.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/UnitTestCommon.hpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/ColorSpaceUtilsTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/FundamentalSectorGeometryTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/TSLColorKeyTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/NolzeHielscherColorKeyTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/GriddedColorKeyTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/PUCMColorKeyTest.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/Test/ColorKeyKindTest.cpp
)


set(DISABLED_TESTS

)

# -----------------------------------------------------------------------------
# Here we are going to setup to download and decompress the test files. In order
# to add your own test files you will need to tar.gz the test file, compute the
# SHA 512 Hash of the file and then upload the file to
# https://github.com/bluequartzsoftware/simplnx/releases/tag/Data_Archive.
#
# Go to the web site above, "edit" the release, add the filename and SHA 512 to
# the table and then upload your compressed file.
# Save the release so that the repo is updated
# -----------------------------------------------------------------------------
option(EBSDLIB_DOWNLOAD_TEST_FILES "Download the test files" ON)

if(EBSDLIB_DOWNLOAD_TEST_FILES)
  if(NOT EXISTS ${EBSDLIB_DATA_DIR}/TestFiles/)
    file(MAKE_DIRECTORY "${EBSDLIB_DATA_DIR}/TestFiles/")
  endif()

  ebsdlib_download_test_data(EBSDLIB_DATA_DIR ${EBSDLIB_DATA_DIR} ARCHIVE_NAME Laue_Orientation_Clusters_v6.tar.gz SHA512 f327d3d2a86d539b3c1f3fc755d8f5741d8eb68aab45fc1ab54d9e5db48643903f9a37898366203e6eb2e7585ce57c6e186cca2107acb1a53318b813345cb10a)
  ebsdlib_download_test_data(EBSDLIB_DATA_DIR ${EBSDLIB_DATA_DIR} ARCHIVE_NAME Pole_Figure_Images_v2.tar.gz SHA512 f272af212fa634d702129721de4b5bbb62a89a0cf872f4230df3ad809cf5c7c6c075847d93efe9c1866c2a832893602b9486937679bb0a1d5987df26f9766799)

endif()


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
    ZLIB::ZLIB
    fmt::fmt
)

target_include_directories(${UNIT_TEST_TARGET}
  PRIVATE
    "${EbsdLibProj_SOURCE_DIR}/3rdParty/canvas_ity/src"
    "${EbsdLibProj_SOURCE_DIR}/Source"
)


if(MSVC)
  set_source_files_properties(${EbsdLibProj_BINARY_DIR}/EbsdLibUnitTest.cpp PROPERTIES COMPILE_FLAGS /bigobj)
endif()

catch_discover_tests(${UNIT_TEST_TARGET} TEST_PREFIX "EbsdLib::")

get_property(EbsdLib_EXTRA_LIBRARY_DIRS GLOBAL PROPERTY EbsdLib_EXTRA_LIBRARY_DIRS)
