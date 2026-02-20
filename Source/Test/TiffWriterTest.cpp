#include <catch2/catch.hpp>

#include "EbsdLib/Test/EbsdLibTestFileLocations.h"
#include "EbsdLib/Utilities/TiffWriter.h"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::TiffWriterTest::WriteColorImage_RGB", "[EbsdLib][TiffWriterTest]")
{
  std::string outputPath = UnitTest::TestTempDir + "TiffWriterTest_RGB.tif";

  int32_t width = 4;
  int32_t height = 4;
  uint16_t samplesPerPixel = 3;

  // Create a solid red image
  std::vector<uint8_t> data(width * height * samplesPerPixel, 0);
  for(int i = 0; i < width * height; i++)
  {
    data[i * 3 + 0] = 255; // R
    data[i * 3 + 1] = 0;   // G
    data[i * 3 + 2] = 0;   // B
  }

  auto [errCode, errMsg] = TiffWriter::WriteColorImage(outputPath, width, height, samplesPerPixel, data.data());
  REQUIRE(errCode == 0);
  REQUIRE(fs::exists(outputPath));
  REQUIRE(fs::file_size(outputPath) > 0);

#if REMOVE_TEST_FILES
  fs::remove(outputPath);
#endif
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::TiffWriterTest::WriteColorImage_RGBA", "[EbsdLib][TiffWriterTest]")
{
  std::string outputPath = UnitTest::TestTempDir + "TiffWriterTest_RGBA.tif";

  int32_t width = 4;
  int32_t height = 4;
  uint16_t samplesPerPixel = 4;

  // Create a solid green image with alpha
  std::vector<uint8_t> data(width * height * samplesPerPixel, 0);
  for(int i = 0; i < width * height; i++)
  {
    data[i * 4 + 0] = 0;   // R
    data[i * 4 + 1] = 255; // G
    data[i * 4 + 2] = 0;   // B
    data[i * 4 + 3] = 255; // A
  }

  auto [errCode, errMsg] = TiffWriter::WriteColorImage(outputPath, width, height, samplesPerPixel, data.data());
  REQUIRE(errCode == 0);
  REQUIRE(fs::exists(outputPath));
  REQUIRE(fs::file_size(outputPath) > 0);

#if REMOVE_TEST_FILES
  fs::remove(outputPath);
#endif
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::TiffWriterTest::WriteGrayScaleImage", "[EbsdLib][TiffWriterTest]")
{
  std::string outputPath = UnitTest::TestTempDir + "TiffWriterTest_Gray.tif";

  int32_t width = 4;
  int32_t height = 4;

  // Create a mid-gray image
  std::vector<uint8_t> data(width * height, 128);

  auto [errCode, errMsg] = TiffWriter::WriteGrayScaleImage(outputPath, width, height, data.data());
  REQUIRE(errCode == 0);
  REQUIRE(fs::exists(outputPath));
  REQUIRE(fs::file_size(outputPath) > 0);

#if REMOVE_TEST_FILES
  fs::remove(outputPath);
#endif
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::TiffWriterTest::WriteColorImage_InvalidPath", "[EbsdLib][TiffWriterTest]")
{
  std::string outputPath = "/nonexistent_directory_xyz/TiffWriterTest_Invalid.tif";

  int32_t width = 4;
  int32_t height = 4;
  uint16_t samplesPerPixel = 3;
  std::vector<uint8_t> data(width * height * samplesPerPixel, 0);

  auto [errCode, errMsg] = TiffWriter::WriteColorImage(outputPath, width, height, samplesPerPixel, data.data());
  REQUIRE(errCode != 0);
}
