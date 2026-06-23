#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/Utilities/PoleFigureUtilities.h"

#include <string>
#include <vector>

using namespace ebsdlib;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureUtilitiesTest::PoleFigureConfiguration_t_Fields", "[EbsdLib][PoleFigureUtilitiesTest]")
{
  PoleFigureConfiguration_t config;
  config.eulers = nullptr;
  config.imageDim = 256;
  config.lambertDim = 64;
  config.numColors = 32;
  config.minScale = 0.0;
  config.maxScale = 1.0;
  config.sphereRadius = 1.0f;
  config.discrete = false;
  config.discreteHeatMap = false;
  config.colorMap = "Default";
  config.labels = {"A", "B", "C"};
  config.order = {0, 1, 2};
  config.phaseName = "Phase1";
  config.flipFinalImage = true;

  REQUIRE(config.imageDim == 256);
  REQUIRE(config.lambertDim == 64);
  REQUIRE(config.numColors == 32);
  REQUIRE(config.minScale == Approx(0.0));
  REQUIRE(config.maxScale == Approx(1.0));
  REQUIRE(config.sphereRadius == Approx(1.0f));
  REQUIRE(config.discrete == false);
  REQUIRE(config.discreteHeatMap == false);
  REQUIRE(config.colorMap == "Default");
  REQUIRE(config.labels.size() == 3);
  REQUIRE(config.order.size() == 3);
  REQUIRE(config.phaseName == "Phase1");
  REQUIRE(config.flipFinalImage == true);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureUtilitiesTest::CreateColorImage_UniformData", "[EbsdLib][PoleFigureUtilitiesTest]")
{
  int width = 16;
  int height = 16;
  int nColors = 16;

  // Create uniform intensity data (all zeros)
  auto data = DoubleArrayType::CreateArray(static_cast<size_t>(width * height), {1ULL}, "Intensity", true);
  data->initializeWithZeros();

  auto image = PoleFigureUtilities::CreateColorImage(data.get(), width, height, nColors, "TestImage", 0.0, 1.0);

  REQUIRE(image != nullptr);
  REQUIRE(image->getNumberOfTuples() == static_cast<size_t>(width * height));
  REQUIRE(image->getNumberOfComponents() == 4); // RGBA
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureUtilitiesTest::CreateColorImage_GradientData", "[EbsdLib][PoleFigureUtilitiesTest]")
{
  int width = 16;
  int height = 16;
  int nColors = 16;

  // Create gradient intensity data (linear ramp 0 to 1)
  auto data = DoubleArrayType::CreateArray(static_cast<size_t>(width * height), {1ULL}, "GradientIntensity", true);
  double numPixels = static_cast<double>(width * height);
  for(size_t i = 0; i < data->getNumberOfTuples(); i++)
  {
    data->setValue(i, static_cast<double>(i) / numPixels);
  }

  auto image = PoleFigureUtilities::CreateColorImage(data.get(), width, height, nColors, "GradientImage", 0.0, 1.0);

  REQUIRE(image != nullptr);
  REQUIRE(image->getNumberOfTuples() == static_cast<size_t>(width * height));

  // Verify that the image has non-zero content (at least some pixel is colored)
  bool hasNonZero = false;
  for(size_t i = 0; i < image->getNumberOfTuples(); i++)
  {
    uint8_t* pixel = image->getTuplePointer(i);
    if(pixel[0] != 0 || pixel[1] != 0 || pixel[2] != 0)
    {
      hasNonZero = true;
      break;
    }
  }
  REQUIRE(hasNonZero);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureUtilitiesTest::GeneratePoleFigureRgbaImageImpl_Operator", "[EbsdLib][PoleFigureUtilitiesTest]")
{
  int imageDim = 16;
  int nColors = 16;

  auto intensity = DoubleArrayType::CreateArray(static_cast<size_t>(imageDim * imageDim), {1ULL}, "Intensity", true);
  for(size_t i = 0; i < intensity->getNumberOfTuples(); i++)
  {
    intensity->setValue(i, static_cast<double>(i) / static_cast<double>(intensity->getNumberOfTuples()));
  }

  PoleFigureConfiguration_t config;
  config.eulers = nullptr;
  config.imageDim = imageDim;
  config.numColors = nColors;
  config.minScale = 0.0;
  config.maxScale = 1.0;
  config.sphereRadius = 1.0f;
  config.discrete = false;
  config.discreteHeatMap = false;

  std::vector<size_t> cDims = {4};
  auto rgba = UInt8ArrayType::CreateArray(static_cast<size_t>(imageDim * imageDim), cDims, "RGBA", true);
  rgba->initializeWithZeros();

  GeneratePoleFigureRgbaImageImpl impl(intensity.get(), &config, rgba.get());
  impl();

  // Verify the image was populated (at least some non-zero pixel)
  bool hasNonZero = false;
  for(size_t i = 0; i < rgba->getNumberOfTuples(); i++)
  {
    uint8_t* pixel = rgba->getTuplePointer(i);
    if(pixel[0] != 0 || pixel[1] != 0 || pixel[2] != 0 || pixel[3] != 0)
    {
      hasNonZero = true;
      break;
    }
  }
  REQUIRE(hasNonZero);
}
