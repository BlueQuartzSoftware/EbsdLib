#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/Utilities/CanvasUtilities.hpp"

#include <cmath>
#include <vector>

using namespace ebsdlib;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::CanvasUtilitiesTest::MirrorImage", "[EbsdLib][CanvasUtilitiesTest]")
{
  const int dim = 4;
  std::vector<size_t> cDims = {4}; // RGBA
  auto src = EbsdDataArray<uint8_t>::CreateArray(dim * dim, cDims, "MirrorTestSrc", true);

  // Set top row (y=0) to red, bottom row (y=3) to blue
  for(int x = 0; x < dim; x++)
  {
    uint8_t red[4] = {255, 0, 0, 255};
    uint8_t blue[4] = {0, 0, 255, 255};
    src->setTuple(0 * dim + x, red);          // top row
    src->setTuple((dim - 1) * dim + x, blue); // bottom row
  }

  auto result = MirrorImage<uint8_t>(src.get(), dim);

  // After mirror, top row should be blue, bottom row should be red
  uint8_t* topPixel = result->getTuplePointer(0);
  REQUIRE(topPixel[0] == 0);   // R
  REQUIRE(topPixel[2] == 255); // B

  uint8_t* bottomPixel = result->getTuplePointer((dim - 1) * dim);
  REQUIRE(bottomPixel[0] == 255); // R
  REQUIRE(bottomPixel[2] == 0);   // B
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::CanvasUtilitiesTest::RotateImage90About001", "[EbsdLib][CanvasUtilitiesTest]")
{
  const int width = 4;
  const int height = 4;
  std::vector<size_t> cDims = {4}; // RGBA
  auto src = EbsdDataArray<uint8_t>::CreateArray(width * height, cDims, "RotateTestSrc", true);
  src->initializeWithZeros();

  // Place a known pixel at (x=0, y=0)
  uint8_t marker[4] = {111, 222, 33, 255};
  src->setTuple(0, marker);

  auto result = RotateImage90About001<uint8_t>(src.get(), width, height);

  // After clockwise 90 rotation: (0,0) -> (height-1, 0) in rotated image
  // new_x = height - y - 1 = 3, new_y = x = 0, rotWidth = height = 4
  // destIdx = new_y * rotWidth + new_x = 0 * 4 + 3 = 3
  uint8_t* rotPixel = result->getTuplePointer(3);
  REQUIRE(rotPixel[0] == 111);
  REQUIRE(rotPixel[1] == 222);
  REQUIRE(rotPixel[2] == 33);
  REQUIRE(rotPixel[3] == 255);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::CanvasUtilitiesTest::ConvertColorOrder", "[EbsdLib][CanvasUtilitiesTest]")
{
  const int dim = 2;
  std::vector<size_t> cDims = {4};
  auto src = EbsdDataArray<uint8_t>::CreateArray(dim * dim, cDims, "ConvertColorSrc", true);

  // Set ARGB pixel: [A=10, R=20, G=30, B=40]
  uint8_t argb[4] = {10, 20, 30, 40};
  src->setTuple(0, argb);

  auto result = ConvertColorOrder<uint8_t>(src.get(), dim);

  // ConvertColorOrder swaps: dest[0]=src[2], dest[1]=src[1], dest[2]=src[0], dest[3]=src[3]
  uint8_t* destPixel = result->getTuplePointer(0);
  REQUIRE(destPixel[0] == 30); // src[2] = G
  REQUIRE(destPixel[1] == 20); // src[1] = R
  REQUIRE(destPixel[2] == 10); // src[0] = A
  REQUIRE(destPixel[3] == 40); // src[3] = B
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::CanvasUtilitiesTest::RemoveAlphaChannel", "[EbsdLib][CanvasUtilitiesTest]")
{
  std::vector<size_t> cDims = {4};
  auto src = EbsdDataArray<uint8_t>::CreateArray(4, cDims, "RemoveAlphaSrc", true);

  uint8_t rgba[4] = {100, 150, 200, 255};
  src->setTuple(0, rgba);

  auto result = RemoveAlphaChannel<uint8_t>(src.get());

  REQUIRE(result->getNumberOfTuples() == 4);
  REQUIRE(result->getNumberOfComponents() == 3);

  uint8_t* destPixel = result->getTuplePointer(0);
  REQUIRE(destPixel[0] == 100);
  REQUIRE(destPixel[1] == 150);
  REQUIRE(destPixel[2] == 200);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::CanvasUtilitiesTest::CropRGBImage", "[EbsdLib][CanvasUtilitiesTest]")
{
  const int width = 8;
  const int height = 8;
  std::vector<size_t> cDims = {3}; // RGB
  auto src = EbsdDataArray<uint8_t>::CreateArray(width * height, cDims, "CropSrc", true);

  // Fill entire image with a gradient based on linear index
  for(int i = 0; i < width * height; i++)
  {
    uint8_t val = static_cast<uint8_t>(i);
    uint8_t pixel[3] = {val, val, val};
    src->setTuple(i, pixel);
  }

  // Crop a 4x4 region starting at col=2, row=2
  auto result = CropRGBImage<uint8_t>(src, width, height, 2, 2, 4, 4);

  REQUIRE(result->getNumberOfTuples() == 16); // 4*4

  // The pixel at (col=2, row=2) in the source is at index 2*8+2=18
  uint8_t* firstPixel = result->getTuplePointer(0);
  REQUIRE(firstPixel[0] == 18);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::CanvasUtilitiesTest::GeneratePointsOnUnitCircle", "[EbsdLib][CanvasUtilitiesTest]")
{
  Point3DType direction(0.0, 0.0, 1.0); // Z-axis
  int numPoints = 36;

  std::vector<Point3DType> points = GeneratePointsOnUnitCircle(direction, numPoints);

  // GeneratePointsOnUnitCircle generates num_points + 1 to close the circle
  REQUIRE(points.size() == static_cast<size_t>(numPoints + 1));

  for(const auto& pt : points)
  {
    double magnitude = std::sqrt(pt[0] * pt[0] + pt[1] * pt[1] + pt[2] * pt[2]);
    REQUIRE(magnitude == Approx(1.0).margin(1e-10));

    // Each point should be orthogonal to the direction (dot product ~0)
    double dot = pt[0] * direction[0] + pt[1] * direction[1] + pt[2] * direction[2];
    REQUIRE(dot == Approx(0.0).margin(1e-10));
  }
}
