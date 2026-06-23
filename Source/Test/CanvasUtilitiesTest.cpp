#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/Utilities/CanvasUtilities.hpp"
#include "EbsdLib/Utilities/PngWriter.h"

#include <cmath>
#include <string>
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
// Pins down EXACTLY what MirrorImage() does to known pixel data. This is the
// "rotate 180 degrees about the horizontal (X) axis" operation used by
// PoleFigureCompositor when flipFinalImage is set.
//
// Each source pixel encodes its own coordinates: R = x (column), G = y (row).
// After the transform we read each destination pixel and recover which source
// pixel landed there. A 180-degree rotation about the X axis is a vertical
// flip: column x is preserved, row y is reversed (y -> dim-1-y).
//
//   For comparison, the alternatives this test distinguishes:
//     * vertical flip   (about X):  dest(x,y) == src(x, dim-1-y)   <-- expected
//     * horizontal flip (about Y):  dest(x,y) == src(dim-1-x, y)
//     * 180 in-plane    (about Z):  dest(x,y) == src(dim-1-x, dim-1-y)
// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::CanvasUtilitiesTest::MirrorImageCoordinateMapping", "[EbsdLib][CanvasUtilitiesTest]")
{
  const int dim = 4;
  std::vector<size_t> cDims = {4}; // RGBA
  auto src = EbsdDataArray<uint8_t>::CreateArray(static_cast<size_t>(dim) * dim, cDims, "MirrorCoordSrc", true);

  // Encode each pixel's coordinates into its color so the mapping is observable.
  for(int y = 0; y < dim; y++)
  {
    for(int x = 0; x < dim; x++)
    {
      uint8_t tuple[4] = {static_cast<uint8_t>(x), static_cast<uint8_t>(y), 0, 255};
      src->setTuple(static_cast<size_t>(y) * dim + x, tuple);
    }
  }

  auto result = MirrorImage<uint8_t>(src.get(), dim);
  REQUIRE(result != nullptr);
  REQUIRE(result->getNumberOfTuples() == static_cast<size_t>(dim) * dim);
  REQUIRE(result->getNumberOfComponents() == 4);

  // Every destination pixel must hold the source pixel from the vertically
  // mirrored row: dest(x,y) == src(x, dim-1-y).
  for(int y = 0; y < dim; y++)
  {
    for(int x = 0; x < dim; x++)
    {
      uint8_t* destPixel = result->getTuplePointer(static_cast<size_t>(y) * dim + x);
      const int recoveredSrcX = destPixel[0];
      const int recoveredSrcY = destPixel[1];
      REQUIRE(recoveredSrcX == x);             // column preserved
      REQUIRE(recoveredSrcY == (dim - 1 - y)); // row reversed
      REQUIRE(destPixel[3] == 255);            // alpha preserved
    }
  }

  // Explicit corner checks make the handedness unambiguous:
  //   top-left  dest(0,0)     <- src(0, dim-1)     (bottom-left)
  //   top-right dest(dim-1,0) <- src(dim-1, dim-1) (bottom-right)
  uint8_t* topLeft = result->getTuplePointer(0);
  REQUIRE(topLeft[0] == 0);
  REQUIRE(topLeft[1] == dim - 1);

  uint8_t* topRight = result->getTuplePointer(static_cast<size_t>(dim) - 1);
  REQUIRE(topRight[0] == dim - 1);
  REQUIRE(topRight[1] == dim - 1);
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

// -----------------------------------------------------------------------------
// Hidden test (note the leading '.' in the tag): does NOT run as part of the
// normal suite. Run explicitly with:
//   ./Bin/EbsdLibUnitTest "[MirrorVisual]"
//
// Renders a recognizable, deliberately asymmetric RGBA image, applies
// MirrorImage() (the "rotate 180 about X" / vertical-flip used by the pole
// figure compositor), and writes both the input and output as PNG files so the
// flip can be inspected by eye.
// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::CanvasUtilitiesTest::MirrorImageVisualExport", "[.][MirrorVisual]")
{
  const int dim = 256;
  const std::string outDir = "/private/tmp/claude-501/-Users-mjackson-Workspace9-DREAM3DNX/485763c8-b0c7-48d6-aa56-f82ab8220d1b/scratchpad";

  std::vector<size_t> cDims = {4}; // RGBA
  auto src = EbsdDataArray<uint8_t>::CreateArray(static_cast<size_t>(dim) * dim, cDims, "MirrorVisualSrc", true);

  // Base pattern: a gradient that is unmistakable about orientation.
  //   R increases left -> right (encodes column x)
  //   G increases top  -> bottom (encodes row y)
  for(int y = 0; y < dim; y++)
  {
    for(int x = 0; x < dim; x++)
    {
      uint8_t r = static_cast<uint8_t>((x * 255) / (dim - 1));
      uint8_t g = static_cast<uint8_t>((y * 255) / (dim - 1));
      uint8_t tuple[4] = {r, g, 0, 255};
      src->setTuple(static_cast<size_t>(y) * dim + x, tuple);
    }
  }

  // Solid BLUE horizontal bar across the TOP 1/8 of the image.
  for(int y = 0; y < dim / 8; y++)
  {
    for(int x = 0; x < dim; x++)
    {
      uint8_t tuple[4] = {0, 0, 255, 255};
      src->setTuple(static_cast<size_t>(y) * dim + x, tuple);
    }
  }

  // Solid WHITE square marker in the TOP-LEFT corner.
  for(int y = 0; y < dim / 8; y++)
  {
    for(int x = 0; x < dim / 8; x++)
    {
      uint8_t tuple[4] = {255, 255, 255, 255};
      src->setTuple(static_cast<size_t>(y) * dim + x, tuple);
    }
  }

  auto result = MirrorImage<uint8_t>(src.get(), dim);
  REQUIRE(result != nullptr);

  auto inResult = PngWriter::WriteColorImage(outDir + "/mirror_input.png", dim, dim, 4, src->data());
  auto outResult = PngWriter::WriteColorImage(outDir + "/mirror_output.png", dim, dim, 4, result->data());
  REQUIRE(inResult.first == 0);
  REQUIRE(outResult.first == 0);

  // After a vertical flip, the BLUE bar and WHITE marker should be at the BOTTOM.
  uint8_t* bottomLeft = result->getTuplePointer(static_cast<size_t>(dim - 1) * dim);
  REQUIRE(bottomLeft[0] == 255); // white marker now bottom-left
  REQUIRE(bottomLeft[1] == 255);
  REQUIRE(bottomLeft[2] == 255);
}
