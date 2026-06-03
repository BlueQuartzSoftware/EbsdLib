/* ============================================================================
 * Copyright (c) 2009-2025 BlueQuartz Software, LLC
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Tests for the ebsdlib::CropImageToContent helper used by the IPF triangle
 * legend renderer to trim the large white margins around the SST.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/Utilities/ImageCrop.hpp"

#include <cstdint>
#include <vector>
#include <array>

using namespace ebsdlib;

namespace
{
// Build a 100x100 RGB canvas of all-white pixels with a single colored
// rectangle from (rectX, rectY) to (rectX+rectW-1, rectY+rectH-1).
UInt8ArrayType::Pointer makeRectImage(int canvasDim, int rectX, int rectY, int rectW, int rectH, std::array<uint8_t, 3> color)
{
  auto img = UInt8ArrayType::CreateArray(static_cast<size_t>(canvasDim * canvasDim), {3ULL}, "rect", true);
  uint8_t* p = img->getPointer(0);
  for(int i = 0; i < canvasDim * canvasDim; ++i)
  {
    p[i * 3] = 255;
    p[i * 3 + 1] = 255;
    p[i * 3 + 2] = 255;
  }
  for(int y = rectY; y < rectY + rectH; ++y)
  {
    for(int x = rectX; x < rectX + rectW; ++x)
    {
      const size_t idx = (static_cast<size_t>(y) * canvasDim + static_cast<size_t>(x)) * 3;
      p[idx] = color[0];
      p[idx + 1] = color[1];
      p[idx + 2] = color[2];
    }
  }
  return img;
}
} // namespace

TEST_CASE("ebsdlib::ImageCropTest::CropsToBoundingBoxPlusPadding", "[EbsdLib][ImageCropTest]")
{
  constexpr int k_CanvasDim = 100;
  constexpr int k_Padding = 4;

  // Red 20x10 rectangle at (40, 30) on a 100x100 white canvas.
  auto img = makeRectImage(k_CanvasDim, 40, 30, 20, 10, {255, 0, 0});

  auto cropped = CropImageToContent(img.get(), k_CanvasDim, k_CanvasDim, /*channels*/ 3, k_Padding);

  REQUIRE(cropped.image != nullptr);
  // Bounding box: x in [40, 59], y in [30, 39] -> 20x10
  // Plus padding on each side: width = 20 + 2*4 = 28, height = 10 + 2*4 = 18
  CHECK(cropped.width == 28);
  CHECK(cropped.height == 18);
  CHECK(cropped.image->getNumberOfTuples() == static_cast<size_t>(cropped.width * cropped.height));
}

TEST_CASE("ebsdlib::ImageCropTest::ClampsBoundingBoxToCanvas", "[EbsdLib][ImageCropTest]")
{
  constexpr int k_CanvasDim = 50;

  // Small rectangle at the corner; padding would push the bounding box
  // outside the canvas but must be clamped.
  auto img = makeRectImage(k_CanvasDim, 0, 0, 5, 5, {0, 255, 0});
  auto cropped = CropImageToContent(img.get(), k_CanvasDim, k_CanvasDim, 3, /*padding*/ 100);

  REQUIRE(cropped.image != nullptr);
  CHECK(cropped.width == k_CanvasDim);
  CHECK(cropped.height == k_CanvasDim);
}

TEST_CASE("ebsdlib::ImageCropTest::AllWhiteImageReturnsOriginalSize", "[EbsdLib][ImageCropTest]")
{
  // A pathological all-white canvas has no non-background content; we
  // should not crash and not return an empty image. Returning the
  // original canvas is the safe fallback.
  constexpr int k_CanvasDim = 32;
  auto img = UInt8ArrayType::CreateArray(static_cast<size_t>(k_CanvasDim * k_CanvasDim), {3ULL}, "white", true);
  uint8_t* p = img->getPointer(0);
  for(int i = 0; i < k_CanvasDim * k_CanvasDim * 3; ++i)
  {
    p[i] = 255;
  }
  auto cropped = CropImageToContent(img.get(), k_CanvasDim, k_CanvasDim, 3, 4);
  REQUIRE(cropped.image != nullptr);
  CHECK(cropped.width == k_CanvasDim);
  CHECK(cropped.height == k_CanvasDim);
}

TEST_CASE("ebsdlib::ImageCropTest::CroppedPixelsMatchSource", "[EbsdLib][ImageCropTest]")
{
  // A blue rectangle. The cropped image's pixels should match the
  // corresponding source pixels (within the bounding-box+padding window).
  constexpr int k_CanvasDim = 64;
  constexpr int k_RectX = 20;
  constexpr int k_RectY = 25;
  constexpr int k_RectW = 8;
  constexpr int k_RectH = 6;
  constexpr int k_Padding = 2;

  auto img = makeRectImage(k_CanvasDim, k_RectX, k_RectY, k_RectW, k_RectH, {0, 0, 255});
  auto cropped = CropImageToContent(img.get(), k_CanvasDim, k_CanvasDim, 3, k_Padding);

  REQUIRE(cropped.image != nullptr);
  CHECK(cropped.width == k_RectW + 2 * k_Padding);
  CHECK(cropped.height == k_RectH + 2 * k_Padding);

  // Center pixel of the rectangle in the cropped image should be blue.
  const int centerXcropped = (cropped.width / 2);
  const int centerYcropped = (cropped.height / 2);
  const size_t idx = (static_cast<size_t>(centerYcropped) * cropped.width + static_cast<size_t>(centerXcropped)) * 3;
  const uint8_t* p = cropped.image->getPointer(0);
  CHECK(static_cast<int>(p[idx]) == 0);
  CHECK(static_cast<int>(p[idx + 1]) == 0);
  CHECK(static_cast<int>(p[idx + 2]) == 255);
}

// -----------------------------------------------------------------------------
// Integration test: feed a real IPF triangle legend into the cropper and
// assert the output is meaningfully smaller than the input canvas.
// HexagonalHigh's SST is a 30° wedge, so the canvas has a lot of whitespace
// and crop should produce a noticeably smaller image.
#include "EbsdLib/LaueOps/HexagonalOps.h"

TEST_CASE("ebsdlib::ImageCropTest::HexagonalHighLegendIsCroppedSmaller", "[EbsdLib][ImageCropTest]")
{
  HexagonalOps ops;
  constexpr int k_CanvasDim = 512;
  auto legend = ops.generateIPFTriangleLegend(k_CanvasDim, /*generateEntirePlane*/ false, ebsdlib::HexConvention::XParallelAStar);
  REQUIRE(legend != nullptr);
  REQUIRE(legend->getNumberOfTuples() == static_cast<size_t>(k_CanvasDim * k_CanvasDim));

  auto cropped = CropImageToContent(legend.get(), k_CanvasDim, k_CanvasDim, /*channels*/ 3, /*padding*/ 8);
  REQUIRE(cropped.image != nullptr);
  CHECK(cropped.width < k_CanvasDim);
  CHECK(cropped.height < k_CanvasDim);
  // Sanity: not so aggressive that we trimmed actual content (>= half the canvas means we left the SST + labels).
  CHECK(cropped.width > k_CanvasDim / 4);
  CHECK(cropped.height > k_CanvasDim / 4);
}
