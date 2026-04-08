/* ============================================================================
 * Copyright (c) 2009-2025 BlueQuartz Software, LLC
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of BlueQuartz Software, the US Air Force, nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/Utilities/PoleFigureCompositor.h"

using namespace ebsdlib;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureCompositorTest::ConfigDefaults", "[EbsdLib][PoleFigureCompositorTest]")
{
  CompositePoleFigureConfiguration_t config;

  REQUIRE(config.eulers == nullptr);
  REQUIRE(config.imageDim == 512);
  REQUIRE(config.lambertDim == 256);
  REQUIRE(config.numColors == 32);
  REQUIRE(config.minScale == Approx(0.0));
  REQUIRE(config.maxScale == Approx(1.0));
  REQUIRE(config.sphereRadius == Approx(1.0f));
  REQUIRE(config.discrete == false);
  REQUIRE(config.discreteHeatMap == false);
  REQUIRE(config.colorMap.empty());
  REQUIRE(config.labels.empty());
  REQUIRE(config.order.size() == 3);
  REQUIRE(config.order[0] == 0);
  REQUIRE(config.order[1] == 1);
  REQUIRE(config.order[2] == 2);
  REQUIRE(config.flipFinalImage == true);
  REQUIRE(config.layoutType == PoleFigureLayoutType::Horizontal);
  REQUIRE(config.laueOpsIndex == 0);
  REQUIRE(config.phaseName.empty());
  REQUIRE(config.phaseNumber == 1);
  REQUIRE(config.title.empty());
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureCompositorTest::LayoutMetrics_Horizontal", "[EbsdLib][PoleFigureCompositorTest]")
{
  CompositePoleFigureConfiguration_t config;
  config.imageDim = 256;
  config.layoutType = PoleFigureLayoutType::Horizontal;

  LayoutMetrics metrics = PoleFigureCompositor::computeLayoutMetrics(config);

  const float imageDim = 256.0f;
  const float expectedFontPtSize = imageDim / 16.0f;   // 16.0f
  const float expectedMargins = imageDim / 32.0f;       // 8.0f

  REQUIRE(metrics.fontPtSize == Approx(expectedFontPtSize));
  REQUIRE(metrics.margins == Approx(expectedMargins));

  // subCanvasWidth > imageDim (includes xCharWidth from font measurement)
  REQUIRE(metrics.subCanvasWidth > imageDim);

  // subCanvasHeight = margins + fontPtSize + imageDim + fontPtSize*2 + margins*2
  const float expectedSubCanvasHeight = expectedMargins + expectedFontPtSize + imageDim + expectedFontPtSize * 2.0f + expectedMargins * 2.0f;
  REQUIRE(metrics.subCanvasHeight == Approx(expectedSubCanvasHeight));

  // Horizontal: pageWidth = subCanvasWidth * 4, pageHeight contains one row
  REQUIRE(metrics.pageWidth == static_cast<int32_t>(metrics.subCanvasWidth) * 4);
  REQUIRE(metrics.pageHeight > 0);

  // All 4 origins should have the same Y (side-by-side in a row)
  const float y0 = metrics.origins[0][1];
  REQUIRE(metrics.origins[1][1] == Approx(y0));
  REQUIRE(metrics.origins[2][1] == Approx(y0));
  REQUIRE(metrics.origins[3][1] == Approx(y0));

  // X positions should increase by subCanvasWidth each step
  REQUIRE(metrics.origins[0][0] == Approx(0.0f));
  REQUIRE(metrics.origins[1][0] == Approx(metrics.subCanvasWidth));
  REQUIRE(metrics.origins[2][0] == Approx(metrics.subCanvasWidth * 2.0f));
  REQUIRE(metrics.origins[3][0] == Approx(metrics.subCanvasWidth * 3.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureCompositorTest::LayoutMetrics_Vertical", "[EbsdLib][PoleFigureCompositorTest]")
{
  CompositePoleFigureConfiguration_t config;
  config.imageDim = 256;
  config.layoutType = PoleFigureLayoutType::Vertical;

  LayoutMetrics metrics = PoleFigureCompositor::computeLayoutMetrics(config);

  const float imageDim = 256.0f;
  const float expectedFontPtSize = imageDim / 16.0f;
  const float expectedMargins = imageDim / 32.0f;

  REQUIRE(metrics.fontPtSize == Approx(expectedFontPtSize));
  REQUIRE(metrics.margins == Approx(expectedMargins));
  REQUIRE(metrics.subCanvasWidth > imageDim);

  // Vertical: pageWidth = subCanvasWidth (single column)
  REQUIRE(metrics.pageWidth == static_cast<int32_t>(metrics.subCanvasWidth));
  REQUIRE(metrics.pageHeight > 0);

  // All 4 origins should have the same X = 0 (stacked in a column)
  REQUIRE(metrics.origins[0][0] == Approx(0.0f));
  REQUIRE(metrics.origins[1][0] == Approx(0.0f));
  REQUIRE(metrics.origins[2][0] == Approx(0.0f));
  REQUIRE(metrics.origins[3][0] == Approx(0.0f));

  // Y positions should increase by subCanvasHeight each step
  const float topY = expectedMargins + expectedFontPtSize;
  REQUIRE(metrics.origins[0][1] == Approx(topY));
  REQUIRE(metrics.origins[1][1] == Approx(topY + metrics.subCanvasHeight));
  REQUIRE(metrics.origins[2][1] == Approx(topY + metrics.subCanvasHeight * 2.0f));
  REQUIRE(metrics.origins[3][1] == Approx(topY + metrics.subCanvasHeight * 3.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureCompositorTest::LayoutMetrics_Square", "[EbsdLib][PoleFigureCompositorTest]")
{
  CompositePoleFigureConfiguration_t config;
  config.imageDim = 256;
  config.layoutType = PoleFigureLayoutType::Square;

  LayoutMetrics metrics = PoleFigureCompositor::computeLayoutMetrics(config);

  const float imageDim = 256.0f;
  const float expectedFontPtSize = imageDim / 16.0f;
  const float expectedMargins = imageDim / 32.0f;

  REQUIRE(metrics.fontPtSize == Approx(expectedFontPtSize));
  REQUIRE(metrics.margins == Approx(expectedMargins));
  REQUIRE(metrics.subCanvasWidth > imageDim);

  // Square: pageWidth = subCanvasWidth * 2 (2 columns)
  REQUIRE(metrics.pageWidth == static_cast<int32_t>(metrics.subCanvasWidth) * 2);
  REQUIRE(metrics.pageHeight > 0);

  // Top row: origins[0] and origins[1] share the same Y
  REQUIRE(metrics.origins[0][0] == Approx(0.0f));
  REQUIRE(metrics.origins[1][0] == Approx(metrics.subCanvasWidth));
  REQUIRE(metrics.origins[0][1] == Approx(metrics.origins[1][1]));

  // Bottom row: origins[2] and origins[3] share the same Y
  REQUIRE(metrics.origins[2][0] == Approx(0.0f));
  REQUIRE(metrics.origins[3][0] == Approx(metrics.subCanvasWidth));
  REQUIRE(metrics.origins[2][1] == Approx(metrics.origins[3][1]));

  // Bottom row Y is one subCanvasHeight below top row Y
  REQUIRE(metrics.origins[2][1] == Approx(metrics.origins[0][1] + metrics.subCanvasHeight));
}
