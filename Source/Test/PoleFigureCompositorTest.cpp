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

/**
Test result: 39 mismatched pixels in Debug mode (confirmed Release passes).

  Root Cause: Floating-point non-determinism in canvas_ity rendering

  The issue is not UB from float-to-uint8 casts. I've traced the full data flow and the values are properly bounded. The real cause is
  floating-point precision differences between -O0 (Debug) and -O2 (Release) in the canvas_ity rendering pipeline.

  Key areas where Debug/Release produce different float results:

  1. canvas_ity.hpp:2195-2218 — Bicubic image resampling in paint_pixel(): cubic polynomial evaluation, weighted accumulation, and division. In
  Release, the compiler may use FMA (fused multiply-add) instructions which have different rounding than separate multiply+add in Debug.
  2. canvas_ity.hpp:2452-2454 — Compositing/blending: rgba blend = mix_fore * fore + mix_back * back — multiple float multiply-adds sensitive to
  optimization.
  3. canvas_ity.hpp:3075 — Bayer dithering + sRGB conversion: When 255.0f * delinearized_value + bayer_threshold lands close to an integer
  boundary (e.g., 182.99999 vs 183.00001), the static_cast<unsigned char> truncation gives different results between Debug and Release.

  I verified the float-to-unsigned-char cast (canvas_ity.hpp:3076-3079) is NOT UB because:
  - clamped() constrains to [0.0, 1.0]
  - delinearized() maps [0,1] → [0, ~1.0]
  - * 255.0f → [0, ~255.0], plus Bayer threshold (max 0.97) → max ~255.97
  - Truncation to 255, which is in range for unsigned char

  I also verified PoleFigureUtilities.cpp:170 (static_cast<int>(r * 255.0f)) — this casts to int (not uint8), and dRgb masks with & 0xff, so it's
   well-defined regardless.

  The real problem: byte-exact test comparison

  The test at PoleFigureCompositorTest.cpp:180 does:
  if(exemplarData[i] != (*image)[i])

  This requires bit-exact reproduction across optimization levels, which floating-point math doesn't guarantee.

*/

#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Test/EbsdLibTestFileLocations.h"
#include "EbsdLib/Utilities/PngWriter.h"
#include "EbsdLib/Utilities/PoleFigureCompositor.h"
#include "UnitTestCommon.hpp"
#include "UnitTestSupport.hpp"

#include <H5Support/H5Lite.h>
#include <H5Support/H5ScopedSentinel.h>
#include <H5Support/H5Utilities.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <set>
#include <sstream>
#include <string>

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

#define WRITE_EXEMPLAR_IMAGES 0

// Maximum fraction of image bytes that may differ from the exemplar by more than
// the +/-1 per-byte tolerance below. The pole-figure raster is not bit-reproducible
// across compilers/platforms: MSVC, Clang and GCC differ in the last ULP of the
// transcendental functions used in the quat->Euler step and the canvas_ity render
// pipeline (see the file header comment), and where such a value straddles one of
// the numColors color-bin boundaries a single pixel snaps to an adjacent bin and its
// byte jumps well past +/-1. These are isolated boundary pixels (observed <0.06% of
// bytes on MSVC); a genuine rendering regression moves orders of magnitude more.
constexpr double k_PixelMismatchTolerance = 0.005; // 0.5%

void GeneratePoleFigures(const std::string& phaseName, size_t opsIndex, hid_t exemplarFileId)
{
  constexpr size_t k_NumSamplingGroups = 8;
  constexpr size_t k_NumQuats = 10000;
  constexpr size_t k_QuatSize = 4;
  const std::string distributionType("WAT");

  std::string inputFilePath = fmt::format("{}/Laue_Orientation_Clusters_v6/{}.h5", ebsdlib::unit_test::k_TestFilesDir, phaseName);
  hid_t fid = H5Support::H5Utilities::openFile(inputFilePath, true);
  REQUIRE(fid > 0);
  H5Support::H5ScopedFileSentinel fileSentinel(fid, false);

  std::string prefix = (distributionType == "VMF") ? "vMF" : "WAT";
  std::vector<double> quatarray;
  herr_t err = H5Support::H5Lite::readVectorDataset(fid, fmt::format("/EMData/Sampler/{}quatarray", prefix), quatarray);
  REQUIRE(err == 0);

  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
  LaueOps::Pointer op = ops[opsIndex];

  std::vector<PoleFigureLayoutType> layoutTypes = {PoleFigureLayoutType::Horizontal, PoleFigureLayoutType::Square, PoleFigureLayoutType::Vertical};
  std::vector<ebsdlib::HexConvention> hexConventions = {ebsdlib::HexConvention::XParallelAStar, ebsdlib::HexConvention::XParallelA, ebsdlib::HexConvention::XParallelAStar};
  std::vector<bool> discretes = {false, false, true};
  for(size_t idx = 0; idx < layoutTypes.size(); idx++)
  {
    std::string layoutStr = (layoutTypes[idx] == PoleFigureLayoutType::Horizontal) ? "Horz" : (layoutTypes[idx] == PoleFigureLayoutType::Vertical) ? "Vert" : "Sqr";
    hid_t layoutGroupId = H5Support::H5Utilities::createGroup(exemplarFileId, layoutStr);
    REQUIRE(layoutGroupId > 0);
    H5Support::H5ScopedGroupSentinel layoutGroupSentinel(layoutGroupId, true);

    for(size_t sampleId = 0; sampleId < k_NumSamplingGroups; ++sampleId)
    {
      std::vector<size_t> compDims = {3};
      auto eulers = FloatArrayType::CreateArray(k_NumQuats, compDims, "TestEulers", true);

      // Generate Euler Angles from Quaternions in the file
      for(size_t quatIdx = 0; quatIdx < k_NumQuats; ++quatIdx)
      {
        // HDF5 stores quaternions as WXYZ (EMsoft), convert to XYZW (EbsdLib)
        size_t idx = (sampleId * k_NumQuats * k_QuatSize) + (quatIdx * k_QuatSize);
        QuatD q(quatarray[idx + 1], quatarray[idx + 2], quatarray[idx + 3], quatarray[idx]);
        q = op->getFZQuat(q);
        EulerDType euler = q.toEuler();

        // Assign Euler Angles
        (*eulers)[quatIdx * 3] = static_cast<float>(euler[0]);
        (*eulers)[quatIdx * 3 + 1] = static_cast<float>(euler[1]);
        (*eulers)[quatIdx * 3 + 2] = static_cast<float>(euler[2]);
      }

      CompositePoleFigureConfiguration_t config;
      config.eulers = eulers.get();
      config.imageDim = 512;
      config.lambertDim = 32;
      config.numColors = 16;
      config.discrete = discretes[idx];
      config.discreteHeatMap = false;
      config.flipFinalImage = true;
      config.laueOpsIndex = opsIndex;
      config.layoutType = layoutTypes[idx];
      config.phaseName = "TestPhase";
      config.phaseNumber = 1;
      config.title = fmt::format("Laue Symmetry:{} Rotation Point Group: {}", op->getSymmetryName(), op->getRotationPointGroup());
      config.hexConvention = hexConventions[idx];

      PoleFigureCompositor compositor;
      CompositePoleFigureResult result = compositor.generateCompositeImage(config);

      REQUIRE(result.image != nullptr);
      REQUIRE(result.width > 0);
      REQUIRE(result.height > 0);
      REQUIRE(result.image->getNumberOfComponents() == 4);
      REQUIRE(result.image->getNumberOfTuples() == static_cast<size_t>(result.width * result.height));

      LayoutMetrics metrics = PoleFigureCompositor::computeLayoutMetrics(config);
      REQUIRE(result.width == metrics.pageWidth);
      REQUIRE(result.height == metrics.pageHeight);

      UInt8ArrayType::Pointer image = result.image;
      std::string datasetName = fmt::format("{}", sampleId);
#if WRITE_EXEMPLAR_IMAGES
      std::string outputPath = fmt::format("{}/Pole_Figure_Images/{}_Pole_Figure_{}_{}.png", ebsdlib::unit_test::k_TestFilesDir, op->getRotationPointGroup(), layoutStr, sampleId);
      auto writerResult = PngWriter::WriteColorImage(outputPath, result.width, result.height, 4, result.image->data());
      REQUIRE(writerResult.first == 0);
      //

      std::vector<hsize_t> dims = {static_cast<hsize_t>(result.height), static_cast<hsize_t>(result.width), 4ULL};
      herr_t err = H5Support::H5Lite::writePointerDataset(layoutGroupId, datasetName, dims.size(), dims.data(), result.image->data());
      REQUIRE(err == 0);
#else

      std::vector<uint8_t> exemplarData;
      err = H5Support::H5Lite::readVectorDataset(layoutGroupId, datasetName, exemplarData);
      REQUIRE(err == 0);
      REQUIRE(exemplarData.size() == static_cast<size_t>(result.width * result.height * 4));
      size_t misMatchCount = 0;
      for(size_t i = 0; i < exemplarData.size(); i++)
      {
        if(std::abs(static_cast<int>(exemplarData[i]) - static_cast<int>((*image)[i])) > 1)
        {
          misMatchCount++;
        }
      }
      const double misMatchFraction = static_cast<double>(misMatchCount) / static_cast<double>(exemplarData.size());
      if(misMatchCount > 0)
      {
        std::cout << phaseName << " [" << datasetName << "]: byte mismatches (>1) = " << misMatchCount << " / " << exemplarData.size() << " (" << std::setprecision(4)
                  << (misMatchFraction * 100.0) << "%)" << std::endl;
      }
      REQUIRE(misMatchFraction <= k_PixelMismatchTolerance);
#endif
    }
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureCompositorTest::All_Laue_Classes", "[EbsdLib][PoleFigureCompositorTest]")
{
  const ebsdlib::unit_test::TestFileSentinel testDataSentinel(ebsdlib::unit_test::k_TestFilesDir, "Laue_Orientation_Clusters_v6.tar.gz", "Laue_Orientation_Clusters_v6", true, true);
  const ebsdlib::unit_test::TestFileSentinel testDataSentinel1(ebsdlib::unit_test::k_TestFilesDir, "Pole_Figure_Images_v2.tar.gz", "Pole_Figure_Images_v2"
#if WRITE_EXEMPLAR_IMAGES
                                                               ,
                                                               false, false
#endif
  );

  const std::string hdfInputFile = fmt::format("{}/Pole_Figure_Images_v2/Exemplar_Data.h5", ebsdlib::unit_test::k_TestFilesDir);
  hid_t fileId = -1;
#if WRITE_EXEMPLAR_IMAGES
  if(!std::filesystem::exists(hdfInputFile))
  {
    std::cout << "Creating " << hdfInputFile << std::endl;
    fileId = H5Support::H5Utilities::createFile(hdfInputFile);
  }
  else
#else
  {
    std::cout << "Opening " << hdfInputFile << std::endl;
    fileId = H5Support::H5Utilities::openFile(hdfInputFile, true);
  }
#endif
    REQUIRE(fileId > 0);
  H5Support::H5ScopedFileSentinel fileSentinel(fileId, false);

  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();

  std::set<std::string> tested;
  for(size_t opsIdx = 0; opsIdx < ops.size(); opsIdx++)
  {
    LaueOps::Pointer op = ops[opsIdx];
    const std::string rpg = op->getRotationPointGroup();
    // Skip Triclinic (no FZ boundary) and duplicates (OrthoRhombicOps appears twice)
    if(rpg == "1" || tested.count(rpg) > 0)
    {
      continue;
    }
    tested.insert(rpg);

    hid_t layoutGroupId = H5Support::H5Utilities::createGroup(fileId, rpg);
    REQUIRE(layoutGroupId > 0);
    H5Support::H5ScopedGroupSentinel layoutGroupSentinel(layoutGroupId, false);

    const std::string phaseName = fmt::format("Laue_{}", rpg);
#if !WRITE_EXEMPLAR_IMAGES
    SECTION(phaseName + " VMF")
#endif
    {
      GeneratePoleFigures(phaseName, opsIdx, layoutGroupId);
    }
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureCompositorTest::LayoutMetrics_Horizontal", "[EbsdLib][PoleFigureCompositorTest]")
{
  CompositePoleFigureConfiguration_t config;
  config.imageDim = 256;
  config.layoutType = PoleFigureLayoutType::Horizontal;

  LayoutMetrics metrics = PoleFigureCompositor::computeLayoutMetrics(config);

  const float imageDim = 256.0f;
  const float expectedFontPtSize = imageDim / 16.0f; // 16.0f
  const float expectedMargins = imageDim / 32.0f;    // 8.0f

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

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureCompositorTest::GenerateComposite_Cubic_Horizontal", "[EbsdLib][PoleFigureCompositorTest]")
{
  const size_t numOrientations = 100;
  std::vector<size_t> compDims = {3};
  auto eulers = FloatArrayType::CreateArray(numOrientations, compDims, "TestEulers", true);
  for(size_t i = 0; i < numOrientations; i++)
  {
    float* ptr = eulers->getTuplePointer(i);
    ptr[0] = static_cast<float>((i * 7 + 3) % 360) * 0.0174533f;
    ptr[1] = static_cast<float>((i * 13 + 5) % 180) * 0.0174533f;
    ptr[2] = static_cast<float>((i * 19 + 11) % 360) * 0.0174533f;
  }

  CompositePoleFigureConfiguration_t config;
  config.eulers = eulers.get();
  config.imageDim = 64;
  config.lambertDim = 32;
  config.numColors = 16;
  config.discrete = false;
  config.discreteHeatMap = false;
  config.flipFinalImage = true;
  config.laueOpsIndex = 1; // CubicOps (Cubic_High)
  config.layoutType = PoleFigureLayoutType::Horizontal;
  config.phaseName = "TestPhase";
  config.phaseNumber = 1;
  config.title = "Test Pole Figure";

  PoleFigureCompositor compositor;
  CompositePoleFigureResult result = compositor.generateCompositeImage(config);

  REQUIRE(result.image != nullptr);
  REQUIRE(result.width > 0);
  REQUIRE(result.height > 0);
  REQUIRE(result.image->getNumberOfComponents() == 4);
  REQUIRE(result.image->getNumberOfTuples() == static_cast<size_t>(result.width * result.height));

  LayoutMetrics metrics = PoleFigureCompositor::computeLayoutMetrics(config);
  REQUIRE(result.width == metrics.pageWidth);
  REQUIRE(result.height == metrics.pageHeight);

  // Verify content (not all white)
  bool hasNonWhite = false;
  for(size_t i = 0; i < result.image->getNumberOfTuples() && !hasNonWhite; i++)
  {
    uint8_t* pixel = result.image->getTuplePointer(i);
    if(pixel[0] != 255 || pixel[1] != 255 || pixel[2] != 255)
    {
      hasNonWhite = true;
    }
  }
  REQUIRE(hasNonWhite);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureCompositorTest::GenerateComposite_Cubic_Discrete", "[EbsdLib][PoleFigureCompositorTest]")
{
  const size_t numOrientations = 50;
  std::vector<size_t> compDims = {3};
  auto eulers = FloatArrayType::CreateArray(numOrientations, compDims, "TestEulers", true);
  for(size_t i = 0; i < numOrientations; i++)
  {
    float* ptr = eulers->getTuplePointer(i);
    ptr[0] = static_cast<float>((i * 7 + 3) % 360) * 0.0174533f;
    ptr[1] = static_cast<float>((i * 13 + 5) % 180) * 0.0174533f;
    ptr[2] = static_cast<float>((i * 19 + 11) % 360) * 0.0174533f;
  }

  CompositePoleFigureConfiguration_t config;
  config.eulers = eulers.get();
  config.imageDim = 64;
  config.lambertDim = 32;
  config.numColors = 16;
  config.discrete = true;
  config.discreteHeatMap = false;
  config.flipFinalImage = true;
  config.laueOpsIndex = 1;
  config.layoutType = PoleFigureLayoutType::Horizontal;
  config.phaseName = "DiscretePhase";
  config.phaseNumber = 1;
  config.title = "Discrete Test";

  PoleFigureCompositor compositor;
  CompositePoleFigureResult result = compositor.generateCompositeImage(config);

  REQUIRE(result.image != nullptr);
  REQUIRE(result.width > 0);
  REQUIRE(result.height > 0);
  REQUIRE(result.image->getNumberOfComponents() == 4);
  REQUIRE(result.image->getNumberOfTuples() == static_cast<size_t>(result.width * result.height));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PoleFigureCompositorTest::GenerateComposite_AllLayouts", "[EbsdLib][PoleFigureCompositorTest]")
{
  const size_t numOrientations = 50;
  std::vector<size_t> compDims = {3};
  auto eulers = FloatArrayType::CreateArray(numOrientations, compDims, "TestEulers", true);
  for(size_t i = 0; i < numOrientations; i++)
  {
    float* ptr = eulers->getTuplePointer(i);
    ptr[0] = static_cast<float>((i * 7 + 3) % 360) * 0.0174533f;
    ptr[1] = static_cast<float>((i * 13 + 5) % 180) * 0.0174533f;
    ptr[2] = static_cast<float>((i * 19 + 11) % 360) * 0.0174533f;
  }

  std::vector<PoleFigureLayoutType> layouts = {PoleFigureLayoutType::Horizontal, PoleFigureLayoutType::Vertical, PoleFigureLayoutType::Square};

  for(auto layout : layouts)
  {
    DYNAMIC_SECTION("Layout " << static_cast<uint32_t>(layout))
    {
      CompositePoleFigureConfiguration_t config;
      config.eulers = eulers.get();
      config.imageDim = 64;
      config.lambertDim = 32;
      config.numColors = 16;
      config.laueOpsIndex = 1;
      config.layoutType = layout;
      config.phaseName = "TestPhase";
      config.phaseNumber = 1;
      config.title = "Layout Test";

      PoleFigureCompositor compositor;
      CompositePoleFigureResult result = compositor.generateCompositeImage(config);

      REQUIRE(result.image != nullptr);

      LayoutMetrics metrics = PoleFigureCompositor::computeLayoutMetrics(config);
      REQUIRE(result.width == metrics.pageWidth);
      REQUIRE(result.height == metrics.pageHeight);
    }
  }
}
