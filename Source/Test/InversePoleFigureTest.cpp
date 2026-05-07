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
 * The code contained herein was partially funded by the following contracts:
 *    United States Air Force Prime Contract FA8650-07-D-5800
 *    United States Air Force Prime Contract FA8650-10-D-5210
 *    United States Prime Contract Navy N00173-07-C-2068
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Utilities/InversePoleFigureUtilities.h"

#include <cmath>
#include <random>
#include <vector>

using namespace ebsdlib;

namespace
{
// Helper to generate random Euler angles (in radians)
ebsdlib::FloatArrayType::Pointer generateRandomEulers(size_t numOrientations, unsigned int seed = 42)
{
  std::vector<size_t> cDims = {3};
  auto eulers = ebsdlib::FloatArrayType::CreateArray(numOrientations, cDims, "EulerAngles", true);

  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> phi1Dist(0.0f, static_cast<float>(ebsdlib::constants::k_2PiD));
  std::uniform_real_distribution<float> phiDist(0.0f, static_cast<float>(ebsdlib::constants::k_PiD));
  std::uniform_real_distribution<float> phi2Dist(0.0f, static_cast<float>(ebsdlib::constants::k_2PiD));

  for(size_t i = 0; i < numOrientations; i++)
  {
    float* ptr = eulers->getTuplePointer(i);
    ptr[0] = phi1Dist(gen);
    ptr[1] = phiDist(gen);
    ptr[2] = phi2Dist(gen);
  }
  return eulers;
}

// Helper to generate single-crystal Euler angles (all orientations identical)
ebsdlib::FloatArrayType::Pointer generateSingleCrystalEulers(size_t numOrientations, float e0, float e1, float e2)
{
  std::vector<size_t> cDims = {3};
  auto eulers = ebsdlib::FloatArrayType::CreateArray(numOrientations, cDims, "EulerAngles", true);

  for(size_t i = 0; i < numOrientations; i++)
  {
    float* ptr = eulers->getTuplePointer(i);
    ptr[0] = e0;
    ptr[1] = e1;
    ptr[2] = e2;
  }
  return eulers;
}

// Standard orthogonal sample directions: RD=[1,0,0], TD=[0,1,0], ND=[0,0,1]
InversePoleFigureConfiguration_t createDefaultConfig(ebsdlib::FloatArrayType* eulers)
{
  InversePoleFigureConfiguration_t config;
  config.eulers = eulers;
  config.sampleDirections = {Matrix3X1D(1.0, 0.0, 0.0), Matrix3X1D(0.0, 1.0, 0.0), Matrix3X1D(0.0, 0.0, 1.0)};
  config.imageWidth = 64;
  config.imageHeight = 64;
  config.lambertDim = 32;
  config.numColors = 32;
  config.colorMap = "Default";
  config.normalizeMRD = true;
  config.labels = {"RD", "TD", "ND"};
  config.phaseName = "TestPhase";
  config.FlipFinalImage = false;
  return config;
}

} // namespace

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::InversePoleFigureTest::Configuration_Fields", "[EbsdLib][InversePoleFigureTest]")
{
  InversePoleFigureConfiguration_t config;
  config.eulers = nullptr;
  config.sampleDirections = {Matrix3X1D(1.0, 0.0, 0.0), Matrix3X1D(0.0, 1.0, 0.0), Matrix3X1D(0.0, 0.0, 1.0)};
  config.imageWidth = 128;
  config.imageHeight = 128;
  config.lambertDim = 64;
  config.numColors = 32;
  config.colorMap = "Default";
  config.normalizeMRD = true;
  config.labels = {"RD", "TD", "ND"};
  config.phaseName = "Phase1";
  config.FlipFinalImage = false;

  REQUIRE(config.imageWidth == 128);
  REQUIRE(config.imageHeight == 128);
  REQUIRE(config.lambertDim == 64);
  REQUIRE(config.numColors == 32);
  REQUIRE(config.normalizeMRD == true);
  REQUIRE(config.labels.size() == 3);
  REQUIRE(config.phaseName == "Phase1");
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::InversePoleFigureTest::ComputeIPFDirections_Cubic", "[EbsdLib][InversePoleFigureTest]")
{
  auto eulers = generateRandomEulers(100);
  auto ops = LaueOps::GetAllOrientationOps();
  // CubicOps is at index 1
  auto& cubicOps = *ops[1];

  Matrix3X1D nd(0.0, 0.0, 1.0); // Normal direction
  auto directions = InversePoleFigureUtilities::computeIPFDirections(cubicOps, eulers.get(), nd);

  REQUIRE(directions != nullptr);
  REQUIRE(directions->getNumberOfTuples() > 0);
  REQUIRE(directions->getNumberOfComponents() == 3);

  // All returned directions should be on the unit sphere (magnitude ~1.0)
  for(size_t i = 0; i < directions->getNumberOfTuples(); i++)
  {
    float* dir = directions->getTuplePointer(i);
    float mag = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    REQUIRE(mag == Approx(1.0f).margin(0.01f));
  }

  // All returned directions should have z >= 0 (upper hemisphere)
  for(size_t i = 0; i < directions->getNumberOfTuples(); i++)
  {
    float* dir = directions->getTuplePointer(i);
    REQUIRE(dir[2] >= -0.01f); // Allow small numerical tolerance
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::InversePoleFigureTest::ComputeIPFIntensity_Cubic", "[EbsdLib][InversePoleFigureTest]")
{
  auto eulers = generateRandomEulers(500);
  auto ops = LaueOps::GetAllOrientationOps();
  auto& cubicOps = *ops[1];

  Matrix3X1D nd(0.0, 0.0, 1.0);
  auto directions = InversePoleFigureUtilities::computeIPFDirections(cubicOps, eulers.get(), nd);

  int imageWidth = 64;
  int imageHeight = 64;
  int lambertDim = 32;

  auto intensity = InversePoleFigureUtilities::computeIPFIntensity(cubicOps, directions.get(), imageWidth, imageHeight, lambertDim, true);

  REQUIRE(intensity != nullptr);
  REQUIRE(intensity->getNumberOfTuples() == static_cast<size_t>(imageWidth * imageHeight));

  // Check that we have some pixels inside the SST (value >= 0) and some outside (value == -1)
  bool hasInsideSST = false;
  bool hasOutsideSST = false;
  double* dataPtr = intensity->getPointer(0);
  for(size_t i = 0; i < intensity->getNumberOfTuples(); i++)
  {
    if(dataPtr[i] >= 0.0)
    {
      hasInsideSST = true;
    }
    if(dataPtr[i] < 0.0)
    {
      hasOutsideSST = true;
    }
    if(hasInsideSST && hasOutsideSST)
    {
      break;
    }
  }
  REQUIRE(hasInsideSST);
  REQUIRE(hasOutsideSST);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::InversePoleFigureTest::CreateIPFColorImage", "[EbsdLib][InversePoleFigureTest]")
{
  int imageWidth = 16;
  int imageHeight = 16;
  int numColors = 16;

  // Create a test intensity image with some SST values and some -1 (outside)
  auto intensity = DoubleArrayType::CreateArray(static_cast<size_t>(imageWidth * imageHeight), {1ULL}, "Intensity", true);
  double* dataPtr = intensity->getPointer(0);
  for(int i = 0; i < imageWidth * imageHeight; i++)
  {
    if(i % 3 == 0)
    {
      dataPtr[i] = -1.0; // Outside SST
    }
    else
    {
      dataPtr[i] = static_cast<double>(i) / static_cast<double>(imageWidth * imageHeight);
    }
  }

  std::vector<size_t> cDims = {4};
  auto rgba = UInt8ArrayType::CreateArray(static_cast<size_t>(imageWidth * imageHeight), cDims, "RGBA", true);
  rgba->initializeWithZeros();

  InversePoleFigureUtilities::createIPFColorImage(intensity.get(), imageWidth, imageHeight, numColors, 0.0, 1.0, rgba.get());

  // Verify image was populated
  bool hasNonZero = false;
  bool hasWhite = false;
  for(size_t i = 0; i < rgba->getNumberOfTuples(); i++)
  {
    uint8_t* pixel = rgba->getTuplePointer(i);
    uint32_t rgbaVal = *reinterpret_cast<uint32_t*>(pixel);
    if(rgbaVal == 0xFFFFFFFF)
    {
      hasWhite = true;
    }
    else if(pixel[0] != 0 || pixel[1] != 0 || pixel[2] != 0 || pixel[3] != 0)
    {
      hasNonZero = true;
    }
  }
  REQUIRE(hasNonZero);
  REQUIRE(hasWhite);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::InversePoleFigureTest::GenerateInversePoleFigure_AllLaueClasses", "[EbsdLib][InversePoleFigureTest]")
{
  auto eulers = generateRandomEulers(200);
  auto ops = LaueOps::GetAllOrientationOps();

  for(size_t index = 0; index < 11; index++)
  {
    SECTION(ops[index]->getSymmetryName())
    {
      auto config = createDefaultConfig(eulers.get());
      auto images = ops[index]->generateInversePoleFigure(config);

      // Should return exactly 3 images
      REQUIRE(images.size() == 3);

      for(size_t imgIdx = 0; imgIdx < 3; imgIdx++)
      {
        REQUIRE(images[imgIdx] != nullptr);
        REQUIRE(images[imgIdx]->getNumberOfTuples() == static_cast<size_t>(config.imageWidth * config.imageHeight));
        REQUIRE(images[imgIdx]->getNumberOfComponents() == 4);

        // Verify the image has some non-white content (SST region should be colored)
        bool hasColoredPixel = false;
        for(size_t i = 0; i < images[imgIdx]->getNumberOfTuples(); i++)
        {
          uint32_t rgbaVal = *reinterpret_cast<uint32_t*>(images[imgIdx]->getTuplePointer(i));
          if(rgbaVal != 0xFFFFFFFF && rgbaVal != 0x00000000)
          {
            hasColoredPixel = true;
            break;
          }
        }
        REQUIRE(hasColoredPixel);
      }
    }
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::InversePoleFigureTest::GenerateInversePoleFigure_MRD_vs_Counts", "[EbsdLib][InversePoleFigureTest]")
{
  auto eulers = generateRandomEulers(200);
  auto ops = LaueOps::GetAllOrientationOps();
  auto& cubicOps = *ops[1]; // Cubic high

  // Test MRD mode
  auto configMRD = createDefaultConfig(eulers.get());
  configMRD.normalizeMRD = true;
  auto imagesMRD = cubicOps.generateInversePoleFigure(configMRD);
  REQUIRE(imagesMRD.size() == 3);

  // Test counts mode
  auto configCounts = createDefaultConfig(eulers.get());
  configCounts.normalizeMRD = false;
  auto imagesCounts = cubicOps.generateInversePoleFigure(configCounts);
  REQUIRE(imagesCounts.size() == 3);

  // Both should produce valid images
  for(size_t imgIdx = 0; imgIdx < 3; imgIdx++)
  {
    REQUIRE(imagesMRD[imgIdx] != nullptr);
    REQUIRE(imagesCounts[imgIdx] != nullptr);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::InversePoleFigureTest::SingleCrystalTexture_Cubic", "[EbsdLib][InversePoleFigureTest]")
{
  // All orientations are identity (0, 0, 0 Euler angles)
  // For ND=[0,0,1], the crystal direction in the SST should be [001]
  auto eulers = generateSingleCrystalEulers(500, 0.0f, 0.0f, 0.0f);
  auto ops = LaueOps::GetAllOrientationOps();
  auto& cubicOps = *ops[1];

  Matrix3X1D nd(0.0, 0.0, 1.0);
  auto directions = InversePoleFigureUtilities::computeIPFDirections(cubicOps, eulers.get(), nd);

  REQUIRE(directions != nullptr);
  REQUIRE(directions->getNumberOfTuples() == 500);

  // All directions should be very close to [001] = (0, 0, 1)
  for(size_t i = 0; i < directions->getNumberOfTuples(); i++)
  {
    float* dir = directions->getTuplePointer(i);
    REQUIRE(std::fabs(dir[0]) < 0.01f);
    REQUIRE(std::fabs(dir[1]) < 0.01f);
    REQUIRE(dir[2] == Approx(1.0f).margin(0.01f));
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::InversePoleFigureTest::ImageDimensions", "[EbsdLib][InversePoleFigureTest]")
{
  auto eulers = generateRandomEulers(100);
  auto ops = LaueOps::GetAllOrientationOps();
  auto& cubicOps = *ops[1];

  int testWidth = 128;
  int testHeight = 96;

  auto config = createDefaultConfig(eulers.get());
  config.imageWidth = testWidth;
  config.imageHeight = testHeight;

  auto images = cubicOps.generateInversePoleFigure(config);

  REQUIRE(images.size() == 3);
  for(auto& img : images)
  {
    REQUIRE(img->getNumberOfTuples() == static_cast<size_t>(testWidth * testHeight));
  }
}

// -----------------------------------------------------------------------------
// PR 2k regression test: InversePoleFigureConfiguration_t carries a
// HexConvention field, and generateAnnotatedIPFDensity threads it down into
// each per-figure annotateIPFImage call. The annotated density images include
// rendered Miller-index labels around the SST, and PR 2h made those labels
// convention-dependent — under X||a the +X corner reads <2-1-10>; under X||a*
// it reads <11-20>. Output bytes for a hex/trig phase MUST therefore differ
// between conventions; if they don't, conv is being silently dropped through
// the IPF density path the same way the PoleFigureCompositor was dropping it
// before PR 2g.
TEST_CASE("ebsdlib::InversePoleFigureTest::AnnotatedIPFDensity_PropagatesHexConvention", "[EbsdLib][InversePoleFigureTest]")
{
  // Need a hex/trig phase for the convention to matter. HexagonalOps is
  // CrystalStructure::Hexagonal_High = index 0.
  auto ops = LaueOps::GetAllOrientationOps();
  auto& hexOps = *ops[ebsdlib::CrystalStructure::Hexagonal_High];

  auto eulers = generateRandomEulers(64);

  InversePoleFigureConfiguration_t configAStar;
  configAStar.eulers = eulers.get();
  configAStar.sampleDirections = {Matrix3X1D(1.0, 0.0, 0.0), Matrix3X1D(0.0, 1.0, 0.0), Matrix3X1D(0.0, 0.0, 1.0)};
  configAStar.imageWidth = 64;
  configAStar.imageHeight = 64;
  configAStar.lambertDim = 16;
  configAStar.numColors = 16;
  configAStar.colorMap = "Default";
  configAStar.normalizeMRD = false;
  configAStar.labels = {"RD", "TD", "ND"};
  configAStar.phaseName = "TestHex";
  configAStar.FlipFinalImage = false;
  configAStar.hexConvention = ebsdlib::HexConvention::XParallelAStar;

  InversePoleFigureConfiguration_t configA = configAStar;
  configA.hexConvention = ebsdlib::HexConvention::XParallelA;

  auto imagesAStar = hexOps.generateAnnotatedIPFDensity(configAStar);
  auto imagesA = hexOps.generateAnnotatedIPFDensity(configA);

  REQUIRE(imagesAStar.size() == 3);
  REQUIRE(imagesA.size() == 3);
  REQUIRE(imagesAStar[0] != nullptr);
  REQUIRE(imagesA[0] != nullptr);
  REQUIRE(imagesAStar[0]->getNumberOfTuples() == imagesA[0]->getNumberOfTuples());

  // The annotated images must differ somewhere — the rendered Miller-index
  // text pixels are different between conventions.
  bool different = false;
  const size_t total = imagesAStar[0]->getSize();
  const uint8_t* pAStar = imagesAStar[0]->getPointer(0);
  const uint8_t* pA = imagesA[0]->getPointer(0);
  for(size_t i = 0; i < total; ++i)
  {
    if(pAStar[i] != pA[i])
    {
      different = true;
      break;
    }
  }
  CHECK(different);
}
