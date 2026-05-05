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

#include "EbsdLib/LaueOps/CubicOps.h"
#include "EbsdLib/LaueOps/HexagonalOps.h"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Test/EbsdLibTestFileLocations.h"
#include "EbsdLib/Texture/StatsGen.hpp"
#include "EbsdLib/Texture/Texture.hpp"
#include "EbsdLib/Utilities/PngWriter.h"
#include "EbsdLib/Utilities/PoleFigureCompositor.h"
#include "UnitTestCommon.hpp"
#include "UnitTestSupport.hpp"

#include <fmt/format.h>

#include <fstream>
#include <iostream>
#include <vector>

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
// TODO: THIS TEST SHOULD BE UPDATED TO ACTUALLY COMPARE RESULTS AGAINST A KNOWN GOOD DATA SET
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#define POPULATE_DATA(i, e1, e2, e3, w, s)                                                                                                                                                             \
  e1s[i] = e1;                                                                                                                                                                                         \
  e2s[i] = e2;                                                                                                                                                                                         \
  e3s[i] = e3;                                                                                                                                                                                         \
  weights[i] = w;                                                                                                                                                                                      \
  sigmas[i] = s;

using namespace ebsdlib;
// -----------------------------------------------------------------------------
template <typename T>
void Print_Coord(const T* om)
{
  printf("Coord:% 3.16f % 3.16f % 3.16f\n", om[0], om[1], om[2]);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ODFTest", "[EbsdLib][ODFTest]")
{

  // Start with degrees
  Texture::ODFTableEntry entry0 = {{180.0, 90.0, 0.0}, 50000.0, 0.05};

  const Texture::ODFTableEntries odfTableEntries = {
      {{entry0.euler[0] * ebsdlib::constants::k_PiOver180D, entry0.euler[1] * ebsdlib::constants::k_PiOver180D, entry0.euler[2] * ebsdlib::constants::k_PiOver180D}, entry0.weight, entry0.sigma}
#if 0
    ,
                                                    {{59 * ebsdlib::constants::k_PiOver180D, 37 * ebsdlib::constants::k_PiOver180D, 63 * ebsdlib::constants::k_PiOver180D}, 1000.0, 1.0}
#endif
  };

  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
  uint32_t opsIndex = 0; // Hexagonal-High
  LaueOps::Pointer op = ops[opsIndex];
  // Calculate the ODF Data
  using OdfValueType = float;
  using OdfContainerType = std::vector<OdfValueType>;

  OdfContainerType odf = Texture::CalculateODFData<OdfValueType, HexagonalOps, OdfContainerType>(odfTableEntries, true);

  using EulerContainerType = std::vector<OdfValueType>;
  size_t numSamplePoints = 500;
  EulerContainerType eulers = StatsGen::GenODFPlotData<OdfValueType, HexagonalOps, OdfContainerType, EulerContainerType>(odf, numSamplePoints);

  fs::path dir = fmt::format("{}/ODFTest", ebsdlib::unit_test::k_TestTempDir);
  if(fs::exists(dir) == false)
  {
    fs::create_directories(dir);
  }

  // Export sampled Euler angles (degrees) for MTEX comparison. One row per orientation: phi1, Phi, phi2
  {
    std::string csvPath = fmt::format("{}/ODFTest/ODFTest_Eulers_deg.csv", ebsdlib::unit_test::k_TestTempDir);
    std::ofstream csv(csvPath);
    csv << "phi1,Phi,phi2\n";
    for(size_t i = 0; i < numSamplePoints; ++i)
    {
      csv << eulers[3 * i] * 180.0 / M_PI << "," << eulers[3 * i + 1] * 180.0 / M_PI << "," << eulers[3 * i + 2] * 180.0 / M_PI << "\n";
    }
    std::cout << "Wrote Euler CSV: " << csvPath << std::endl;
  }

  ebsdlib::FloatArrayType::Pointer poleFigureEulersPtr = ebsdlib::FloatArrayType::FromStdVector(eulers, numSamplePoints, 3ULL, "Eulers");
  ebsdlib::CompositePoleFigureConfiguration_t config;
  config.eulers = poleFigureEulersPtr.get();
  config.imageDim = 512;
  config.lambertDim = 128;
  config.numColors = 16;
  config.discrete = true;
  config.discreteHeatMap = false;
  // flipFinalImage uses the default (true); flips image Y so sample +Y points up,
  // matching MTEX convention (X east, Y north, Z out of page).
  config.laueOpsIndex = opsIndex;
  config.layoutType = ebsdlib::PoleFigureLayoutType::Horizontal;
  config.phaseName = "EbsdLib ODF Test";
  config.phaseNumber = 1;
  config.title = fmt::format("{} <{}, {}, {}> ", op->getSymmetryName(), entry0.euler[0], entry0.euler[1], entry0.euler[2]);

  PoleFigureCompositor compositor;
  CompositePoleFigureResult result = compositor.generateCompositeImage(config);

  std::string outputPath = fmt::format("{}/ODFTest/Pole_Figure_{}.png", ebsdlib::unit_test::k_TestTempDir, op->getRotationPointGroup());

  auto writerResult = PngWriter::WriteColorImage(outputPath, result.width, result.height, 4, result.image->data());
  REQUIRE(writerResult.first == 0);
}

TEST_CASE("ebsdlib::ODFTest::TestRotation", "[EbsdLib][ODFTest]")
{
  float phi1 = 0.0f * ebsdlib::constants::k_PiOver180D;
  float PHI = 180.0f;
  float phi2 = 0.0f;

  // float ga[3][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};

  Matrix3X3<float> ga = EulerFType(phi1, PHI, phi2).toOrientationMatrix().toGMatrix();

  // OrientationTransformation::eu2om<OrientationF, OrientationF>(OrientationF(phi1, PHI, phi2)).toGMatrix(ga);

  Matrix3X1 coordsRotated = {0.0f, 0.0f, 0.0f};
  Matrix3X1 coords = {0.0f, 0.0f, 0.0f};
  float xc = -0.0;
  float yc = -5.0;
  float zc = 0.0;
  coords[0] = coords[0] - xc;
  coords[1] = coords[1] - yc;
  coords[2] = coords[2] - zc;

  coordsRotated = ga * coords;

  Print_Coord<float>(coords.data());
  Print_Coord<float>(coordsRotated.data());
}
