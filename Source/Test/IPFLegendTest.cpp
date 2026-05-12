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

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/LaueOps/CubicOps.h"
#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/FundamentalSectorGeometry.hpp"
#include "EbsdLib/Utilities/GriddedColorKey.hpp"
#include "EbsdLib/Utilities/NolzeHielscherColorKey.hpp"
#include "EbsdLib/Utilities/PUCMColorKey.hpp"
#include "EbsdLib/Utilities/PngWriter.h"
#include "EbsdLib/Utilities/TSLColorKey.hpp"

#include "EbsdLib/Test/EbsdLibTestFileLocations.h"
#include "UnitTestSupport.hpp"

#include <fmt/format.h>

#include <filesystem>
#include <fstream>
#include <set>

#define IMAGE_WIDTH 512
#define IMAGE_HEIGHT 512

using namespace ebsdlib;

// TODO: This unit test needs to compare the output to something that has been verified as correct

TEST_CASE("ebsdlib::IPFLegendTest", "[EbsdLib][IPFLegendTest]")
{

  fs::path dir = fmt::format("{}/IPFLegendTest", ebsdlib::unit_test::k_TestTempDir);
  if(fs::exists(dir) == false)
  {
    fs::create_directories(dir);
  }

  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();

  for(size_t index = 0; index < 11; index++)
  {
    SECTION(ops[index]->getSymmetryName())
    {
      ebsdlib::UInt8ArrayType::Pointer image = ops[index]->generateIPFTriangleLegend(IMAGE_WIDTH, false, ebsdlib::HexConvention::XParallelAStar);

      std::string outputFilePath = fmt::format("{}/IPFLegendTest/{}.png", ebsdlib::unit_test::k_TestTempDir, ops[index]->getNameOfClass());
      EnsureParentDirectoryExists(outputFilePath);
      auto result = PngWriter::WriteColorImage(outputFilePath, IMAGE_WIDTH, IMAGE_WIDTH, 3, image->data());
      REQUIRE(result.first == 0);
    }
  }
}

TEST_CASE("ebsdlib::IPFLegendTest::NolzeHielscherLegend", "[EbsdLib][IPFLegendTest]")
{
  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();

  for(size_t index = 0; index < 11; index++)
  {
    SECTION(ops[index]->getSymmetryName() + " NH Legend")
    {
      // Request the per-class NolzeHielscher legend (each LaueOps subclass owns
      // its own NH singleton built from its FundamentalSectorGeometry).
      auto legend = ops[index]->generateIPFTriangleLegend(64, false, ebsdlib::HexConvention::XParallelAStar, ebsdlib::ColorKeyKind::NolzeHielscher);
      REQUIRE(legend != nullptr);
      REQUIRE(legend->getNumberOfTuples() > 0);

      // Verify the image has some non-white pixels (NH key produces colors)
      bool hasNonWhitePixel = false;
      size_t numTuples = legend->getNumberOfTuples();
      for(size_t i = 0; i < numTuples; i++)
      {
        uint8_t* pixel = legend->getTuplePointer(i);
        // Legend is RGB (3 components after alpha removal)
        if(legend->getNumberOfComponents() == 3 || legend->getNumberOfComponents() == 4)
        {
          if(pixel[0] != 255 || pixel[1] != 255 || pixel[2] != 255)
          {
            hasNonWhitePixel = true;
            break;
          }
        }
      }
      REQUIRE(hasNonWhitePixel);
    }
  }
}

// -----------------------------------------------------------------------------
// Corner probe: for every Laue class, the TSL IPF color at the crystal c-axis
// direction (sample refDir pointing along crystal [001]/[0001]) must be pure
// red. computeIPFColor maps chi=0 -> R=1, G=0, B=0; this test is therefore a
// convention sanity check that catches:
//   - Euler-to-matrix sign flips (c-axis lands somewhere other than chi=0)
//   - Inversion/symmetry bugs that move the vertex off the triangle corner
//   - Color-key regressions in the TSL default
// It doesn't cover the interior of the triangle; for that, compare against
// the MTEX legends under Data/IPF_Legend/MTEX_Reference/.
TEST_CASE("ebsdlib::IPFLegendTest::CAxisIsRed", "[EbsdLib][IPFLegendTest]")
{
  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
  std::set<std::string> seen;

  double identityEuler[3] = {0.0, 0.0, 0.0};
  double cAxisSampleDir[3] = {0.0, 0.0, 1.0};

  for(size_t i = 0; i < ops.size(); ++i)
  {
    LaueOps::Pointer op = ops[i];
    const std::string rpg = op->getRotationPointGroup();
    if(seen.count(rpg) > 0)
    {
      continue;
    }
    seen.insert(rpg);

    Rgb color = op->generateIPFColor(identityEuler, cAxisSampleDir, false, ebsdlib::ColorKeyKind::TSL);
    int r = RgbColor::dRed(color);
    int g = RgbColor::dGreen(color);
    int b = RgbColor::dBlue(color);

    INFO(op->getSymmetryName() << " (" << rpg << ") c-axis -> RGB(" << r << ", " << g << ", " << b << ")");
    // Red-dominant, and green+blue should be low (pure-red triangle vertex)
    CHECK(r >= 200);
    CHECK(g <= 60);
    CHECK(b <= 60);
  }
}

// -----------------------------------------------------------------------------
// Per-Laue-class FundamentalSectorGeometry lookup so each Laue class's
// NolzeHielscherColorKey is constructed with its own sector instead of the
// cubicHigh placeholder used elsewhere in this file.
namespace
{
ebsdlib::FundamentalSectorGeometry SectorForRotationPointGroup(const std::string& rpg)
{
  if(rpg == "432")
  {
    return ebsdlib::FundamentalSectorGeometry::cubicHigh();
  }
  if(rpg == "23")
  {
    return ebsdlib::FundamentalSectorGeometry::cubicLow();
  }
  if(rpg == "622")
  {
    return ebsdlib::FundamentalSectorGeometry::hexagonalHigh();
  }
  if(rpg == "6")
  {
    return ebsdlib::FundamentalSectorGeometry::hexagonalLow();
  }
  if(rpg == "422")
  {
    return ebsdlib::FundamentalSectorGeometry::tetragonalHigh();
  }
  if(rpg == "4")
  {
    return ebsdlib::FundamentalSectorGeometry::tetragonalLow();
  }
  if(rpg == "32")
  {
    return ebsdlib::FundamentalSectorGeometry::trigonalHigh();
  }
  if(rpg == "3")
  {
    return ebsdlib::FundamentalSectorGeometry::trigonalLow();
  }
  if(rpg == "222")
  {
    return ebsdlib::FundamentalSectorGeometry::orthorhombic();
  }
  if(rpg == "2")
  {
    return ebsdlib::FundamentalSectorGeometry::monoclinic();
  }
  return ebsdlib::FundamentalSectorGeometry::triclinic();
}
} // namespace

// -----------------------------------------------------------------------------
// Dump every Laue class's IPF legend with the TSL color key (EbsdLib's
// default) into Testing/Temporary/IPFComparison/<rpg>/. Companion MATLAB
// script at Code_Review/compare_ipf_legends_all_laue.m emits matching
// mtex_ipf_legend_tsl.png and mtex_ipf_legend_hsv.png so the two pairs can
// be compared apples-to-apples per Laue class:
//   ebsdlib_ipf_legend_tsl.png  vs  mtex_ipf_legend_tsl.png  (TSL key)
//   ebsdlib_ipf_legend_nh.png   vs  mtex_ipf_legend_hsv.png  (NH = MTEX HSV)
// (Analogous to the PoleFigureLaueComparisonTest.)
TEST_CASE("ebsdlib::IPFLegendTest::TSL_Compare_MTEX_IPF_Legends", "[EbsdLib][IPFLegendTest]")
{
  const std::string baseDir = std::string(ebsdlib::unit_test::k_TestTempDir) + "IPFComparison";
  std::filesystem::create_directories(baseDir);

  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
  std::set<std::string> seen;

  std::ofstream master(baseDir + "/manifest.txt");
  master << "# IPF legend Laue-class comparison\n";
  master << "# columns: rotationPointGroup, symmetryName\n";

  for(size_t i = 0; i < ops.size(); ++i)
  {
    LaueOps::Pointer op = ops[i];
    const std::string rpg = op->getRotationPointGroup();
    if(seen.count(rpg) > 0)
    {
      continue;
    }
    seen.insert(rpg);

    std::string safe = rpg;
    for(char& c : safe)
    {
      if(c == '/' || c == ' ')
      {
        c = '_';
      }
    }

    std::string dir = baseDir + "/" + safe;
    std::filesystem::create_directories(dir);

    // TSL legend (per-pixel sampling, EbsdLib default).
    {
      auto legend = op->generateIPFTriangleLegend(1024, false, ebsdlib::HexConvention::XParallelAStar, ebsdlib::ColorKeyKind::TSL, /*gridded=*/false);
      REQUIRE(legend != nullptr);
      std::string tifPath = dir + "/tsl_ebsdlib_ipf_legend.png";
      auto result = PngWriter::WriteColorImage(tifPath, 1024, 1024, 3, legend->data());
      REQUIRE(result.first == 0);
    }

    // Gridded TSL legend. MTEX renders all its color keys via 1-degree grid
    // sampling; this is the apples-to-apples render style for comparison
    // against MTEX ipfTSLKey output.
    {
      auto legend = op->generateIPFTriangleLegend(1024, false, ebsdlib::HexConvention::XParallelAStar, ebsdlib::ColorKeyKind::TSL, /*gridded=*/true);
      REQUIRE(legend != nullptr);
      std::string tifPath = dir + "/tsl_gridded_ebsdlib_ipf_legend.png";
      auto result = PngWriter::WriteColorImage(tifPath, 1024, 1024, 3, legend->data());
      REQUIRE(result.first == 0);
    }

    master << rpg << "," << op->getSymmetryName() << "\n";
  }
}

// -----------------------------------------------------------------------------
// Dump every Laue class's IPF legend with MTEX Nolze-Hielscher color key (the EbsdLib analog of MTEX's
// ipfHSVKey) into Testing/Temporary/IPFComparison/<rpg>/. Companion MATLAB
// script at Code_Review/compare_ipf_legends_all_laue.m emits matching
// mtex_ipf_legend_tsl.png and mtex_ipf_legend_hsv.png so the two pairs can
// be compared apples-to-apples per Laue class:
//   ebsdlib_ipf_legend_tsl.png  vs  mtex_ipf_legend_tsl.png  (TSL key)
//   ebsdlib_ipf_legend_nh.png   vs  mtex_ipf_legend_hsv.png  (NH = MTEX HSV)
// (Analogous to the PoleFigureLaueComparisonTest.)
TEST_CASE("ebsdlib::IPFLegendTest::NH_Compare_MTEX_IPF_Legends", "[EbsdLib][IPFLegendTest]")
{
  const std::string baseDir = std::string(ebsdlib::unit_test::k_TestTempDir) + "IPFComparison";
  std::filesystem::create_directories(baseDir);

  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
  std::set<std::string> seen;

  std::ofstream master(baseDir + "/manifest.txt");
  master << "# IPF legend Laue-class comparison\n";
  master << "# columns: rotationPointGroup, symmetryName\n";

  for(size_t i = 0; i < ops.size(); ++i)
  {
    LaueOps::Pointer op = ops[i];
    const std::string rpg = op->getRotationPointGroup();
    if(seen.count(rpg) > 0)
    {
      continue;
    }
    seen.insert(rpg);

    std::string safe = rpg;
    for(char& c : safe)
    {
      if(c == '/' || c == ' ')
      {
        c = '_';
      }
    }

    std::string dir = baseDir + "/" + safe;
    std::filesystem::create_directories(dir);

    // Nolze-Hielscher legend (per-pixel sampling). Compare against MTEX ipfHSVKey.
    // Each LaueOps subclass owns its own per-class NH singleton built from the
    // corresponding FundamentalSectorGeometry; we just pick the kind here.
    {
      auto legend = op->generateIPFTriangleLegend(1024, false, ebsdlib::HexConvention::XParallelAStar, ebsdlib::ColorKeyKind::NolzeHielscher, /*gridded=*/false);
      REQUIRE(legend != nullptr);
      std::string tifPath = dir + "/nh_ebsdlib_ipf_legend.png";
      auto result = PngWriter::WriteColorImage(tifPath, 1024, 1024, 3, legend->data());
      REQUIRE(result.first == 0);
    }

    // Gridded Nolze-Hielscher legend (1-degree flat-shaded cells, MTEX-style).
    {
      auto legend = op->generateIPFTriangleLegend(1024, false, ebsdlib::HexConvention::XParallelAStar, ebsdlib::ColorKeyKind::NolzeHielscher, /*gridded=*/true);
      REQUIRE(legend != nullptr);
      std::string tifPath = dir + "/nh_gridded_ebsdlib_ipf_legend.png";
      auto result = PngWriter::WriteColorImage(tifPath, 1024, 1024, 3, legend->data());
      REQUIRE(result.first == 0);
    }

    master << rpg << "," << op->getSymmetryName() << "\n";
  }
}

// -----------------------------------------------------------------------------
// Dump every Laue class's IPF legend with MTEX Nolze-Hielscher color key (the EbsdLib analog of MTEX's
// ipfHSVKey) into Testing/Temporary/IPFComparison/<rpg>/. Companion MATLAB
// script at Code_Review/compare_ipf_legends_all_laue.m emits matching
// mtex_ipf_legend_tsl.png and mtex_ipf_legend_hsv.png so the two pairs can
// be compared apples-to-apples per Laue class:
//   ebsdlib_ipf_legend_tsl.png  vs  mtex_ipf_legend_tsl.png  (TSL key)
//   ebsdlib_ipf_legend_nh.png   vs  mtex_ipf_legend_hsv.png  (NH = MTEX HSV)
// (Analogous to the PoleFigureLaueComparisonTest.)
TEST_CASE("ebsdlib::IPFLegendTest::PUCM_Compare_MTEX_IPF_Legends", "[EbsdLib][IPFLegendTest]")
{
  const std::string baseDir = std::string(ebsdlib::unit_test::k_TestTempDir) + "IPFComparison";
  std::filesystem::create_directories(baseDir);

  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
  std::set<std::string> seen;

  std::ofstream master(baseDir + "/manifest.txt");
  master << "# IPF legend Laue-class comparison\n";
  master << "# columns: rotationPointGroup, symmetryName\n";

  for(size_t i = 0; i < ops.size(); ++i)
  {
    LaueOps::Pointer op = ops[i];
    const std::string rpg = op->getRotationPointGroup();
    if(seen.count(rpg) > 0)
    {
      continue;
    }
    seen.insert(rpg);

    std::string safe = rpg;
    for(char& c : safe)
    {
      if(c == '/' || c == ' ')
      {
        c = '_';
      }
    }

    std::string dir = baseDir + "/" + safe;
    std::filesystem::create_directories(dir);

    // PUCM legend (per-pixel). Each LaueOps subclass owns its own per-class
    // PUCM singleton (rotation point group baked in).
    {
      auto legend = op->generateIPFTriangleLegend(1024, false, ebsdlib::HexConvention::XParallelAStar, ebsdlib::ColorKeyKind::PUCM, /*gridded=*/false);
      REQUIRE(legend != nullptr);
      std::string tifPath = dir + "/pucm_ebsdlib_ipf_legend.png";
      auto result = PngWriter::WriteColorImage(tifPath, 1024, 1024, 3, legend->data());
      REQUIRE(result.first == 0);
    }

    // Gridded PUCM legend (1-degree flat-shaded cells, MTEX-style).
    {
      auto legend = op->generateIPFTriangleLegend(1024, false, ebsdlib::HexConvention::XParallelAStar, ebsdlib::ColorKeyKind::PUCM, /*gridded=*/true);
      REQUIRE(legend != nullptr);
      std::string tifPath = dir + "/pucm_gridded_ebsdlib_ipf_legend.png";
      auto result = PngWriter::WriteColorImage(tifPath, 1024, 1024, 3, legend->data());
      REQUIRE(result.first == 0);
    }

    master << rpg << "," << op->getSymmetryName() << "\n";
  }
}
