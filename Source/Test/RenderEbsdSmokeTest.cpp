/* ============================================================================
 * Copyright (c) 2009-2025 BlueQuartz Software, LLC
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Smoke test for the render_ebsd CLI driver. Drives the driver as a library
 * function (not as a subprocess) over the {convention} x {color-key} matrix
 * for one hex/trig phase and asserts the expected PNGs are written.
 *
 * Fixture: Data/ipf_color_tests/AllLaueClasses_RandO.ang -- a 12-phase scan
 * with one phase per Laue class. We filter to Phase 4 (Hexagonal_High,
 * "dihexagonal"); that's the most informative class for HexConvention since
 * basal-plane direction tables differ between X||a and X||a*.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

#include "Apps/render_ebsd.h"

#include <array>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <vector>

namespace
{
constexpr int k_HexagonalHighPhase = 4; // AllLaueClasses_RandO.ang Phase 4 = dihexagonal (6/mmm)

void requirePngExists(const std::string& path)
{
  REQUIRE(std::filesystem::exists(path));
  REQUIRE(std::filesystem::file_size(path) > 256ULL); // a real PNG header alone is ~100 bytes; 256 is a sanity lower bound
}

void runOneCell(ebsdlib::HexConvention conv, ebsdlib::ColorKeyKind colorKey)
{
  ebsdlib::render_ebsd::Options opts;
  opts.inputFile = ebsdlib::unit_test::RenderEbsdTest::AllLaueClassesAng;
  opts.outputDir = ebsdlib::unit_test::RenderEbsdTest::OutputDir;
  opts.convention = conv;
  opts.colorKey = colorKey;
  opts.phaseFilter = k_HexagonalHighPhase;
  opts.refDir = {0.0F, 0.0F, 1.0F};
  opts.imageDim = 256; // small for fast smoke run
  opts.lambertDim = 32;
  opts.legendImageDim = 256;

  std::filesystem::create_directories(opts.outputDir);

  ebsdlib::render_ebsd::Result result = ebsdlib::render_ebsd::run(opts);
  REQUIRE(result.ok);
  REQUIRE(result.phases.size() == 1ULL);

  const auto& phase = result.phases[0];
  CHECK(phase.phaseIndex == k_HexagonalHighPhase);
  CHECK(phase.ok);
  requirePngExists(phase.poleFigurePath);
  requirePngExists(phase.ipfMapPath);
  requirePngExists(phase.legendPath);
}
} // namespace

TEST_CASE("ebsdlib::RenderEbsdSmokeTest::ConventionColorKeyMatrix", "[EbsdLib][RenderEbsdSmokeTest]")
{
  using ebsdlib::ColorKeyKind;
  using ebsdlib::HexConvention;

  SECTION("X||a* + TSL")
  {
    runOneCell(HexConvention::XParallelAStar, ColorKeyKind::TSL);
  }
  SECTION("X||a* + PUCM")
  {
    runOneCell(HexConvention::XParallelAStar, ColorKeyKind::PUCM);
  }
  SECTION("X||a + TSL")
  {
    runOneCell(HexConvention::XParallelA, ColorKeyKind::TSL);
  }
  SECTION("X||a + PUCM")
  {
    runOneCell(HexConvention::XParallelA, ColorKeyKind::PUCM);
  }
}

// -----------------------------------------------------------------------------
// Crucial property check: under both conventions the PF / IPF / legend PNGs
// must be DIFFERENT files for hex/trig phases. If they came out byte-identical
// we'd know the HexConvention parameter is being silently ignored upstream.
// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::RenderEbsdSmokeTest::ConventionsDifferOnHexagonalHigh", "[EbsdLib][RenderEbsdSmokeTest]")
{
  using ebsdlib::ColorKeyKind;
  using ebsdlib::HexConvention;

  ebsdlib::render_ebsd::Options optsAStar;
  optsAStar.inputFile = ebsdlib::unit_test::RenderEbsdTest::AllLaueClassesAng;
  optsAStar.outputDir = ebsdlib::unit_test::RenderEbsdTest::OutputDir;
  optsAStar.convention = HexConvention::XParallelAStar;
  optsAStar.colorKey = ColorKeyKind::TSL;
  optsAStar.phaseFilter = k_HexagonalHighPhase;
  optsAStar.imageDim = 256;
  optsAStar.lambertDim = 32;
  optsAStar.legendImageDim = 256;
  std::filesystem::create_directories(optsAStar.outputDir);

  ebsdlib::render_ebsd::Options optsA = optsAStar;
  optsA.convention = HexConvention::XParallelA;

  auto rA = ebsdlib::render_ebsd::run(optsA);
  auto rAStar = ebsdlib::render_ebsd::run(optsAStar);
  REQUIRE(rA.ok);
  REQUIRE(rAStar.ok);
  REQUIRE(rA.phases.size() == 1ULL);
  REQUIRE(rAStar.phases.size() == 1ULL);

  // The composite PF MUST differ byte-for-byte between conventions: the basal-
  // plane direction families ({10-10}, {2-1-10}) project into different
  // positions on the unit disk under X||a vs X||a*. If they came out
  // identical, the HexConvention parameter is being silently dropped.
  //
  // The IPF MAP for 6/mmm specifically is convention-invariant (the SST is
  // reached via c-axis rotations + inversion fold; the basal-plane sym ops
  // that differ between bases are operationally redundant — see the comment
  // in LaueOpsTest::GenerateIPFColor_HexConvention_HexagonalOps).
  //
  // The IPF LEGEND MUST differ byte-for-byte between conventions: the SST
  // colored region is convention-invariant for 6/mmm, but the Miller-index
  // labels drawn around the unit circle change (PR 2h plumbed conv through
  // annotateIPFImage / drawIPFAnnotations). Under X||a the +X corner reads
  // [2-1-10]; under X||a* it reads [10-10].
  //
  // So for HexagonalHigh we check: PF differs (positive proof of compositor
  // plumbing), IPF map identical (6/mmm SST color invariance), and legend
  // differs (positive proof of label plumbing).
  const auto readBytes = [](const std::string& path) {
    std::ifstream ifs(path, std::ios::binary);
    return std::vector<char>{std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>()};
  };
  const auto pfBytesA = readBytes(rA.phases[0].poleFigurePath);
  const auto pfBytesAStar = readBytes(rAStar.phases[0].poleFigurePath);
  REQUIRE_FALSE(pfBytesA.empty());
  REQUIRE_FALSE(pfBytesAStar.empty());
  CHECK(pfBytesA != pfBytesAStar);

  const auto ipfBytesA = readBytes(rA.phases[0].ipfMapPath);
  const auto ipfBytesAStar = readBytes(rAStar.phases[0].ipfMapPath);
  CHECK(ipfBytesA == ipfBytesAStar);

  const auto legBytesA = readBytes(rA.phases[0].legendPath);
  const auto legBytesAStar = readBytes(rAStar.phases[0].legendPath);
  CHECK(legBytesA != legBytesAStar);
}
