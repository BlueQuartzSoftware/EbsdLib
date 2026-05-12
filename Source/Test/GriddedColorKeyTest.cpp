#include <catch2/catch.hpp>

#include "EbsdLib/LaueOps/CubicOps.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/FundamentalSectorGeometry.hpp"
#include "EbsdLib/Utilities/GriddedColorKey.hpp"
#include "EbsdLib/Utilities/NolzeHielscherColorKey.hpp"
#include "EbsdLib/Utilities/TSLColorKey.hpp"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

TEST_CASE("ebsdlib::GriddedColorKey::BasicProperties", "[EbsdLib][GriddedColorKey]")
{
  auto tslKey = std::make_shared<ebsdlib::TSLColorKey>();
  auto gridKey = std::make_shared<ebsdlib::GriddedColorKey>(tslKey, 2.0);

  SECTION("Name includes gridded suffix")
  {
    REQUIRE(gridKey->name() == "TSL (gridded)");
  }

  SECTION("Inner key is accessible")
  {
    REQUIRE(gridKey->innerKey()->name() == "TSL");
  }

  SECTION("Resolution is stored correctly")
  {
    REQUIRE(gridKey->resolutionDeg() == Approx(2.0));
  }
}

TEST_CASE("ebsdlib::GriddedColorKey::FlatShading", "[EbsdLib][GriddedColorKey]")
{
  auto nhKey = std::make_shared<ebsdlib::NolzeHielscherColorKey>(ebsdlib::FundamentalSectorGeometry::cubicHigh());
  auto gridKey = std::make_shared<ebsdlib::GriddedColorKey>(nhKey, 2.0); // coarse 2-degree grid

  SECTION("Nearby points within same grid cell produce identical colors")
  {
    // Two points that are less than 2 degrees apart should snap to the same grid cell
    double eta1 = 0.2;
    double chi1 = 0.3;
    double eta2 = 0.2 + 0.01; // ~0.6 degrees apart
    double chi2 = 0.3 + 0.01;

    std::array<double, 3> limits = {0.0, M_PI / 4.0, 0.6};
    auto c1 = gridKey->direction2Color(eta1, chi1, limits);
    auto c2 = gridKey->direction2Color(eta2, chi2, limits);

    // Should be exactly equal (same grid cell)
    REQUIRE(c1[0] == Approx(c2[0]).margin(1e-10));
    REQUIRE(c1[1] == Approx(c2[1]).margin(1e-10));
    REQUIRE(c1[2] == Approx(c2[2]).margin(1e-10));
  }

  SECTION("Points in different grid cells may produce different colors")
  {
    double eta1 = 0.2;
    double eta2 = 0.2 + 0.05; // ~2.9 degrees apart, different cell

    std::array<double, 3> limits = {0.0, M_PI / 4.0, 0.6};
    auto c1 = gridKey->direction2Color(eta1, 0.3, limits);
    auto c2 = gridKey->direction2Color(eta2, 0.3, limits);

    // These may or may not differ depending on the color function
    // Just verify they are valid
    REQUIRE(c1[0] >= 0.0);
    REQUIRE(c1[0] <= 1.0);
    REQUIRE(c2[0] >= 0.0);
    REQUIRE(c2[0] <= 1.0);
  }

  SECTION("All outputs are valid RGB")
  {
    for(double eta = 0.01; eta < M_PI / 4.0 - 0.01; eta += 0.05)
    {
      double chiMax = std::acos(std::sqrt(1.0 / (2.0 + std::tan(eta) * std::tan(eta))));
      for(double chi = 0.01; chi < chiMax - 0.01; chi += 0.05)
      {
        std::array<double, 3> limits = {0.0, M_PI / 4.0, chiMax};
        auto [r, g, b] = gridKey->direction2Color(eta, chi, limits);
        REQUIRE(r >= 0.0);
        REQUIRE(r <= 1.0);
        REQUIRE(g >= 0.0);
        REQUIRE(g <= 1.0);
        REQUIRE(b >= 0.0);
        REQUIRE(b <= 1.0);
      }
    }
  }
}

TEST_CASE("ebsdlib::GriddedColorKey::LaueOpsLegendIntegration", "[EbsdLib][GriddedColorKey]")
{
  auto allOps = ebsdlib::LaueOps::GetAllOrientationOps();
  auto& cubicOps = *allOps[1]; // Cubic_High

  SECTION("generateIPFTriangleLegend(gridded=true) returns a valid image")
  {
    auto legend = cubicOps.generateIPFTriangleLegend(64, false, ebsdlib::HexConvention::NotApplicable, ebsdlib::ColorKeyKind::NolzeHielscher, /*gridded=*/true);
    REQUIRE(legend != nullptr);
    REQUIRE(legend->getNumberOfTuples() > 0);
  }

  SECTION("Gridded vs non-gridded legends both render for each kind")
  {
    for(const auto kind : {ebsdlib::ColorKeyKind::TSL, ebsdlib::ColorKeyKind::PUCM, ebsdlib::ColorKeyKind::NolzeHielscher})
    {
      auto perPixel = cubicOps.generateIPFTriangleLegend(64, false, ebsdlib::HexConvention::NotApplicable, kind, /*gridded=*/false);
      auto gridded = cubicOps.generateIPFTriangleLegend(64, false, ebsdlib::HexConvention::NotApplicable, kind, /*gridded=*/true);
      REQUIRE(perPixel != nullptr);
      REQUIRE(gridded != nullptr);
    }
  }
}

// -----------------------------------------------------------------------------
// Regression test for the angleLimits-discard bug. The 3-argument overload
// of GriddedColorKey::direction2Color must honor the caller's angleLimits;
// it cannot just look up colors from a precomputed grid that was baked using
// the inner key's default (cubic) angle limits.
//
// Test: at (eta=15°, chi=45°) with hexagonal-high angle limits
// (etaMin=0, etaMax=30°, chiMax=90°), the gridded TSL key's color must equal
// the per-pixel TSL key's color at the same SNAPPED (eta, chi). Previously
// the gridded key was returning colors computed under cubic m-3m limits
// (etaMax=45°, chiMax=35.26°) for every Laue class, producing wrong-colored
// IPF legends across the board.
TEST_CASE("ebsdlib::GriddedColorKey::HonorsAngleLimitsIn3ArgOverload", "[EbsdLib][GriddedColorKey]")
{
  auto tslKey = std::make_shared<ebsdlib::TSLColorKey>();
  auto gridKey = std::make_shared<ebsdlib::GriddedColorKey>(tslKey, 1.0);

  // Hexagonal-high IPF SST limits, in radians.
  const std::array<double, 3> hexLimits = {0.0, M_PI / 6.0, M_PI / 2.0};
  // Cubic-m3m IPF SST limits — what the gridded key currently bakes into its
  // grid via TSLColorKey's default angle limits.
  const std::array<double, 3> cubicLimits = {0.0, M_PI / 4.0, std::acos(1.0 / std::sqrt(3.0))};

  // (eta, chi) chosen so the cubic and hexagonal formulas give clearly
  // different colors: chi=45° is much more than the cubic chiMax (~35.26°)
  // so the cubic formula clamps red to 0, while the hex formula gives red>0.5.
  const double eta = 15.0 * M_PI / 180.0;
  const double chi = 45.0 * M_PI / 180.0;

  auto gridded = gridKey->direction2Color(eta, chi, hexLimits);
  auto perPixelHex = tslKey->direction2Color(eta, chi, hexLimits);
  auto perPixelCubic = tslKey->direction2Color(eta, chi, cubicLimits);

  INFO("gridded RGB (under hex limits)         = (" << gridded[0] << ", " << gridded[1] << ", " << gridded[2] << ")");
  INFO("per-pixel TSL RGB under hex limits     = (" << perPixelHex[0] << ", " << perPixelHex[1] << ", " << perPixelHex[2] << ")");
  INFO("per-pixel TSL RGB under cubic limits   = (" << perPixelCubic[0] << ", " << perPixelCubic[1] << ", " << perPixelCubic[2] << ")");

  // The two limit sets must produce visibly different colors (otherwise the
  // test wouldn't actually catch the bug). Verify that as a precondition.
  REQUIRE(std::abs(perPixelHex[0] - perPixelCubic[0]) > 0.05);

  // The gridded color under hex limits should equal the per-pixel TSL color
  // under hex limits (modulo grid snapping; with 1° grid and exact-degree
  // input the snap is essentially identity).
  CHECK(gridded[0] == Approx(perPixelHex[0]).margin(0.01));
  CHECK(gridded[1] == Approx(perPixelHex[1]).margin(0.01));
  CHECK(gridded[2] == Approx(perPixelHex[2]).margin(0.01));
}

// -----------------------------------------------------------------------------
// Regression test: GriddedColorKey must pass eta to the inner key
// unmodified, even when eta is negative. Trigonal-low (-3) and trigonal-high
// (-3m) have negative etaMin (-120° and -90° respectively), and the inner
// TSL formula uses |eta - etaMin| which already handles negative eta
// correctly. A pre-snap "wrap to [0, 2π]" step in the grid lookup will
// destroy that math by remapping eta=-60° to +300°.
TEST_CASE("ebsdlib::GriddedColorKey::HandlesNegativeEta", "[EbsdLib][GriddedColorKey]")
{
  auto tslKey = std::make_shared<ebsdlib::TSLColorKey>();
  auto gridKey = std::make_shared<ebsdlib::GriddedColorKey>(tslKey, 1.0);

  // Trigonal-3m angle limits in radians.
  const std::array<double, 3> trigLimits = {-M_PI / 2.0, -M_PI / 6.0, M_PI / 2.0};

  // Pick eta = -60° which lies between etaMin=-90° and etaMax=-30°.
  const double eta = -60.0 * M_PI / 180.0;
  const double chi = 45.0 * M_PI / 180.0;

  auto gridded = gridKey->direction2Color(eta, chi, trigLimits);
  auto perPixel = tslKey->direction2Color(eta, chi, trigLimits);

  INFO("gridded   = (" << gridded[0] << ", " << gridded[1] << ", " << gridded[2] << ")");
  INFO("per-pixel = (" << perPixel[0] << ", " << perPixel[1] << ", " << perPixel[2] << ")");

  CHECK(gridded[0] == Approx(perPixel[0]).margin(0.01));
  CHECK(gridded[1] == Approx(perPixel[1]).margin(0.01));
  CHECK(gridded[2] == Approx(perPixel[2]).margin(0.01));
}

// -----------------------------------------------------------------------------
// Regression test for boundary pixels of the cubic-m3m IPF triangle. The
// curved [011]->[111] edge has chiMax that varies with eta. The legend
// renderer passes angleLimits computed at the *original* (pre-snap) eta, but
// GriddedColorKey snaps eta and chi to grid cells before computing the color.
// For a pixel just inside the boundary, the snap can push chi to be equal-to
// or just past angleLimits[2], producing NaN in the TSL formula
// (1 - chi/chiMax → negative → sqrt). The result is a stippled gray/dark line
// along the curved edge of the cubic IPF legend.
//
// Expected behavior: gridded value should be a valid (non-NaN, R/G/B in [0,1])
// color whose red channel is clamped to 0 rather than going NaN.
TEST_CASE("ebsdlib::GriddedColorKey::BoundarySnapDoesNotProduceNaN", "[EbsdLib][GriddedColorKey]")
{
  auto tslKey = std::make_shared<ebsdlib::TSLColorKey>();
  auto gridKey = std::make_shared<ebsdlib::GriddedColorKey>(tslKey, 1.0);

  // Cubic m-3m at eta=22.5° has chiMax ≈ 47.27°. Pick a pixel JUST inside.
  const double eta = 22.5 * M_PI / 180.0;
  const double chiMax = std::acos(std::sqrt(1.0 / (2.0 + std::tan(eta) * std::tan(eta))));
  const double chi = chiMax - 0.05 * M_PI / 180.0; // 0.05° inside the boundary
  const std::array<double, 3> angleLimits = {0.0, M_PI / 4.0, chiMax};

  auto gridded = gridKey->direction2Color(eta, chi, angleLimits);

  INFO("gridded boundary pixel = (" << gridded[0] << ", " << gridded[1] << ", " << gridded[2] << ")");
  CHECK(std::isfinite(gridded[0]));
  CHECK(std::isfinite(gridded[1]));
  CHECK(std::isfinite(gridded[2]));
  CHECK(gridded[0] >= 0.0);
  CHECK(gridded[0] <= 1.0);
  CHECK(gridded[1] >= 0.0);
  CHECK(gridded[1] <= 1.0);
  CHECK(gridded[2] >= 0.0);
  CHECK(gridded[2] <= 1.0);
}

// -----------------------------------------------------------------------------
// Regression test for triclinic IPF legend coloring. Triclinic (-1) has
// etaMin=0, etaMax=π — i.e. the upper bound is 180° and the legend renders
// the full stereographic disk. Pixels in the lower hemisphere of the disk
// have eta from atan2 in [-π, 0]; the TSL formula uses |eta - etaMin| =
// |eta| so negative eta colors the disk symmetrically about y=0. Clamping
// snappedEta to [angleLimits[0], angleLimits[1]] would collapse all of
// those pixels to a single eta value, ruining the lower-hemisphere colors.
TEST_CASE("ebsdlib::GriddedColorKey::TriclinicNegativeEtaProducesColor", "[EbsdLib][GriddedColorKey]")
{
  auto tslKey = std::make_shared<ebsdlib::TSLColorKey>();
  auto gridKey = std::make_shared<ebsdlib::GriddedColorKey>(tslKey, 1.0);

  // Triclinic IPF angle limits.
  const std::array<double, 3> tricLimits = {0.0, M_PI, M_PI / 2.0};

  // A lower-hemisphere pixel at eta = -90°. Per-pixel TSL must render this
  // with the same color as eta = +90° (because the formula uses |eta|).
  const double eta = -90.0 * M_PI / 180.0;
  const double chi = 30.0 * M_PI / 180.0;

  auto gridded = gridKey->direction2Color(eta, chi, tricLimits);
  auto perPixelNeg = tslKey->direction2Color(eta, chi, tricLimits);
  auto perPixelPos = tslKey->direction2Color(-eta, chi, tricLimits);

  // The per-pixel TSL formula is symmetric in |eta|.
  REQUIRE(perPixelNeg[0] == Approx(perPixelPos[0]).margin(1e-9));
  REQUIRE(perPixelNeg[1] == Approx(perPixelPos[1]).margin(1e-9));
  REQUIRE(perPixelNeg[2] == Approx(perPixelPos[2]).margin(1e-9));

  // The gridded TSL should match the per-pixel result (modulo grid snap).
  CHECK(gridded[0] == Approx(perPixelNeg[0]).margin(0.01));
  CHECK(gridded[1] == Approx(perPixelNeg[1]).margin(0.01));
  CHECK(gridded[2] == Approx(perPixelNeg[2]).margin(0.01));
}
