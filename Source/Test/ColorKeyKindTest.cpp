#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Utilities/ColorTable.h"

#include <array>
#include <cstdint>

// ---------------------------------------------------------------------------
// These tests pin down the ColorKeyKind dispatch API on LaueOps. The contract:
//
//   1. ColorKeyKind { TSL, PUCM, NolzeHielscher } enum exists in `ebsdlib::`.
//   2. generateIPFColor's default kind is TSL — the no-kind overload and an
//      explicit ColorKeyKind::TSL call agree exactly.
//   3. PUCM and NolzeHielscher kinds change the per-pixel output (proves the
//      kind argument actually routes through different color keys, not just
//      compiled-out-and-ignored).
//   4. All 11 Laue classes accept all 3 kinds without throwing — each subclass
//      owns a per-class PUCM/NH singleton matched to its point group / sector.
// ---------------------------------------------------------------------------

namespace
{
struct ColorTriple
{
  int r;
  int g;
  int b;
};

ColorTriple unpack(ebsdlib::Rgb argb)
{
  return {ebsdlib::RgbColor::dRed(argb), ebsdlib::RgbColor::dGreen(argb), ebsdlib::RgbColor::dBlue(argb)};
}

bool sameColor(ebsdlib::Rgb a, ebsdlib::Rgb b)
{
  const auto ca = unpack(a);
  const auto cb = unpack(b);
  return ca.r == cb.r && ca.g == cb.g && ca.b == cb.b;
}
} // namespace

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::ColorKeyKind::EnumExists", "[EbsdLib][ColorKeyKind]")
{
  // Just an "it compiles" test: the enum must be present in ebsdlib:: with
  // these three named values.
  REQUIRE(static_cast<uint8_t>(ebsdlib::ColorKeyKind::TSL) == 0);
  REQUIRE(static_cast<uint8_t>(ebsdlib::ColorKeyKind::PUCM) == 1);
  REQUIRE(static_cast<uint8_t>(ebsdlib::ColorKeyKind::NolzeHielscher) == 2);
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::ColorKeyKind::DefaultIsTSL", "[EbsdLib][ColorKeyKind]")
{
  // No-kind generateIPFColor must match an explicit ColorKeyKind::TSL call,
  // for every Laue class. Captures the "default to TSL" requirement.
  auto allOps = ebsdlib::LaueOps::GetAllOrientationOps();
  double eulers[3] = {0.5, 0.3, 0.2};
  double refDir[3] = {0.6, 0.3, 0.7};

  for(size_t i = 0; i < allOps.size(); ++i)
  {
    const auto defaultColor = allOps[i]->generateIPFColor(eulers, refDir, false);
    const auto explicitTsl = allOps[i]->generateIPFColor(eulers, refDir, false, ebsdlib::ColorKeyKind::TSL);
    INFO("Laue index " << i);
    REQUIRE(sameColor(defaultColor, explicitTsl));
  }
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::ColorKeyKind::PUCMDiffersFromTSL", "[EbsdLib][ColorKeyKind]")
{
  // PUCM and TSL color the SST differently for a generic off-corner direction.
  // Identity orientation + tilted refDir is the simplest input that lands well
  // inside the SST (not on a primary [001]/[011]/[111] axis where keys agree).
  auto allOps = ebsdlib::LaueOps::GetAllOrientationOps();
  double eulers[3] = {0.0, 0.0, 0.0};
  double refDir[3] = {0.6, 0.3, 0.7};

  for(size_t i = 0; i < allOps.size(); ++i)
  {
    const auto tslColor = allOps[i]->generateIPFColor(eulers, refDir, false, ebsdlib::ColorKeyKind::TSL);
    const auto pucmColor = allOps[i]->generateIPFColor(eulers, refDir, false, ebsdlib::ColorKeyKind::PUCM);
    INFO("Laue index " << i);
    REQUIRE_FALSE(sameColor(tslColor, pucmColor));
  }
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::ColorKeyKind::NolzeHielscherDiffersFromTSL", "[EbsdLib][ColorKeyKind]")
{
  // NH differs from TSL for the same reason; each Laue class has its own
  // FundamentalSectorGeometry baked into its NH singleton.
  auto allOps = ebsdlib::LaueOps::GetAllOrientationOps();
  double eulers[3] = {0.0, 0.0, 0.0};
  double refDir[3] = {0.6, 0.3, 0.7};

  for(size_t i = 0; i < allOps.size(); ++i)
  {
    const auto tslColor = allOps[i]->generateIPFColor(eulers, refDir, false, ebsdlib::ColorKeyKind::TSL);
    const auto nhColor = allOps[i]->generateIPFColor(eulers, refDir, false, ebsdlib::ColorKeyKind::NolzeHielscher);
    INFO("Laue index " << i);
    REQUIRE_FALSE(sameColor(tslColor, nhColor));
  }
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::ColorKeyKind::LegendAcceptsKindAndGridded", "[EbsdLib][ColorKeyKind]")
{
  // generateIPFTriangleLegend must accept (conv, kind, gridded). All 11 Laue
  // classes should produce a non-null image for each kind, both gridded and
  // non-gridded, in the canonical XParallelAStar convention.
  auto allOps = ebsdlib::LaueOps::GetAllOrientationOps();
  constexpr int k_ImageDim = 128;

  for(const auto& kind : {ebsdlib::ColorKeyKind::TSL, ebsdlib::ColorKeyKind::PUCM, ebsdlib::ColorKeyKind::NolzeHielscher})
  {
    for(const bool gridded : {false, true})
    {
      for(size_t i = 0; i < allOps.size(); ++i)
      {
        auto img = allOps[i]->generateIPFTriangleLegend(k_ImageDim, /*generateEntirePlane=*/false, ebsdlib::HexConvention::XParallelAStar, kind, gridded);
        INFO("Laue index " << i << " kind=" << static_cast<int>(kind) << " gridded=" << gridded);
        REQUIRE(img != nullptr);
        REQUIRE(img->size() > 0);
      }
    }
  }
}
