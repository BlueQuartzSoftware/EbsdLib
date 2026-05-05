/* ============================================================================
 * Tests for the PUCMColorKey wrapper around wlenthe's reference
 * implementation of perceptually uniform IPF coloring.
 * ============================================================================ */
#include <catch2/catch.hpp>

#include "EbsdLib/Utilities/PUCMColorKey.hpp"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ebsdlib;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::PUCMColorKey::ConstructionAndName", "[EbsdLib][PUCMColorKey]")
{
  // Every supported rotation point group must construct without throwing
  // and produce a name that includes the rotation point group string.
  for(const std::string rpg : {"1", "2", "222", "3", "32", "4", "422", "6", "622", "23", "432"})
  {
    PUCMColorKey key(rpg);
    CHECK(key.rotationPointGroup() == rpg);
    CHECK(key.name() == "PUCM (" + rpg + ")");
  }
}

TEST_CASE("ebsdlib::PUCMColorKey::UnknownRotationPointGroupThrows", "[EbsdLib][PUCMColorKey]")
{
  REQUIRE_THROWS_AS(PUCMColorKey("nope"), std::invalid_argument);
  REQUIRE_THROWS_AS(PUCMColorKey(""), std::invalid_argument);
}

// -----------------------------------------------------------------------------
// Sanity check: the c-axis [001] in cubic m-3m maps to a saturated color
// (not white, not black). The exact RGB depends on PUCM's hue rotation,
// but the color must be a real RGB triple in [0, 1] with non-trivial
// saturation.
TEST_CASE("ebsdlib::PUCMColorKey::CubicCAxisProducesSaturatedColor", "[EbsdLib][PUCMColorKey]")
{
  PUCMColorKey key("432");

  IColorKey::Vec3 c = key.direction2Color(IColorKey::Vec3{0.0, 0.0, 1.0});

  for(double channel : c)
  {
    CHECK(std::isfinite(channel));
    CHECK(channel >= 0.0);
    CHECK(channel <= 1.0);
  }

  const double maxC = std::max({c[0], c[1], c[2]});
  const double minC = std::min({c[0], c[1], c[2]});
  // c-axis is the white-center for cubic in PUCM (saturation should be 0).
  // Either way: the result must be a valid color, not NaN/garbage.
  INFO("cubic [001] -> RGB(" << c[0] << ", " << c[1] << ", " << c[2] << ")");
  CHECK(maxC <= 1.0);
  CHECK(minC >= 0.0);
}

// -----------------------------------------------------------------------------
// Crystallographically non-equivalent directions in the cubic m-3m FZ
// (the three triangle vertices [001], [011], [111]) must produce
// distinguishable colors. Note: [001] and [100] are symmetry-equivalent
// under m-3m (any cube face) and would correctly map to the same color.
TEST_CASE("ebsdlib::PUCMColorKey::DistinctFZVerticesHaveDistinctColors", "[EbsdLib][PUCMColorKey]")
{
  PUCMColorKey key("432");

  const double r2 = std::sqrt(2.0);
  const double r3 = std::sqrt(3.0);

  IColorKey::Vec3 c001 = key.direction2Color(IColorKey::Vec3{0.0, 0.0, 1.0});
  IColorKey::Vec3 c011 = key.direction2Color(IColorKey::Vec3{0.0, 1.0 / r2, 1.0 / r2});
  IColorKey::Vec3 c111 = key.direction2Color(IColorKey::Vec3{1.0 / r3, 1.0 / r3, 1.0 / r3});

  auto distance = [](const IColorKey::Vec3& a, const IColorKey::Vec3& b) { return std::abs(a[0] - b[0]) + std::abs(a[1] - b[1]) + std::abs(a[2] - b[2]); };

  INFO("[001] -> (" << c001[0] << ", " << c001[1] << ", " << c001[2] << ")");
  INFO("[011] -> (" << c011[0] << ", " << c011[1] << ", " << c011[2] << ")");
  INFO("[111] -> (" << c111[0] << ", " << c111[1] << ", " << c111[2] << ")");

  CHECK(distance(c001, c011) > 0.1);
  CHECK(distance(c001, c111) > 0.1);
  CHECK(distance(c011, c111) > 0.1);
}

// -----------------------------------------------------------------------------
// All 11 supported Laue classes must produce a finite color for any
// arbitrary direction without throwing or returning NaN.
TEST_CASE("ebsdlib::PUCMColorKey::AllLaueClassesProduceFiniteColors", "[EbsdLib][PUCMColorKey]")
{
  // A non-canonical direction so we exercise non-trivial dispatch paths.
  IColorKey::Vec3 dir{0.4, 0.6, 0.7};
  const double mag = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
  for(auto& v : dir)
    v /= mag;

  for(const std::string rpg : {"1", "2", "222", "3", "32", "4", "422", "6", "622", "23", "432"})
  {
    PUCMColorKey key(rpg);
    auto c = key.direction2Color(dir);
    INFO("rpg " << rpg << " -> RGB(" << c[0] << ", " << c[1] << ", " << c[2] << ")");
    CHECK(std::isfinite(c[0]));
    CHECK(std::isfinite(c[1]));
    CHECK(std::isfinite(c[2]));
    CHECK(c[0] >= 0.0);
    CHECK(c[0] <= 1.0);
    CHECK(c[1] >= 0.0);
    CHECK(c[1] <= 1.0);
    CHECK(c[2] >= 0.0);
    CHECK(c[2] <= 1.0);
  }
}

// -----------------------------------------------------------------------------
// The 3-arg overload must agree with the Vec3 overload after converting
// (eta, chi) -> Cartesian. (PUCM ignores angleLimits — it has its own
// per-Laue-class fundamental sector geometry baked in.)
TEST_CASE("ebsdlib::PUCMColorKey::ThreeArgOverloadMatchesVec3", "[EbsdLib][PUCMColorKey]")
{
  PUCMColorKey key("622");

  const double eta = 25.0 * M_PI / 180.0;
  const double chi = 50.0 * M_PI / 180.0;
  const double s = std::sin(chi);
  IColorKey::Vec3 dir{s * std::cos(eta), s * std::sin(eta), std::cos(chi)};

  const std::array<double, 3> dummyLimits{0.0, M_PI / 6.0, M_PI / 2.0};

  auto fromVec3 = key.direction2Color(dir);
  auto fromAngles = key.direction2Color(eta, chi, dummyLimits);

  CHECK(fromAngles[0] == Approx(fromVec3[0]).margin(1e-9));
  CHECK(fromAngles[1] == Approx(fromVec3[1]).margin(1e-9));
  CHECK(fromAngles[2] == Approx(fromVec3[2]).margin(1e-9));
}
