#include <catch2/catch.hpp>

#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/FundamentalSectorGeometry.hpp"
#include "EbsdLib/Utilities/NolzeHielscherColorKey.hpp"
#include "EbsdLib/Utilities/TSLColorKey.hpp"

#include <array>
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::TSLColorKey::Name", "[EbsdLib][TSLColorKey]")
{
  ebsdlib::TSLColorKey tslKey;
  REQUIRE(tslKey.name() == "TSL");
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::TSLColorKey::KnownCubicDirections", "[EbsdLib][TSLColorKey]")
{
  ebsdlib::TSLColorKey tslKey;

  SECTION("[001] direction at chi=0 -> red (r=1, g=0, b=0)")
  {
    // At chi = 0: r = sqrt(1 - 0/chiMax) = 1, g = 0, b = 0 => red
    double eta = 0.0;
    double chi = 0.0;
    double chiMax = std::acos(std::sqrt(1.0 / 3.0));
    std::array<double, 3> limits = {0.0, M_PI / 4.0, chiMax};
    auto [r, g, b] = tslKey.direction2Color(eta, chi, limits);
    REQUIRE(r == Approx(1.0).margin(0.01));
    REQUIRE(g == Approx(0.0).margin(0.01));
    REQUIRE(b == Approx(0.0).margin(0.01));
  }

  SECTION("eta=pi/4, chi=0 -> pure red (chi=0 zeros out g and b)")
  {
    // At chi=0: b *= chi/chiMax = 0, g *= chi/chiMax = 0
    // r = sqrt(1 - 0) = 1.0 => result is always red at chi=0
    double eta = M_PI / 4.0;
    double chi = 0.0;
    double chiMax = std::acos(std::sqrt(1.0 / 3.0));
    std::array<double, 3> limits = {0.0, M_PI / 4.0, chiMax};
    auto [r, g, b] = tslKey.direction2Color(eta, chi, limits);
    REQUIRE(r == Approx(1.0).margin(0.01));
    REQUIRE(g == Approx(0.0).margin(0.01));
    REQUIRE(b == Approx(0.0).margin(0.01));
  }

  SECTION("[111] at eta=pi/4, chi=chiMax -> pure blue")
  {
    // At eta=etaMax, chi=chiMax:
    // r = 1 - 1 = 0, b_raw = 1*1 = 1 => sqrt(1) = 1, g = (1-1)*1 = 0
    // After normalization: (0, 0, 1) = blue
    double chiMax = std::acos(std::sqrt(1.0 / 3.0));
    std::array<double, 3> limits = {0.0, M_PI / 4.0, chiMax};
    auto [r, g, b] = tslKey.direction2Color(M_PI / 4.0, chiMax, limits);
    REQUIRE(r == Approx(0.0).margin(0.01));
    REQUIRE(g == Approx(0.0).margin(0.01));
    REQUIRE(b == Approx(1.0).margin(0.01));
  }

  SECTION("Grid of directions all produce valid [0,1] outputs")
  {
    double etaMax = M_PI / 4.0;
    for(double eta = 0.0; eta <= etaMax; eta += 0.05)
    {
      double tanEta = std::tan(std::max(eta, 1e-6));
      double chiMax = std::acos(std::sqrt(1.0 / (2.0 + tanEta * tanEta)));
      for(double chi = 0.0; chi <= chiMax; chi += 0.05)
      {
        std::array<double, 3> limits = {0.0, etaMax, chiMax};
        auto [r, g, b] = tslKey.direction2Color(eta, chi, limits);
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

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::TSLColorKey::ExactRegressionValues", "[EbsdLib][TSLColorKey]")
{
  ebsdlib::TSLColorKey tslKey;

  SECTION("eta=0, chi=0.5, chiMax=1.0 -> near red/green mix, no blue")
  {
    // r = sqrt(1 - 0.5) = sqrt(0.5) ~ 0.707
    // b = |0 - 0| / (pi/4 - 0) * 0.5 = 0, so sqrt(0) = 0
    // g = (1-0)*0.5 = 0.5, sqrt(0.5) ~ 0.707
    // maxVal = 0.707, r/max = 1, g/max = 1, b = 0
    double chiMax = 1.0;
    double chi = 0.5;
    double eta = 0.0;
    std::array<double, 3> limits = {0.0, M_PI / 4.0, chiMax};
    auto [r, g, b] = tslKey.direction2Color(eta, chi, limits);
    REQUIRE(r == Approx(1.0).margin(0.01));
    REQUIRE(g == Approx(1.0).margin(0.01));
    REQUIRE(b == Approx(0.0).margin(0.01));
  }

  SECTION("eta=pi/4, chi=chiMax -> blue corner (b=1)")
  {
    // chi = chiMax, eta = pi/4 = etaMax
    // r = 1 - chiMax/chiMax = 0 => sqrt(0) = 0
    // b = |pi/4 - 0| / (pi/4 - 0) * chiMax/chiMax = 1 => sqrt(1) = 1
    // g = (1-1)*1 = 0 => sqrt(0) = 0
    // maxVal = 1, result = (0, 0, 1) => blue
    double chiMax = 0.9553; // acos(1/sqrt(3))
    double chi = chiMax;
    double eta = M_PI / 4.0;
    std::array<double, 3> limits = {0.0, M_PI / 4.0, chiMax};
    auto [r, g, b] = tslKey.direction2Color(eta, chi, limits);
    REQUIRE(r == Approx(0.0).margin(0.01));
    REQUIRE(g == Approx(0.0).margin(0.01));
    REQUIRE(b == Approx(1.0).margin(0.01));
  }
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::TSLColorKey::DefaultAngleLimitsOverride", "[EbsdLib][TSLColorKey]")
{
  ebsdlib::TSLColorKey tslKey;

  SECTION("setDefaultAngleLimits affects direction2Color(Vec3)")
  {
    // With chi=0 (direction = [0,0,1]), result should always be red
    std::array<double, 3> limits = {0.0, M_PI / 4.0, 0.9553};
    tslKey.setDefaultAngleLimits(limits);

    // [0, 0, 1] has chi = acos(1) = 0 => pure red
    ebsdlib::IColorKey::Vec3 dir = {0.0, 0.0, 1.0};
    auto [r, g, b] = tslKey.direction2Color(dir);
    REQUIRE(r == Approx(1.0).margin(0.01));
    REQUIRE(g == Approx(0.0).margin(0.01));
    REQUIRE(b == Approx(0.0).margin(0.01));
  }
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::TSLColorKey::InheritedSphericalDefault", "[EbsdLib][TSLColorKey]")
{
  // The IColorKey base provides a default direction2Color(eta, chi, limits) that
  // converts to Cartesian and calls direction2Color(Vec3). TSLColorKey overrides
  // this, so the spherical overload should take precedence.
  ebsdlib::TSLColorKey tslKey;

  // Verify that both overloads agree for a known direction
  double eta = 0.2;
  double chi = 0.3;
  double chiMax = 0.9553;
  std::array<double, 3> limits = {0.0, M_PI / 4.0, chiMax};

  auto colorFromSpherical = tslKey.direction2Color(eta, chi, limits);

  // Also call via the Vec3 overload with equivalent Cartesian coords
  tslKey.setDefaultAngleLimits(limits);
  double sinChi = std::sin(chi);
  ebsdlib::IColorKey::Vec3 dir = {sinChi * std::cos(eta), sinChi * std::sin(eta), std::cos(chi)};
  auto colorFromCartesian = tslKey.direction2Color(dir);

  // Both paths should agree closely (within floating-point round-trip error)
  REQUIRE(colorFromSpherical[0] == Approx(colorFromCartesian[0]).margin(0.01));
  REQUIRE(colorFromSpherical[1] == Approx(colorFromCartesian[1]).margin(0.01));
  REQUIRE(colorFromSpherical[2] == Approx(colorFromCartesian[2]).margin(0.01));
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::TSLColorKey::PolymorphicUsage", "[EbsdLib][TSLColorKey]")
{
  // Verify TSLColorKey can be used through the IColorKey interface
  std::shared_ptr<ebsdlib::IColorKey> key = std::make_shared<ebsdlib::TSLColorKey>();
  REQUIRE(key->name() == "TSL");

  ebsdlib::IColorKey::Vec3 dir = {0.0, 0.0, 1.0};
  auto color = key->direction2Color(dir);
  REQUIRE(color[0] >= 0.0);
  REQUIRE(color[0] <= 1.0);
  REQUIRE(color[1] >= 0.0);
  REQUIRE(color[1] <= 1.0);
  REQUIRE(color[2] >= 0.0);
  REQUIRE(color[2] <= 1.0);
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::LaueOps::ColorKeyIntegration", "[EbsdLib][ColorKeyIntegration]")
{
  using namespace ebsdlib;

  auto allOps = LaueOps::GetAllOrientationOps();

  SECTION("Per-Laue-class TSL output is non-black for a tilted ref direction")
  {
    double refDir[3] = {0.0, 0.0, 1.0};
    double eulers[3] = {0.5, 0.3, 0.2};

    for(size_t i = 0; i < 11; i++)
    {
      auto color = allOps[i]->generateIPFColor(eulers, refDir, false, ColorKeyKind::TSL);
      int r = RgbColor::dRed(color);
      int g = RgbColor::dGreen(color);
      int b = RgbColor::dBlue(color);
      REQUIRE(r + g + b > 0);
    }
  }
}
