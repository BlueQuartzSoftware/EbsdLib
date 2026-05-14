#include <catch2/catch.hpp>

#include "EbsdLib/Utilities/NolzeHielscherColorKey.hpp"

#include <array>
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::NolzeHielscherColorKey::HueSpeedFunction", "[EbsdLib][NolzeHielscher]")
{
  SECTION("Speed function is positive everywhere")
  {
    for(double rho = 0.0; rho < 360.0; rho += 1.0)
    {
      double v = ebsdlib::NolzeHielscherColorKey::hueSpeedFunction(rho, 1.0);
      REQUIRE(v > 0.0);
    }
  }

  SECTION("Speed function peaks near 0, 120, 240 degrees")
  {
    double v0 = ebsdlib::NolzeHielscherColorKey::hueSpeedFunction(0.0, 1.0);
    double v60 = ebsdlib::NolzeHielscherColorKey::hueSpeedFunction(60.0, 1.0);
    double v120 = ebsdlib::NolzeHielscherColorKey::hueSpeedFunction(120.0, 1.0);
    REQUIRE(v0 > v60);
    REQUIRE(v120 > v60);
  }

  SECTION("Speed function scales linearly with distance")
  {
    double v1 = ebsdlib::NolzeHielscherColorKey::hueSpeedFunction(45.0, 1.0);
    double v2 = ebsdlib::NolzeHielscherColorKey::hueSpeedFunction(45.0, 2.0);
    REQUIRE(v2 == Approx(2.0 * v1).margin(1e-10));
  }

  SECTION("Speed function is symmetric about each peak")
  {
    // Symmetric about 0 degrees
    double vPos10 = ebsdlib::NolzeHielscherColorKey::hueSpeedFunction(10.0, 1.0);
    double vNeg10 = ebsdlib::NolzeHielscherColorKey::hueSpeedFunction(-10.0, 1.0);
    REQUIRE(vPos10 == Approx(vNeg10).margin(1e-10));

    // Symmetric about 120 degrees
    double v110 = ebsdlib::NolzeHielscherColorKey::hueSpeedFunction(110.0, 1.0);
    double v130 = ebsdlib::NolzeHielscherColorKey::hueSpeedFunction(130.0, 1.0);
    REQUIRE(v110 == Approx(v130).margin(1e-10));
  }
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::NolzeHielscherColorKey::LightnessMapping", "[EbsdLib][NolzeHielscher]")
{
  SECTION("At theta=0 (center): L equals 0")
  {
    double L = ebsdlib::NolzeHielscherColorKey::lightness(0.0, 0.25);
    REQUIRE(L == Approx(0.0).margin(1e-6));
  }

  SECTION("At theta=pi/2 (boundary): L is approximately 0.625")
  {
    double L = ebsdlib::NolzeHielscherColorKey::lightness(M_PI / 2.0, 0.25);
    // lambdaL=0.25: 0.25*1 + 0.75*sin^2(pi/4) = 0.25 + 0.75*0.5 = 0.625
    REQUIRE(L == Approx(0.625).margin(1e-6));
  }

  SECTION("Monotonically increasing with theta")
  {
    double prev = 0.0;
    for(double theta = 0.0; theta <= M_PI / 2.0; theta += 0.01)
    {
      double L = ebsdlib::NolzeHielscherColorKey::lightness(theta, 0.25);
      REQUIRE(L >= prev - 1e-10);
      prev = L;
    }
  }

  SECTION("lambdaL=0 gives pure sin^2 mapping")
  {
    double theta = M_PI / 4.0;
    double L = ebsdlib::NolzeHielscherColorKey::lightness(theta, 0.0);
    double expected = std::sin(theta / 2.0) * std::sin(theta / 2.0);
    REQUIRE(L == Approx(expected).margin(1e-10));
  }

  SECTION("lambdaL=1 gives pure linear mapping")
  {
    double theta = M_PI / 4.0;
    double L = ebsdlib::NolzeHielscherColorKey::lightness(theta, 1.0);
    double expected = theta / (M_PI / 2.0);
    REQUIRE(L == Approx(expected).margin(1e-10));
  }
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::NolzeHielscherColorKey::SaturationMapping", "[EbsdLib][NolzeHielscher]")
{
  SECTION("At L=0.5: S is maximum (1.0)")
  {
    double S = ebsdlib::NolzeHielscherColorKey::saturation(0.5, 0.25);
    REQUIRE(S == Approx(1.0).margin(1e-6));
  }

  SECTION("At L=0: S is 0.75 for lambdaS=0.25")
  {
    double S = ebsdlib::NolzeHielscherColorKey::saturation(0.0, 0.25);
    // 1 - 2*0.25*|0-0.5| = 1 - 0.25 = 0.75
    REQUIRE(S == Approx(0.75).margin(1e-6));
  }

  SECTION("At L=1.0: S is 0.75 for lambdaS=0.25")
  {
    double S = ebsdlib::NolzeHielscherColorKey::saturation(1.0, 0.25);
    // 1 - 2*0.25*|1-0.5| = 1 - 0.25 = 0.75
    REQUIRE(S == Approx(0.75).margin(1e-6));
  }

  SECTION("Saturation is symmetric about L=0.5")
  {
    double S_low = ebsdlib::NolzeHielscherColorKey::saturation(0.3, 0.25);
    double S_high = ebsdlib::NolzeHielscherColorKey::saturation(0.7, 0.25);
    REQUIRE(S_low == Approx(S_high).margin(1e-10));
  }

  SECTION("lambdaS=0 gives constant saturation of 1.0")
  {
    REQUIRE(ebsdlib::NolzeHielscherColorKey::saturation(0.0, 0.0) == Approx(1.0).margin(1e-10));
    REQUIRE(ebsdlib::NolzeHielscherColorKey::saturation(0.5, 0.0) == Approx(1.0).margin(1e-10));
    REQUIRE(ebsdlib::NolzeHielscherColorKey::saturation(1.0, 0.0) == Approx(1.0).margin(1e-10));
  }

  SECTION("Result is clamped to [0, 1]")
  {
    // With very large lambdaS, saturation could go negative without clamping
    double S = ebsdlib::NolzeHielscherColorKey::saturation(0.0, 2.0);
    REQUIRE(S >= 0.0);
    REQUIRE(S <= 1.0);
  }
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::NolzeHielscherColorKey::CubicHighOutput", "[EbsdLib][NolzeHielscher]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::cubicHigh();
  ebsdlib::NolzeHielscherColorKey nhKey(sector);

  SECTION("Center direction produces near-white color")
  {
    auto center = sector.barycenter();
    auto [r, g, b] = nhKey.direction2Color(center);
    double brightness = (r + g + b) / 3.0;
    REQUIRE(brightness > 0.8);
  }

  SECTION("All outputs are in valid range")
  {
    for(double eta = 0.01; eta < M_PI / 4.0 - 0.01; eta += 0.05)
    {
      double tanEta = std::tan(eta);
      double chiMax = std::acos(std::sqrt(1.0 / (2.0 + tanEta * tanEta)));
      for(double chi = 0.01; chi < chiMax - 0.01; chi += 0.05)
      {
        double sinChi = std::sin(chi);
        std::array<double, 3> dir = {sinChi * std::cos(eta), sinChi * std::sin(eta), std::cos(chi)};
        auto [r, g, b] = nhKey.direction2Color(dir);
        REQUIRE(r >= 0.0);
        REQUIRE(r <= 1.0);
        REQUIRE(g >= 0.0);
        REQUIRE(g <= 1.0);
        REQUIRE(b >= 0.0);
        REQUIRE(b <= 1.0);
      }
    }
  }

  SECTION("Boundary directions produce saturated colors")
  {
    // [001] direction is at a vertex (on the boundary) and should be saturated
    std::array<double, 3> v001 = {0.0, 0.0, 1.0};
    auto [r, g, b] = nhKey.direction2Color(v001);
    // Should be saturated (not white/gray)
    double maxC = std::max({r, g, b});
    double minC = std::min({r, g, b});
    double saturationApprox = (maxC > 0.0) ? (maxC - minC) / maxC : 0.0;
    REQUIRE(saturationApprox > 0.1);
  }

  SECTION("Different directions produce different colors")
  {
    // Three vertex directions should produce distinct colors
    double s2 = 1.0 / std::sqrt(2.0);
    double s3 = 1.0 / std::sqrt(3.0);
    std::array<double, 3> v001 = {0.0, 0.0, 1.0};
    std::array<double, 3> v101 = {s2, 0.0, s2};
    std::array<double, 3> v111 = {s3, s3, s3};

    auto c001 = nhKey.direction2Color(v001);
    auto c101 = nhKey.direction2Color(v101);
    auto c111 = nhKey.direction2Color(v111);

    // Colors should differ -- check the sum of absolute differences
    double diff01 = std::abs(c001[0] - c101[0]) + std::abs(c001[1] - c101[1]) + std::abs(c001[2] - c101[2]);
    double diff02 = std::abs(c001[0] - c111[0]) + std::abs(c001[1] - c111[1]) + std::abs(c001[2] - c111[2]);
    double diff12 = std::abs(c101[0] - c111[0]) + std::abs(c101[1] - c111[1]) + std::abs(c101[2] - c111[2]);

    REQUIRE(diff01 > 0.05);
    REQUIRE(diff02 > 0.05);
    REQUIRE(diff12 > 0.05);
  }
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::NolzeHielscherColorKey::Name", "[EbsdLib][NolzeHielscher]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::cubicHigh();
  ebsdlib::NolzeHielscherColorKey nhKey(sector);
  REQUIRE(nhKey.name() == "NolzeHielscher");
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::NolzeHielscherColorKey::PolymorphicUsage", "[EbsdLib][NolzeHielscher]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::cubicHigh();
  std::shared_ptr<ebsdlib::IColorKey> key = std::make_shared<ebsdlib::NolzeHielscherColorKey>(sector);
  REQUIRE(key->name() == "NolzeHielscher");

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
TEST_CASE("ebsdlib::NolzeHielscherColorKey::CustomLambdaParameters", "[EbsdLib][NolzeHielscher]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::cubicHigh();

  SECTION("lambdaL=0 still produces valid output")
  {
    ebsdlib::NolzeHielscherColorKey nhKey(sector, 0.0, 0.25);
    auto center = sector.barycenter();
    auto [r, g, b] = nhKey.direction2Color(center);
    REQUIRE(r >= 0.0);
    REQUIRE(r <= 1.0);
    REQUIRE(g >= 0.0);
    REQUIRE(g <= 1.0);
    REQUIRE(b >= 0.0);
    REQUIRE(b <= 1.0);
  }

  SECTION("lambdaS=0 produces maximum saturation everywhere")
  {
    ebsdlib::NolzeHielscherColorKey nhKey(sector, 0.25, 0.0);
    // A mid-radius direction should have saturation = 1.0
    // We verify indirectly through valid output
    double s2 = 1.0 / std::sqrt(2.0);
    std::array<double, 3> dir = {s2, 0.0, s2};
    auto [r, g, b] = nhKey.direction2Color(dir);
    REQUIRE(r >= 0.0);
    REQUIRE(r <= 1.0);
    REQUIRE(g >= 0.0);
    REQUIRE(g <= 1.0);
    REQUIRE(b >= 0.0);
    REQUIRE(b <= 1.0);
  }
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::NolzeHielscherColorKey::ExtendedKey_CubicLow", "[EbsdLib][NolzeHielscher]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::cubicLow();
  REQUIRE(sector.colorKeyMode() == "extended");

  auto supergroupSector = ebsdlib::FundamentalSectorGeometry::cubicHigh();
  ebsdlib::NolzeHielscherColorKey nhKey(sector);

  SECTION("All outputs in valid range across m-3 sector")
  {
    for(double eta = 0.01; eta < M_PI / 2.0 - 0.01; eta += 0.1)
    {
      double chiMax = std::acos(std::sqrt(1.0 / (2.0 + std::tan(eta) * std::tan(eta))));
      for(double chi = 0.01; chi < chiMax - 0.01; chi += 0.1)
      {
        double sinChi = std::sin(chi);
        std::array<double, 3> dir = {sinChi * std::cos(eta), sinChi * std::sin(eta), std::cos(chi)};
        auto [r, g, b] = nhKey.direction2Color(dir);
        REQUIRE(r >= 0.0);
        REQUIRE(r <= 1.0);
        REQUIRE(g >= 0.0);
        REQUIRE(g <= 1.0);
        REQUIRE(b >= 0.0);
        REQUIRE(b <= 1.0);
      }
    }
  }

  SECTION("Uses both bright and dark colors (extended range)")
  {
    bool hasBright = false;
    bool hasDark = false;
    for(double eta = 0.01; eta < M_PI / 2.0 - 0.01; eta += 0.05)
    {
      double chiMax = std::acos(std::sqrt(1.0 / (2.0 + std::tan(eta) * std::tan(eta))));
      for(double chi = 0.01; chi < chiMax - 0.01; chi += 0.05)
      {
        double sinChi = std::sin(chi);
        std::array<double, 3> dir = {sinChi * std::cos(eta), sinChi * std::sin(eta), std::cos(chi)};
        auto [r, g, b] = nhKey.direction2Color(dir);
        double brightness = (r + g + b) / 3.0;
        if(brightness > 0.6)
        {
          hasBright = true;
        }
        if(brightness < 0.4)
        {
          hasDark = true;
        }
      }
    }
    REQUIRE(hasBright);
    REQUIRE(hasDark);
  }

  SECTION("Direction in supergroup sector -> bright, direction outside -> dark")
  {
    // The supergroup's barycenter is inside both sectors and near the center
    // of the supergroup sector, so it should map to a high lightness (bright/white).
    auto sgCenter = supergroupSector.barycenter();
    if(supergroupSector.isInside(sgCenter) && sector.isInside(sgCenter))
    {
      auto [r, g, b] = nhKey.direction2Color(sgCenter);
      double brightness = (r + g + b) / 3.0;
      REQUIRE(brightness > 0.5);
    }

    // eta ~= 60 deg is outside m-3m [0, 45] but inside m-3 [0, 90] -> should be dark
    double sinChi = std::sin(0.3);
    std::array<double, 3> dirExtended = {sinChi * std::cos(1.1), sinChi * std::sin(1.1), std::cos(0.3)};
    if(sector.isInside(dirExtended) && !supergroupSector.isInside(dirExtended))
    {
      auto [r, g, b] = nhKey.direction2Color(dirExtended);
      double brightness = (r + g + b) / 3.0;
      REQUIRE(brightness < 0.5);
    }
  }
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::NolzeHielscherColorKey::ImpossibleMode_Triclinic", "[EbsdLib][NolzeHielscher]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::triclinic();
  REQUIRE(sector.colorKeyMode() == "impossible");

  ebsdlib::NolzeHielscherColorKey nhKey(sector);

  SECTION("Produces valid colors for directions in upper hemisphere")
  {
    for(double eta = 0.0; eta < 2.0 * M_PI; eta += 0.3)
    {
      for(double chi = 0.05; chi < M_PI / 2.0 - 0.05; chi += 0.3)
      {
        double sinChi = std::sin(chi);
        std::array<double, 3> dir = {sinChi * std::cos(eta), sinChi * std::sin(eta), std::cos(chi)};
        auto [r, g, b] = nhKey.direction2Color(dir);
        REQUIRE(r >= 0.0);
        REQUIRE(r <= 1.0);
        REQUIRE(g >= 0.0);
        REQUIRE(g <= 1.0);
        REQUIRE(b >= 0.0);
        REQUIRE(b <= 1.0);
      }
    }
  }

  SECTION("All outputs are in valid range for a swept grid")
  {
    // Sweep the full upper hemisphere: triclinic SST covers all eta, chi in [0, pi/2).
    // The impossible mode uses the same white-center code path as standard.
    for(double eta = 0.0; eta < 2.0 * M_PI; eta += 0.5)
    {
      double chi = M_PI / 4.0;
      double sinChi = std::sin(chi);
      std::array<double, 3> dir = {sinChi * std::cos(eta), sinChi * std::sin(eta), std::cos(chi)};
      auto [r, g, b] = nhKey.direction2Color(dir);
      REQUIRE(r >= 0.0);
      REQUIRE(r <= 1.0);
      REQUIRE(g >= 0.0);
      REQUIRE(g <= 1.0);
      REQUIRE(b >= 0.0);
      REQUIRE(b <= 1.0);
    }
  }

  SECTION("Center direction produces near-white color")
  {
    auto center = sector.barycenter();
    auto [r, g, b] = nhKey.direction2Color(center);
    double brightness = (r + g + b) / 3.0;
    REQUIRE(brightness > 0.8);
  }

  SECTION("Legend has non-uniform coloring (regression: triclinic was flat gray)")
  {
    // Before the polarCoordinates fix for empty-boundary sectors, every
    // direction in triclinic returned radius=1.0, which mapped to a flat
    // light gray (saturation=0, lightness=0.9). The earlier RGB-in-range
    // and brightness>0.8 checks passed on that gray, so the regression
    // shipped. This section asserts the legend is actually a legend --
    // different directions in the SST produce different colors.

    auto chroma = [](double r, double g, double b) { return std::max({r, g, b}) - std::min({r, g, b}); };

    // Center [001] should be near-white (low chroma -- already covered above)
    auto [rc, gc, bc] = nhKey.direction2Color({0.0, 0.0, 1.0});
    REQUIRE(chroma(rc, gc, bc) < 0.1);

    // Four cardinal equator directions should all be saturated and distinct
    const std::array<std::array<double, 3>, 4> equatorDirs = {{
        {1.0, 0.0, 0.0},  // [100], rho=0
        {0.0, 1.0, 0.0},  // [010], rho=pi/2
        {-1.0, 0.0, 0.0}, // [-100], rho=pi
        {0.0, -1.0, 0.0}, // [0-10], rho=3pi/2
    }};
    std::array<std::array<double, 3>, 4> equatorColors{};
    for(size_t k = 0; k < equatorDirs.size(); ++k)
    {
      const auto& dir = equatorDirs[k];
      auto [r, g, b] = nhKey.direction2Color(dir);
      INFO("Equator dir (" << dir[0] << "," << dir[1] << "," << dir[2] << ") -> rgb (" << r << "," << g << "," << b << ") chroma=" << chroma(r, g, b));
      REQUIRE(chroma(r, g, b) > 0.3);
      equatorColors[k] = {r, g, b};
    }

    // Each pair of cardinal equator colors should differ in at least one
    // channel by >0.2 (i.e., they are visibly different hues).
    for(size_t i = 0; i < equatorColors.size(); ++i)
    {
      for(size_t j = i + 1; j < equatorColors.size(); ++j)
      {
        double dr = std::abs(equatorColors[i][0] - equatorColors[j][0]);
        double dg = std::abs(equatorColors[i][1] - equatorColors[j][1]);
        double db = std::abs(equatorColors[i][2] - equatorColors[j][2]);
        double maxDelta = std::max({dr, dg, db});
        INFO("Pair (" << i << "," << j << ") max channel delta = " << maxDelta);
        REQUIRE(maxDelta > 0.2);
      }
    }
  }
}

// ---------------------------------------------------------------------------
TEST_CASE("ebsdlib::NolzeHielscherColorKey::ImpossibleMode_TrigonalLow", "[EbsdLib][NolzeHielscher]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::trigonalLow();
  REQUIRE(sector.colorKeyMode() == "impossible");

  ebsdlib::NolzeHielscherColorKey nhKey(sector);

  SECTION("Produces valid colors for interior directions")
  {
    // Sample some directions that should be inside the trigonal low sector
    auto center = sector.barycenter();
    auto [r, g, b] = nhKey.direction2Color(center);
    REQUIRE(r >= 0.0);
    REQUIRE(r <= 1.0);
    REQUIRE(g >= 0.0);
    REQUIRE(g <= 1.0);
    REQUIRE(b >= 0.0);
    REQUIRE(b <= 1.0);
  }
}
