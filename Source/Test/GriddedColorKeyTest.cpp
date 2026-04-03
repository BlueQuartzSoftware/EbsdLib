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

TEST_CASE("ebsdlib::GriddedColorKey::LaueOpsIntegration", "[EbsdLib][GriddedColorKey]")
{
  auto allOps = ebsdlib::LaueOps::GetAllOrientationOps();
  auto& cubicOps = *allOps[1]; // Cubic_High

  SECTION("Can set gridded color key on LaueOps")
  {
    auto nhKey = std::make_shared<ebsdlib::NolzeHielscherColorKey>(ebsdlib::FundamentalSectorGeometry::cubicHigh());
    auto gridKey = std::make_shared<ebsdlib::GriddedColorKey>(nhKey, 1.0);
    cubicOps.setColorKey(gridKey);
    REQUIRE(cubicOps.getColorKey()->name() == "NolzeHielscher (gridded)");

    // Generate a legend with the gridded key
    auto legend = cubicOps.generateIPFTriangleLegend(64, false);
    REQUIRE(legend != nullptr);
    REQUIRE(legend->getNumberOfTuples() > 0);

    // Reset
    cubicOps.setColorKey(std::make_shared<ebsdlib::TSLColorKey>());
  }

  SECTION("IPF colors work with gridded key")
  {
    auto nhKey = std::make_shared<ebsdlib::NolzeHielscherColorKey>(ebsdlib::FundamentalSectorGeometry::cubicHigh());
    auto gridKey = std::make_shared<ebsdlib::GriddedColorKey>(nhKey, 1.0);
    cubicOps.setColorKey(gridKey);

    double eulers[3] = {0.5, 0.3, 0.2};
    double refDir[3] = {0.0, 0.0, 1.0};
    ebsdlib::Rgb color = cubicOps.generateIPFColor(eulers, refDir, false);
    int r = ebsdlib::RgbColor::dRed(color);
    int g = ebsdlib::RgbColor::dGreen(color);
    int b = ebsdlib::RgbColor::dBlue(color);
    REQUIRE((r + g + b) > 0);

    cubicOps.setColorKey(std::make_shared<ebsdlib::TSLColorKey>());
  }
}

TEST_CASE("ebsdlib::GriddedColorKey::SetLegendRenderMode", "[EbsdLib][GriddedColorKey]")
{
  auto allOps = ebsdlib::LaueOps::GetAllOrientationOps();
  auto& cubicOps = *allOps[1]; // Cubic_High

  SECTION("Switch to GridInterpolated mode wraps the color key")
  {
    cubicOps.setColorKey(std::make_shared<ebsdlib::TSLColorKey>());
    cubicOps.setLegendRenderMode(ebsdlib::LegendRenderMode::GridInterpolated, 2.0);
    REQUIRE(cubicOps.getColorKey()->name() == "TSL (gridded)");
  }

  SECTION("Switch back to PerPixel mode unwraps the color key")
  {
    cubicOps.setColorKey(std::make_shared<ebsdlib::TSLColorKey>());
    cubicOps.setLegendRenderMode(ebsdlib::LegendRenderMode::GridInterpolated, 2.0);
    cubicOps.setLegendRenderMode(ebsdlib::LegendRenderMode::PerPixel);
    REQUIRE(cubicOps.getColorKey()->name() == "TSL");
  }

  SECTION("Double-wrapping is prevented")
  {
    cubicOps.setColorKey(std::make_shared<ebsdlib::TSLColorKey>());
    cubicOps.setLegendRenderMode(ebsdlib::LegendRenderMode::GridInterpolated, 2.0);
    cubicOps.setLegendRenderMode(ebsdlib::LegendRenderMode::GridInterpolated, 1.0);
    // Should still have only one layer of wrapping
    REQUIRE(cubicOps.getColorKey()->name() == "TSL (gridded)");
    auto griddedKey = std::dynamic_pointer_cast<ebsdlib::GriddedColorKey>(cubicOps.getColorKey());
    REQUIRE(griddedKey != nullptr);
    REQUIRE(griddedKey->resolutionDeg() == Approx(1.0));
    REQUIRE(griddedKey->innerKey()->name() == "TSL");
  }

  // Reset to default
  cubicOps.setColorKey(std::make_shared<ebsdlib::TSLColorKey>());
}
