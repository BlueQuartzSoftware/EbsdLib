#include <catch2/catch.hpp>

#include "EbsdLib/Utilities/FundamentalSectorGeometry.hpp"
#include "EbsdLib/Math/EbsdLibMath.h"

#include <cmath>

using Vec3 = std::array<double, 3>;

static Vec3 normalize(Vec3 v)
{
  double len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  return {v[0] / len, v[1] / len, v[2] / len};
}

TEST_CASE("ebsdlib::FundamentalSectorGeometry::CubicHighVertices", "[EbsdLib][FundamentalSector]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::cubicHigh();

  SECTION("Has 3 vertices")
  {
    REQUIRE(sector.vertices().size() == 3);
  }

  SECTION("Vertices are [001], [101], [111]")
  {
    auto verts = sector.vertices();
    // [001] = {0, 0, 1}
    REQUIRE(verts[0][2] == Approx(1.0).margin(1e-6));
    // [101] = {s2, 0, s2}
    REQUIRE(verts[1][0] == Approx(1.0 / std::sqrt(2.0)).margin(1e-6));
    REQUIRE(verts[1][2] == Approx(1.0 / std::sqrt(2.0)).margin(1e-6));
    // [111] = {s3, s3, s3}
    REQUIRE(verts[2][0] == Approx(1.0 / std::sqrt(3.0)).margin(1e-6));
  }

  SECTION("Barycenter is normalized mean of vertices")
  {
    auto center = sector.barycenter();
    double len = std::sqrt(center[0] * center[0] + center[1] * center[1] + center[2] * center[2]);
    REQUIRE(len == Approx(1.0).margin(1e-6));
  }
}

TEST_CASE("ebsdlib::FundamentalSectorGeometry::PolarCoordinates", "[EbsdLib][FundamentalSector]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::cubicHigh();

  SECTION("At barycenter: radius = 1 (center of sector)")
  {
    auto center = sector.barycenter();
    auto [radius, rho] = sector.polarCoordinates(center);
    // Convention: radius=1 at center, 0 at boundary (orix/MTEX convention)
    REQUIRE(radius == Approx(1.0).margin(1e-4));
  }

  SECTION("At vertex [001]: radius near 0 (on boundary)")
  {
    Vec3 v001 = {0.0, 0.0, 1.0};
    auto [radius, rho] = sector.polarCoordinates(v001);
    REQUIRE(radius == Approx(0.0).margin(0.05));
  }

  SECTION("At vertex [101]: radius near 0 (on boundary)")
  {
    Vec3 v101 = normalize({1.0, 0.0, 1.0});
    auto [radius, rho] = sector.polarCoordinates(v101);
    REQUIRE(radius == Approx(0.0).margin(0.05));
  }

  SECTION("At vertex [111]: radius near 0 (on boundary)")
  {
    Vec3 v111 = normalize({1.0, 1.0, 1.0});
    auto [radius, rho] = sector.polarCoordinates(v111);
    REQUIRE(radius == Approx(0.0).margin(0.05));
  }

  SECTION("Radius is in [0, 1] for interior point")
  {
    // Use a point well inside the cubic high SST: eta ~ 20deg, chi ~ 20deg
    Vec3 interior = normalize({0.3, 0.1, 1.0});
    auto [radius, rho] = sector.polarCoordinates(interior);
    REQUIRE(radius >= 0.0);
    REQUIRE(radius <= 1.0);
  }

  SECTION("Rho is in [0, 2*pi)")
  {
    Vec3 interior = normalize({0.3, 0.1, 1.0});
    auto [radius, rho] = sector.polarCoordinates(interior);
    REQUIRE(rho >= 0.0);
    REQUIRE(rho < ebsdlib::constants::k_2PiD);
  }
}

TEST_CASE("ebsdlib::FundamentalSectorGeometry::EdgeCases", "[EbsdLib][FundamentalSector]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::cubicHigh();

  SECTION("Direction exactly at barycenter returns radius = 1 (center)")
  {
    auto center = sector.barycenter();
    auto [radius, rho] = sector.polarCoordinates(center);
    REQUIRE(radius == Approx(1.0).margin(1e-6));
  }

  SECTION("isInside returns true for interior, false for exterior")
  {
    REQUIRE(sector.isInside({0.0, 0.0, 1.0}));
    // Interior point: eta ~ 18deg < 45deg, small chi
    REQUIRE(sector.isInside(normalize({0.3, 0.1, 1.0})));
    // [100] is outside (violates hypotenuse boundary)
    REQUIRE_FALSE(sector.isInside({1.0, 0.0, 0.0}));
  }
}

TEST_CASE("ebsdlib::FundamentalSectorGeometry::NonTriangularSectors", "[EbsdLib][FundamentalSector]")
{
  SECTION("Cubic low (m-3) has 4 vertices")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::cubicLow();
    REQUIRE(sector.vertices().size() == 4);
    REQUIRE(sector.colorKeyMode() == "extended");
  }

  SECTION("Triclinic (-1) has 0 vertices and covers upper hemisphere")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::triclinic();
    REQUIRE(sector.vertices().empty());
    REQUIRE(sector.colorKeyMode() == "impossible");
    REQUIRE(sector.isInside({0.0, 0.0, 1.0}));
    REQUIRE(sector.isInside(normalize({0.5, 0.5, 0.1})));
  }

  SECTION("Monoclinic (2/m) is extended")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::monoclinic();
    REQUIRE(sector.colorKeyMode() == "extended");
  }
}

TEST_CASE("ebsdlib::FundamentalSectorGeometry::AllLaueGroups", "[EbsdLib][FundamentalSector]")
{
  std::vector<ebsdlib::FundamentalSectorGeometry> sectors = {
      ebsdlib::FundamentalSectorGeometry::cubicHigh(),    ebsdlib::FundamentalSectorGeometry::cubicLow(),       ebsdlib::FundamentalSectorGeometry::hexagonalHigh(),
      ebsdlib::FundamentalSectorGeometry::hexagonalLow(), ebsdlib::FundamentalSectorGeometry::tetragonalHigh(), ebsdlib::FundamentalSectorGeometry::tetragonalLow(),
      ebsdlib::FundamentalSectorGeometry::trigonalHigh(), ebsdlib::FundamentalSectorGeometry::trigonalLow(),    ebsdlib::FundamentalSectorGeometry::orthorhombic(),
      ebsdlib::FundamentalSectorGeometry::monoclinic(),   ebsdlib::FundamentalSectorGeometry::triclinic(),
  };

  for(size_t i = 0; i < sectors.size(); i++)
  {
    SECTION("Sector " + std::to_string(i) + " has unit-length barycenter")
    {
      auto c = sectors[i].barycenter();
      double len = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
      REQUIRE(len == Approx(1.0).margin(1e-6));
    }
  }
}

TEST_CASE("ebsdlib::FundamentalSectorGeometry::CorrectAzimuthalAngle", "[EbsdLib][FundamentalSector]")
{
  auto sector = ebsdlib::FundamentalSectorGeometry::cubicHigh();

  SECTION("Correction produces monotonically increasing output")
  {
    // The corrected angle should increase monotonically with input angle
    double prev = 0.0;
    for(double rho = 0.01; rho < ebsdlib::constants::k_2PiD - 0.01; rho += 0.05)
    {
      double corrected = sector.correctAzimuthalAngle(rho);
      REQUIRE(corrected >= prev - 0.01); // monotonic (with small tolerance)
      prev = corrected;
    }
  }

  SECTION("Result is in [0, 2*pi)")
  {
    double corrected = sector.correctAzimuthalAngle(-0.5);
    REQUIRE(corrected >= 0.0);
    REQUIRE(corrected < ebsdlib::constants::k_2PiD + 0.01);
  }
}

TEST_CASE("ebsdlib::FundamentalSectorGeometry::IsInsideBoundaryPoints", "[EbsdLib][FundamentalSector]")
{
  SECTION("Cubic high: all vertices are inside or on boundary")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::cubicHigh();
    for(const auto& v : sector.vertices())
    {
      REQUIRE(sector.isInside(v));
    }
  }

  SECTION("Hexagonal high: all vertices are inside or on boundary")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::hexagonalHigh();
    for(const auto& v : sector.vertices())
    {
      REQUIRE(sector.isInside(v));
    }
  }

  SECTION("Tetragonal high: all vertices are inside or on boundary")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::tetragonalHigh();
    for(const auto& v : sector.vertices())
    {
      REQUIRE(sector.isInside(v));
    }
  }

  SECTION("Trigonal high: all vertices are inside or on boundary")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::trigonalHigh();
    for(const auto& v : sector.vertices())
    {
      REQUIRE(sector.isInside(v));
    }
  }

  SECTION("Orthorhombic: all vertices are inside or on boundary")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::orthorhombic();
    for(const auto& v : sector.vertices())
    {
      REQUIRE(sector.isInside(v));
    }
  }

  SECTION("Cubic low: all vertices are inside or on boundary")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::cubicLow();
    for(const auto& v : sector.vertices())
    {
      REQUIRE(sector.isInside(v));
    }
  }

  SECTION("Trigonal low: all vertices are inside or on boundary")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::trigonalLow();
    for(const auto& v : sector.vertices())
    {
      REQUIRE(sector.isInside(v));
    }
  }

  SECTION("Monoclinic: all vertices are inside or on boundary")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::monoclinic();
    for(const auto& v : sector.vertices())
    {
      REQUIRE(sector.isInside(v));
    }
  }
}

TEST_CASE("ebsdlib::FundamentalSectorGeometry::BarycenterIsInside", "[EbsdLib][FundamentalSector]")
{
  SECTION("Cubic high barycenter is inside")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::cubicHigh();
    REQUIRE(sector.isInside(sector.barycenter()));
  }

  SECTION("Hexagonal high barycenter is inside")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::hexagonalHigh();
    REQUIRE(sector.isInside(sector.barycenter()));
  }

  SECTION("Cubic low barycenter is inside")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::cubicLow();
    REQUIRE(sector.isInside(sector.barycenter()));
  }

  SECTION("Trigonal high barycenter is inside")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::trigonalHigh();
    REQUIRE(sector.isInside(sector.barycenter()));
  }

  SECTION("Trigonal low barycenter is inside")
  {
    auto sector = ebsdlib::FundamentalSectorGeometry::trigonalLow();
    REQUIRE(sector.isInside(sector.barycenter()));
  }
}
