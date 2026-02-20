#include <catch2/catch.hpp>

#include "EbsdLib/Math/Matrix3X1.hpp"
#include "EbsdLib/Utilities/ComputeStereographicProjection.h"

#include <cmath>
#include <vector>

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::StereographicProjectionTest::StereoToSpherical_Origin", "[EbsdLib][StereographicProjectionTest]")
{
  // Origin (0,0) in stereographic should map to north pole (0,0,1)
  auto result = stereographic::utils::StereoToSpherical<double>(0.0, 0.0);
  REQUIRE(result[0] == Approx(0.0));
  REQUIRE(result[1] == Approx(0.0));
  REQUIRE(result[2] == Approx(1.0));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::StereographicProjectionTest::SphericalToStereo_NorthPole", "[EbsdLib][StereographicProjectionTest]")
{
  // North pole (0,0,1) should map to origin in stereographic
  auto result = stereographic::utils::SphericalToStereo<double>(0.0, 0.0, 1.0);
  REQUIRE(result[0] == Approx(0.0));
  REQUIRE(result[1] == Approx(0.0));
  REQUIRE(result[2] == Approx(0.0));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::StereographicProjectionTest::RoundTrip_StereoToSphericalToStereo", "[EbsdLib][StereographicProjectionTest]")
{
  // Round-trip: SphericalToStereo(StereoToSpherical(p)) should approximately equal p
  std::vector<std::pair<double, double>> testPoints = {{0.1, 0.2}, {-0.3, 0.4}, {0.5, -0.1}, {0.0, 0.3}};

  for(const auto& [x, y] : testPoints)
  {
    ebsdlib::Matrix3X1<double> stereo(x, y, 0.0);
    auto sphere = stereographic::utils::StereoToSpherical<double>(stereo);
    auto backToStereo = stereographic::utils::SphericalToStereo<double>(sphere);
    CHECK(backToStereo[0] == Approx(x).margin(1.0e-10));
    CHECK(backToStereo[1] == Approx(y).margin(1.0e-10));
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::StereographicProjectionTest::RoundTrip_SphericalToStereoToSpherical", "[EbsdLib][StereographicProjectionTest]")
{
  // Round-trip for unit sphere points with z > -1
  std::vector<ebsdlib::Matrix3X1<double>> spherePoints = {
      {0.0, 0.0, 1.0},                                                    // north pole
      {1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0)}, // (1,1,1)/sqrt(3)
      {1.0 / std::sqrt(2.0), 0.0, 1.0 / std::sqrt(2.0)},                  // 45 degrees
      {std::sqrt(3.0) / 2.0, 0.0, 0.5},                                   // 60 degrees from pole
  };

  for(const auto& pt : spherePoints)
  {
    auto stereo = stereographic::utils::SphericalToStereo<double>(pt);
    auto backToSphere = stereographic::utils::StereoToSpherical<double>(stereo);
    CHECK(backToSphere[0] == Approx(pt[0]).margin(1.0e-10));
    CHECK(backToSphere[1] == Approx(pt[1]).margin(1.0e-10));
    CHECK(backToSphere[2] == Approx(pt[2]).margin(1.0e-10));
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::StereographicProjectionTest::EquatorPoint", "[EbsdLib][StereographicProjectionTest]")
{
  // Equator point (1,0,0): stereo = x/(1+z) = 1/(1+0) = 1, y/(1+z) = 0
  auto stereo = stereographic::utils::SphericalToStereo<double>(1.0, 0.0, 0.0);
  REQUIRE(stereo[0] == Approx(1.0));
  REQUIRE(stereo[1] == Approx(0.0));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::StereographicProjectionTest::TransformUnitSphereToStereographicCoords", "[EbsdLib][StereographicProjectionTest]")
{
  std::vector<ebsdlib::Matrix3X1<float>> points;

  // Northern hemisphere point
  points.push_back({0.0f, 0.0f, 1.0f}); // north pole

  // Southern hemisphere point
  points.push_back({0.0f, 0.0f, -1.0f}); // south pole - z < 0 uses (1-z) denominator

  // Equator
  points.push_back({1.0f, 0.0f, 0.0f});

  auto result = stereographic::utils::TransformUnitSphereToStereographicCoords<float>(points);
  REQUIRE(result.size() == 3);

  // North pole -> origin
  CHECK(result[0][0] == Approx(0.0f).margin(1.0e-6f));
  CHECK(result[0][1] == Approx(0.0f).margin(1.0e-6f));

  // South pole with southern projection: x/(1-z) = 0/(1-(-1)) = 0
  CHECK(result[1][0] == Approx(0.0f).margin(1.0e-6f));
  CHECK(result[1][1] == Approx(0.0f).margin(1.0e-6f));

  // Equator (1,0,0) with z>=0: x/(1+z) = 1/(1+0) = 1
  CHECK(result[2][0] == Approx(1.0f).margin(1.0e-6f));
  CHECK(result[2][1] == Approx(0.0f).margin(1.0e-6f));
}
