#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Utilities/ModifiedLambertProjection3D.hpp"

#include <cmath>
#include <vector>

using namespace ebsdlib;
using MLP3D = ModifiedLambertProjection3D<std::vector<double>, double>;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjection3DTest::GetPyramid", "[EbsdLib][ModifiedLambertProjection3DTest]")
{
  // +Z axis -> pyramid 1
  {
    std::vector<double> xyz = {0.0, 0.0, 1.0};
    REQUIRE(MLP3D::GetPyramid(xyz) == 1);
  }
  // -Z axis -> pyramid 2
  {
    std::vector<double> xyz = {0.0, 0.0, -1.0};
    REQUIRE(MLP3D::GetPyramid(xyz) == 2);
  }
  // +X axis -> pyramid 3
  {
    std::vector<double> xyz = {1.0, 0.0, 0.0};
    REQUIRE(MLP3D::GetPyramid(xyz) == 3);
  }
  // -X axis -> pyramid 4
  {
    std::vector<double> xyz = {-1.0, 0.0, 0.0};
    REQUIRE(MLP3D::GetPyramid(xyz) == 4);
  }
  // +Y axis -> pyramid 5
  {
    std::vector<double> xyz = {0.0, 1.0, 0.0};
    REQUIRE(MLP3D::GetPyramid(xyz) == 5);
  }
  // -Y axis -> pyramid 6
  {
    std::vector<double> xyz = {0.0, -1.0, 0.0};
    REQUIRE(MLP3D::GetPyramid(xyz) == 6);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjection3DTest::LambertCubeToBall_Origin", "[EbsdLib][ModifiedLambertProjection3DTest]")
{
  std::vector<double> origin = {0.0, 0.0, 0.0};
  int ierr = 0;
  auto result = MLP3D::LambertCubeToBall(origin, ierr);

  REQUIRE(ierr == 0);
  REQUIRE(result.size() == 3);
  CHECK(result[0] == Approx(0.0).margin(1.0e-10));
  CHECK(result[1] == Approx(0.0).margin(1.0e-10));
  CHECK(result[2] == Approx(0.0).margin(1.0e-10));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjection3DTest::LambertCubeToBall_OutOfRange", "[EbsdLib][ModifiedLambertProjection3DTest]")
{
  // ap/2 ≈ 1.0725, so values > ap/2 + epsilon should return ierr=-1
  std::vector<double> outOfRange = {10.0, 0.0, 0.0};
  int ierr = 0;
  auto result = MLP3D::LambertCubeToBall(outOfRange, ierr);

  REQUIRE(ierr == -1);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjection3DTest::LambertBallToCube_Origin", "[EbsdLib][ModifiedLambertProjection3DTest]")
{
  std::vector<double> origin = {0.0, 0.0, 0.0};
  int ierr = 0;
  auto result = MLP3D::LambertBallToCube(origin, ierr);

  REQUIRE(ierr == 0);
  REQUIRE(result.size() == 3);
  CHECK(result[0] == Approx(0.0).margin(1.0e-10));
  CHECK(result[1] == Approx(0.0).margin(1.0e-10));
  CHECK(result[2] == Approx(0.0).margin(1.0e-10));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjection3DTest::LambertBallToCube_OutOfRange", "[EbsdLib][ModifiedLambertProjection3DTest]")
{
  // R1 ≈ 1.33, so values with magnitude > R1 should return ierr=-1
  std::vector<double> outOfRange = {10.0, 0.0, 0.0};
  int ierr = 0;
  auto result = MLP3D::LambertBallToCube(outOfRange, ierr);

  REQUIRE(ierr == -1);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjection3DTest::RoundTrip_CubeToBallToCube", "[EbsdLib][ModifiedLambertProjection3DTest]")
{
  // Test round-trip for several valid cube points
  std::vector<std::vector<double>> testPoints = {
      {0.1, 0.0, 0.5}, {0.0, 0.3, 0.4}, {-0.2, 0.1, 0.6}, {0.3, -0.3, 0.5}, {0.0, 0.0, 0.8},
  };

  for(const auto& pt : testPoints)
  {
    int ierr1 = 0;
    auto ball = MLP3D::LambertCubeToBall(pt, ierr1);
    REQUIRE(ierr1 == 0);

    int ierr2 = 0;
    auto cube = MLP3D::LambertBallToCube(ball, ierr2);
    REQUIRE(ierr2 == 0);

    CHECK(cube[0] == Approx(pt[0]).margin(1.0e-6));
    CHECK(cube[1] == Approx(pt[1]).margin(1.0e-6));
    CHECK(cube[2] == Approx(pt[2]).margin(1.0e-6));
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjection3DTest::BallOutputBounded", "[EbsdLib][ModifiedLambertProjection3DTest]")
{
  // Ball output should have magnitude <= R1
  std::vector<std::vector<double>> testPoints = {
      {0.5, 0.5, 0.5},
      {-0.5, 0.3, 0.2},
      {0.0, 0.0, 1.0},
  };

  for(const auto& pt : testPoints)
  {
    int ierr = 0;
    auto ball = MLP3D::LambertCubeToBall(pt, ierr);
    REQUIRE(ierr == 0);

    double mag = std::sqrt(ball[0] * ball[0] + ball[1] * ball[1] + ball[2] * ball[2]);
    CHECK(mag <= LambertParametersType::R1 + 1.0e-8);
  }
}
