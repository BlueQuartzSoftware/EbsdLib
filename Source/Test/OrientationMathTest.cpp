#include <catch2/catch.hpp>

#include "EbsdLib/Core/OrientationMath.h"
#include "EbsdLib/Math/EbsdLibMath.h"

#include <cmath>
#include <cstdint>

using namespace ebsdlib;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationMathTest::MetricTensorCubic", "[EbsdLib][OrientationMathTest]")
{
  // Cubic: a=b=c=1, alpha=beta=gamma=90 degrees
  float mt[3][3] = {};
  float alpha = static_cast<float>(M_PI_2);
  float beta = static_cast<float>(M_PI_2);
  float gamma = static_cast<float>(M_PI_2);

  OrientationMath::MetricTensorFromLatticeParameters(1.0f, 1.0f, 1.0f, alpha, beta, gamma, mt);

  // For cubic: metric tensor = identity
  REQUIRE(mt[0][0] == Approx(1.0f).margin(1e-5f));
  REQUIRE(mt[1][1] == Approx(1.0f).margin(1e-5f));
  REQUIRE(mt[2][2] == Approx(1.0f).margin(1e-5f));
  REQUIRE(mt[0][1] == Approx(0.0f).margin(1e-5f));
  REQUIRE(mt[0][2] == Approx(0.0f).margin(1e-5f));
  REQUIRE(mt[1][0] == Approx(0.0f).margin(1e-5f));
  REQUIRE(mt[1][2] == Approx(0.0f).margin(1e-5f));
  REQUIRE(mt[2][0] == Approx(0.0f).margin(1e-5f));
  REQUIRE(mt[2][1] == Approx(0.0f).margin(1e-5f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationMathTest::MetricTensorCubicScaled", "[EbsdLib][OrientationMathTest]")
{
  // Cubic with a=b=c=2
  float mt[3][3] = {};
  float alpha = static_cast<float>(M_PI_2);
  float beta = static_cast<float>(M_PI_2);
  float gamma = static_cast<float>(M_PI_2);

  OrientationMath::MetricTensorFromLatticeParameters(2.0f, 2.0f, 2.0f, alpha, beta, gamma, mt);

  // For cubic a=2: diagonal = a^2 = 4
  REQUIRE(mt[0][0] == Approx(4.0f).margin(1e-5f));
  REQUIRE(mt[1][1] == Approx(4.0f).margin(1e-5f));
  REQUIRE(mt[2][2] == Approx(4.0f).margin(1e-5f));
  REQUIRE(mt[0][1] == Approx(0.0f).margin(1e-5f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationMathTest::MetricTensorSymmetric", "[EbsdLib][OrientationMathTest]")
{
  // The metric tensor should always be symmetric
  float mt[3][3] = {};
  float alpha = 1.2f;
  float beta = 1.3f;
  float gamma = 1.1f;

  OrientationMath::MetricTensorFromLatticeParameters(2.0f, 3.0f, 4.0f, alpha, beta, gamma, mt);

  REQUIRE(mt[0][1] == Approx(mt[1][0]).margin(1e-5f));
  REQUIRE(mt[0][2] == Approx(mt[2][0]).margin(1e-5f));
  REQUIRE(mt[1][2] == Approx(mt[2][1]).margin(1e-5f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationMathTest::RootTensorCubic", "[EbsdLib][OrientationMathTest]")
{
  float rt[3][3] = {};
  float alpha = static_cast<float>(M_PI_2);
  float beta = static_cast<float>(M_PI_2);
  float gamma = static_cast<float>(M_PI_2);

  OrientationMath::RootTensorFromLatticeParameters(1.0f, 1.0f, 1.0f, alpha, beta, gamma, rt);

  // For cubic with 90-degree angles:
  // rt[0][0] = a*sin(beta)*sin(gamma) = 1*1*1 = 1
  REQUIRE(rt[0][0] == Approx(1.0f).margin(1e-5f));
  // rt[1][1] = b*sin(alpha) = 1*1 = 1
  REQUIRE(rt[1][1] == Approx(1.0f).margin(1e-5f));
  // rt[2][2] = c = 1
  REQUIRE(rt[2][2] == Approx(1.0f).margin(1e-5f));
  // Off-diagonals should be ~0 for cubic
  REQUIRE(rt[0][1] == Approx(0.0f).margin(1e-5f));
  REQUIRE(rt[0][2] == Approx(0.0f).margin(1e-5f));
  REQUIRE(rt[1][2] == Approx(0.0f).margin(1e-5f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationMathTest::MillerBravaisToMillerDirection", "[EbsdLib][OrientationMathTest]")
{
  // [2 -1 -1 0] -> [2-(-1), -1-(-1), 0] = [3, 0, 0]
  int32_t millerBravais[4] = {2, -1, -1, 0};
  int32_t miller[3] = {};

  OrientationMath::MillerBravaisToMillerDirection(millerBravais, miller);
  REQUIRE(miller[0] == 3);
  REQUIRE(miller[1] == 0);
  REQUIRE(miller[2] == 0);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationMathTest::MillerToMillerBravaisDirection", "[EbsdLib][OrientationMathTest]")
{
  int32_t miller[3] = {1, 0, 0};
  int32_t millerBravais[4] = {};

  OrientationMath::MillerToMillerBravaisDirection(miller, millerBravais);

  // U = (2*1 - 0)/3 = 0 (truncated from 0.66)
  // V = (2*0 - 1)/3 = 0 (truncated from -0.33)
  // T = -(1+0)/3 = 0 (truncated from -0.33)
  // W = 0
  // Check that U + V + T = 0 approximately (Miller-Bravais constraint)
  REQUIRE(millerBravais[0] + millerBravais[1] + millerBravais[2] == 0);
  REQUIRE(millerBravais[3] == 0);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationMathTest::MillerBravaisToMillerPlane", "[EbsdLib][OrientationMathTest]")
{
  // For planes: (H K I L) -> (H K L), I is dropped
  int32_t millerBravais[4] = {1, 0, -1, 2};
  int32_t miller[3] = {};

  OrientationMath::MillerBravaisToMillerPlane(millerBravais, miller);
  REQUIRE(miller[0] == 1);
  REQUIRE(miller[1] == 0);
  REQUIRE(miller[2] == 2);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationMathTest::MillerToMillerBravaisPlane", "[EbsdLib][OrientationMathTest]")
{
  int32_t miller[3] = {1, 1, 0};
  int32_t millerBravais[4] = {};

  OrientationMath::MillerToMillerBravaisPlane(miller, millerBravais);

  REQUIRE(millerBravais[0] == 1);
  REQUIRE(millerBravais[1] == 1);
  REQUIRE(millerBravais[2] == -2); // -(H + K) = -(1+1) = -2
  REQUIRE(millerBravais[3] == 0);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationMathTest::MillerBravaisPlaneRoundTrip", "[EbsdLib][OrientationMathTest]")
{
  // Round-trip: Miller -> MillerBravais -> Miller should give back the original
  int32_t millerOrig[3] = {1, 2, 3};
  int32_t millerBravais[4] = {};
  int32_t millerRecovered[3] = {};

  OrientationMath::MillerToMillerBravaisPlane(millerOrig, millerBravais);
  OrientationMath::MillerBravaisToMillerPlane(millerBravais, millerRecovered);

  REQUIRE(millerRecovered[0] == millerOrig[0]);
  REQUIRE(millerRecovered[1] == millerOrig[1]);
  REQUIRE(millerRecovered[2] == millerOrig[2]);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationMathTest::MillerBravaisPlaneConstraint", "[EbsdLib][OrientationMathTest]")
{
  // The Miller-Bravais plane indices must satisfy H + K + I = 0
  int32_t miller[3] = {2, 1, 0};
  int32_t millerBravais[4] = {};

  OrientationMath::MillerToMillerBravaisPlane(miller, millerBravais);

  REQUIRE(millerBravais[0] + millerBravais[1] + millerBravais[2] == 0);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationMathTest::MillerBravaisDirectionKnownValues", "[EbsdLib][OrientationMathTest]")
{
  // Known crystallographic direction: [1 1 -2 0] in MB -> [1-(-2), 1-(-2), 0] = [3, 3, 0] in Miller
  int32_t millerBravais[4] = {1, 1, -2, 0};
  int32_t miller[3] = {};

  OrientationMath::MillerBravaisToMillerDirection(millerBravais, miller);
  REQUIRE(miller[0] == 3);
  REQUIRE(miller[1] == 3);
  REQUIRE(miller[2] == 0);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationMathTest::MetricTensorHexagonal", "[EbsdLib][OrientationMathTest]")
{
  // Hexagonal: a=b, c different, alpha=beta=90, gamma=120 degrees
  float mt[3][3] = {};
  float a = 2.0f;
  float c = 3.0f;
  float alpha = static_cast<float>(M_PI_2);
  float beta = static_cast<float>(M_PI_2);
  float gamma = static_cast<float>(2.0 * M_PI / 3.0); // 120 degrees in radians

  OrientationMath::MetricTensorFromLatticeParameters(a, a, c, alpha, beta, gamma, mt);

  // g11 = a^2 = 4
  REQUIRE(mt[0][0] == Approx(4.0f).margin(1e-4f));
  // g22 = b^2 = 4
  REQUIRE(mt[1][1] == Approx(4.0f).margin(1e-4f));
  // g33 = c^2 = 9
  REQUIRE(mt[2][2] == Approx(9.0f).margin(1e-4f));
  // g12 = a*b*cos(gamma) = 4*cos(120) = 4*(-0.5) = -2
  REQUIRE(mt[0][1] == Approx(-2.0f).margin(1e-4f));
  // g13 = a*c*cos(beta) = 2*3*cos(90) = 0
  REQUIRE(mt[0][2] == Approx(0.0f).margin(1e-4f));
}
