#include <catch2/catch.hpp>

#include "EbsdLib/Utilities/PoleFigureProjection.h"

#include <array>

using namespace ebsdlib;

TEST_CASE("ebsdlib::PoleFigureProjectionTest::UpperHemisphere", "[EbsdLib][PoleFigureProjectionTest]")
{
  auto north = StereographicProjectUpperHemisphere(0.0f, 0.0f, 1.0f);
  REQUIRE(north[0] == Approx(0.0f));
  REQUIRE(north[1] == Approx(0.0f));

  auto eqX = StereographicProjectUpperHemisphere(1.0f, 0.0f, 0.0f);
  REQUIRE(eqX[0] == Approx(1.0f));
  REQUIRE(eqX[1] == Approx(0.0f));

  auto eqY = StereographicProjectUpperHemisphere(0.0f, 1.0f, 0.0f);
  REQUIRE(eqY[0] == Approx(0.0f));
  REQUIRE(eqY[1] == Approx(1.0f));

  auto south = StereographicProjectUpperHemisphere(0.0f, 0.0f, -1.0f); // folded up
  REQUIRE(south[0] == Approx(0.0f));
  REQUIRE(south[1] == Approx(0.0f));

  const float s = 0.70710678f;
  auto mid = StereographicProjectUpperHemisphere(s, 0.0f, s);
  REQUIRE(mid[0] == Approx(s / (1.0f + s)));
  REQUIRE(mid[1] == Approx(0.0f));

  auto folded = StereographicProjectUpperHemisphere(0.5f, -0.5f, -0.70710678f);
  auto expected = StereographicProjectUpperHemisphere(-0.5f, 0.5f, 0.70710678f);
  REQUIRE(folded[0] == Approx(expected[0]));
  REQUIRE(folded[1] == Approx(expected[1]));
}
