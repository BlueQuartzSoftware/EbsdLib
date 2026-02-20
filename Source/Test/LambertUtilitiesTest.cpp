#include <catch2/catch.hpp>

#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Utilities/LambertUtilities.h"

#include <cmath>

using namespace ebsdlib;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::LambertUtilitiesTest::Origin_NorthHemisphere", "[EbsdLib][LambertUtilitiesTest]")
{
  // Origin (0,0,0) should map to the north pole (0,0,1)
  float vert[3] = {0.0f, 0.0f, 0.0f};
  int32_t result = LambertUtilities::LambertSquareVertToSphereVert(vert, LambertUtilities::Hemisphere::North);
  REQUIRE(result == 0);
  REQUIRE(vert[0] == Approx(0.0f).margin(1.0e-6f));
  REQUIRE(vert[1] == Approx(0.0f).margin(1.0e-6f));
  REQUIRE(vert[2] == Approx(1.0f).margin(1.0e-6f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::LambertUtilitiesTest::SouthHemisphere_NonZeroInput", "[EbsdLib][LambertUtilitiesTest]")
{
  // For south hemisphere, hemiFactor = 1.0 which produces z = (2a^2/pi*r) - r
  // For north hemisphere, hemiFactor = -1.0 which produces z = -((2a^2/pi*r) - r) = r - 2a^2/pi*r
  // At the same non-zero input, south should differ from north in z sign
  float vertNorth[3] = {0.5f, 0.3f, 0.0f};
  float vertSouth[3] = {0.5f, 0.3f, 0.0f};

  LambertUtilities::LambertSquareVertToSphereVert(vertNorth, LambertUtilities::Hemisphere::North);
  LambertUtilities::LambertSquareVertToSphereVert(vertSouth, LambertUtilities::Hemisphere::South);

  // x and y should be the same, z should have opposite signs
  CHECK(vertNorth[0] == Approx(vertSouth[0]).margin(1.0e-5f));
  CHECK(vertNorth[1] == Approx(vertSouth[1]).margin(1.0e-5f));
  CHECK(vertNorth[2] == Approx(-vertSouth[2]).margin(1.0e-5f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::LambertUtilitiesTest::OutputOnUnitSphere", "[EbsdLib][LambertUtilitiesTest]")
{
  // Test several points to verify output is on the unit sphere
  float halfRange = static_cast<float>(std::sqrt(ebsdlib::constants::k_PiD / 2.0));
  float testValues[] = {0.0f, 0.3f, -0.3f, 0.6f, -0.6f, halfRange * 0.5f};

  for(float x : testValues)
  {
    for(float y : testValues)
    {
      float vert[3] = {x, y, 0.0f};
      int32_t result = LambertUtilities::LambertSquareVertToSphereVert(vert, LambertUtilities::Hemisphere::North);
      REQUIRE(result == 0);

      float mag = std::sqrt(vert[0] * vert[0] + vert[1] * vert[1] + vert[2] * vert[2]);
      CHECK(mag == Approx(1.0f).margin(1.0e-4f));
    }
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::LambertUtilitiesTest::ReturnValueValid", "[EbsdLib][LambertUtilitiesTest]")
{
  float vert[3] = {0.5f, 0.5f, 0.0f};
  int32_t result = LambertUtilities::LambertSquareVertToSphereVert(vert, LambertUtilities::Hemisphere::North);
  REQUIRE(result == 0);
}
