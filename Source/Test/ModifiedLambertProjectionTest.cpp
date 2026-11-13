#include <catch2/catch.hpp>

#include "EbsdLib/LaueOps/CubicOps.h"
#include "EbsdLib/Utilities/ComputeStereographicProjection.h"
#include "EbsdLib/Utilities/InversePoleFigureUtilities.h"
#include "EbsdLib/Utilities/ModifiedLambertProjection.h"
#include "EbsdLib/Utilities/PoleFigureUtilities.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

using namespace ebsdlib;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionTest::InitializeSquares", "[EbsdLib][ModifiedLambertProjectionTest]")
{
  auto mlp = ModifiedLambertProjection::New();
  REQUIRE(mlp != nullptr);

  int dim = 10;
  float sphereRadius = 1.0f;
  mlp->initializeSquares(dim, sphereRadius);

  REQUIRE(mlp->getDimension() == dim);
  REQUIRE(mlp->getSphereRadius() == Approx(sphereRadius));
  CHECK(mlp->getStepSize() > 0.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionTest::SquareArrays", "[EbsdLib][ModifiedLambertProjectionTest]")
{
  auto mlp = ModifiedLambertProjection::New();
  int dim = 8;
  float sphereRadius = 1.0f;
  mlp->initializeSquares(dim, sphereRadius);

  auto northSquare = mlp->getNorthSquare();
  auto southSquare = mlp->getSouthSquare();
  REQUIRE(northSquare != nullptr);
  REQUIRE(southSquare != nullptr);

  // Size should be dim * dim
  REQUIRE(northSquare->getNumberOfTuples() == static_cast<size_t>(dim * dim));
  REQUIRE(southSquare->getNumberOfTuples() == static_cast<size_t>(dim * dim));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionTest::SetGetValue", "[EbsdLib][ModifiedLambertProjectionTest]")
{
  auto mlp = ModifiedLambertProjection::New();
  int dim = 8;
  mlp->initializeSquares(dim, 1.0f);

  // Set and get a value in the north square
  mlp->setValue(ModifiedLambertProjection::NorthSquare, 0, 42.0);
  REQUIRE(mlp->getValue(ModifiedLambertProjection::NorthSquare, 0) == Approx(42.0));

  // Set and get a value in the south square
  mlp->setValue(ModifiedLambertProjection::SouthSquare, 5, 99.0);
  REQUIRE(mlp->getValue(ModifiedLambertProjection::SouthSquare, 5) == Approx(99.0));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionTest::AddValue", "[EbsdLib][ModifiedLambertProjectionTest]")
{
  auto mlp = ModifiedLambertProjection::New();
  int dim = 8;
  mlp->initializeSquares(dim, 1.0f);

  mlp->setValue(ModifiedLambertProjection::NorthSquare, 3, 10.0);
  mlp->addValue(ModifiedLambertProjection::NorthSquare, 3, 5.0);
  REQUIRE(mlp->getValue(ModifiedLambertProjection::NorthSquare, 3) == Approx(15.0));

  mlp->addValue(ModifiedLambertProjection::NorthSquare, 3, 2.5);
  REQUIRE(mlp->getValue(ModifiedLambertProjection::NorthSquare, 3) == Approx(17.5));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionTest::NormalizeSquares", "[EbsdLib][ModifiedLambertProjectionTest]")
{
  auto mlp = ModifiedLambertProjection::New();
  int dim = 4;
  mlp->initializeSquares(dim, 1.0f);

  // Set some values in both squares
  for(int i = 0; i < dim * dim; i++)
  {
    mlp->setValue(ModifiedLambertProjection::NorthSquare, i, 1.0);
    mlp->setValue(ModifiedLambertProjection::SouthSquare, i, 1.0);
  }

  mlp->normalizeSquares();

  // After normalization, each hemisphere sums to 1.0 independently
  double northTotal = 0.0;
  double southTotal = 0.0;
  for(int i = 0; i < dim * dim; i++)
  {
    northTotal += mlp->getValue(ModifiedLambertProjection::NorthSquare, i);
    southTotal += mlp->getValue(ModifiedLambertProjection::SouthSquare, i);
  }
  REQUIRE(northTotal == Approx(1.0).margin(1.0e-6));
  REQUIRE(southTotal == Approx(1.0).margin(1.0e-6));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionTest::GetSquareCoord_NorthPole", "[EbsdLib][ModifiedLambertProjectionTest]")
{
  auto mlp = ModifiedLambertProjection::New();
  int dim = 16;
  mlp->initializeSquares(dim, 1.0f);

  // North pole (0,0,1) should return north square (true)
  float northPole[3] = {0.0f, 0.0f, 1.0f};
  float sqCoord[2] = {0.0f, 0.0f};
  bool isNorth = mlp->getSquareCoord(northPole, sqCoord);
  REQUIRE(isNorth == true);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionTest::GetSquareCoord_SouthPole", "[EbsdLib][ModifiedLambertProjectionTest]")
{
  auto mlp = ModifiedLambertProjection::New();
  int dim = 16;
  mlp->initializeSquares(dim, 1.0f);

  // South pole (0,0,-1) should return south square (false)
  float southPole[3] = {0.0f, 0.0f, -1.0f};
  float sqCoord[2] = {0.0f, 0.0f};
  bool isNorth = mlp->getSquareCoord(southPole, sqCoord);
  REQUIRE(isNorth == false);
}

namespace
{
// Unit vectors along +x, -x, +y, +z and -z: points that land on the edge, the middle and both squares.
ebsdlib::FloatArrayType::Pointer CreateUnitSphereCoords()
{
  const std::vector<float> xyz = {1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, -1.0f};
  auto coords = ebsdlib::FloatArrayType::CreateArray(xyz.size() / 3, std::vector<size_t>{3}, "UnitSphereCoords", true);
  for(size_t i = 0; i < xyz.size(); i++)
  {
    coords->setValue(i, xyz[i]);
  }
  return coords;
}
} // namespace

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionTest::InitializeSquares_NonPositiveDimension", "[EbsdLib][ModifiedLambertProjectionTest]")
{
  // 0 divided the square edge by zero (infinite step size). -1 wrapped to SIZE_MAX when converted to size_t, and
  // SIZE_MAX * SIZE_MAX overflows back to 1, so a 1-bin square was allocated with a negative step size. Larger
  // negative values ask for enormous allocations, so this test does not use them.
  for(const int dim : {0, -1})
  {
    CAPTURE(dim);
    auto mlp = ModifiedLambertProjection::New();
    mlp->initializeSquares(dim, 1.0f);
    CHECK(mlp->getDimension() == 0);
    CHECK(mlp->getStepSize() == 0.0f);
    REQUIRE(mlp->getNorthSquare() != nullptr);
    REQUIRE(mlp->getSouthSquare() != nullptr);
    CHECK(mlp->getNorthSquare()->getNumberOfTuples() == 0);
    CHECK(mlp->getSouthSquare()->getNumberOfTuples() == 0);

    const std::array<float, 2> sqCoord = {0.0f, 0.0f};
    CHECK(mlp->getInterpolatedValue(ModifiedLambertProjection::NorthSquare, sqCoord.data()) == 0.0);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionTest::LambertBallToSquare_ZeroDimension", "[EbsdLib][ModifiedLambertProjectionTest]")
{
  // Binning into a 0-dimension projection divided by a zero step size and wrote outside the empty squares.
  auto coords = CreateUnitSphereCoords();
  auto mlp = ModifiedLambertProjection::LambertBallToSquare(coords.get(), 0, 1.0f);
  REQUIRE(mlp != nullptr);
  CHECK(mlp->getDimension() == 0);
  mlp->normalizeSquaresToMRD();

  auto stereo = mlp->createStereographicProjection(8);
  REQUIRE(stereo != nullptr);
  REQUIRE(stereo->getNumberOfTuples() == 64);
  for(size_t i = 0; i < stereo->getNumberOfTuples(); i++)
  {
    CHECK(stereo->getValue(i) == 0.0);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionTest::CreateProjection_NegativeDimension", "[EbsdLib][ModifiedLambertProjectionTest]")
{
  // A negative image dimension wrapped when converted to size_t (or was squared to a positive size).
  auto mlp = ModifiedLambertProjection::New();
  mlp->initializeSquares(8, 1.0f);
  auto stereo = mlp->createStereographicProjection(-1);
  REQUIRE(stereo != nullptr);
  CHECK(stereo->getNumberOfTuples() == 0);
  CHECK(mlp->createCircularProjection(-1).empty());
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionTest::ComputeStereographicProjection_SmallDimensions", "[EbsdLib][ModifiedLambertProjectionTest]")
{
  auto coords = CreateUnitSphereCoords();
  PoleFigureConfiguration_t config;
  config.eulers = nullptr;
  config.numColors = 32;
  config.minScale = 0.0;
  config.maxScale = 1.0;
  config.sphereRadius = 1.0f;
  config.discreteHeatMap = false;

  SECTION("Discrete, image dimension 0")
  {
    // The intensity image was resized to 0 pixels and every point was still written into it.
    config.discrete = true;
    config.imageDim = 0;
    config.lambertDim = 8;
    auto intensity = ebsdlib::DoubleArrayType::CreateArray(1, "Intensity", true);
    ComputeStereographicProjection(coords.get(), &config, intensity.get())();
    CHECK(intensity->getNumberOfTuples() == 0);
  }
  SECTION("Discrete, image dimension 1")
  {
    // halfDim - 1 is -1, so points on the equator were written one pixel before or after the 1-pixel image.
    config.discrete = true;
    config.imageDim = 1;
    config.lambertDim = 8;
    auto intensity = ebsdlib::DoubleArrayType::CreateArray(1, "Intensity", true);
    ComputeStereographicProjection(coords.get(), &config, intensity.get())();
    REQUIRE(intensity->getNumberOfTuples() == 1);
    // Only the two poles (-z is folded onto +z) land in the single pixel; +x, -x and +y land outside it and are skipped.
    CHECK(intensity->getValue(0) == 2.0);
  }
  SECTION("Lambert dimension 0")
  {
    config.discrete = false;
    config.imageDim = 8;
    config.lambertDim = 0;
    auto intensity = ebsdlib::DoubleArrayType::CreateArray(1, "Intensity", true);
    ComputeStereographicProjection(coords.get(), &config, intensity.get())();
    REQUIRE(intensity->getNumberOfTuples() == 64);
    for(size_t i = 0; i < intensity->getNumberOfTuples(); i++)
    {
      CHECK(intensity->getValue(i) == 0.0);
    }
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ModifiedLambertProjectionTest::ComputeIPFIntensity_ZeroLambertDimension", "[EbsdLib][ModifiedLambertProjectionTest]")
{
  // The bin count was computed from lambertDim instead of from the allocated squares.
  auto coords = CreateUnitSphereCoords();
  CubicOps ops;
  auto intensity = InversePoleFigureUtilities::computeIPFIntensity(ops, coords.get(), 16, 16, 0, true, true);
  REQUIRE(intensity != nullptr);
  REQUIRE(intensity->getNumberOfTuples() == 256);
  for(size_t i = 0; i < intensity->getNumberOfTuples(); i++)
  {
    // Pixels outside the standard stereographic triangle are -1; the rest have no intensity.
    CHECK((intensity->getValue(i) == 0.0 || intensity->getValue(i) == -1.0));
  }
}
