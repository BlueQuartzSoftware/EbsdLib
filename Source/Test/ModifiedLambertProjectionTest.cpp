#include <catch2/catch.hpp>

#include "EbsdLib/Utilities/ModifiedLambertProjection.h"

#include <cmath>

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
