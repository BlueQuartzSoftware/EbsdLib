#include <catch2/catch.hpp>

#include "EbsdLib/Math/ArrayHelpers.hpp"

#include <cmath>
#include <vector>

using VecF = std::vector<float>;
using Helpers = ArrayHelpers<VecF, float>;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ArrayHelpersTest::Splat", "[EbsdLib][ArrayHelpersTest]")
{
  VecF a(5);
  Helpers::splat(a, 3.14f);
  for(size_t i = 0; i < a.size(); i++)
  {
    REQUIRE(a[i] == Approx(3.14f));
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ArrayHelpersTest::Multiply", "[EbsdLib][ArrayHelpersTest]")
{
  VecF a = {1.0f, 2.0f, 3.0f};
  VecF b = {4.0f, 5.0f, 6.0f};

  auto c = Helpers::multiply(a, b);
  REQUIRE(c.size() == 3);
  REQUIRE(c[0] == Approx(4.0f));
  REQUIRE(c[1] == Approx(10.0f));
  REQUIRE(c[2] == Approx(18.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ArrayHelpersTest::MultiplyWithMax", "[EbsdLib][ArrayHelpersTest]")
{
  VecF a = {1.0f, 2.0f, 3.0f, 4.0f};
  VecF b = {5.0f, 6.0f, 7.0f, 8.0f};

  auto c = Helpers::multiply(a, b, 2);
  REQUIRE(c.size() == 2);
  REQUIRE(c[0] == Approx(5.0f));
  REQUIRE(c[1] == Approx(12.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ArrayHelpersTest::ScalarMultiply", "[EbsdLib][ArrayHelpersTest]")
{
  VecF a = {1.0f, 2.0f, 3.0f};
  Helpers::scalarMultiply(a, 2.0f);
  REQUIRE(a[0] == Approx(2.0f));
  REQUIRE(a[1] == Approx(4.0f));
  REQUIRE(a[2] == Approx(6.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ArrayHelpersTest::ScalarDivide", "[EbsdLib][ArrayHelpersTest]")
{
  VecF a = {10.0f, 20.0f, 30.0f};
  Helpers::scalarDivide(a, 10.0f);
  REQUIRE(a[0] == Approx(1.0f));
  REQUIRE(a[1] == Approx(2.0f));
  REQUIRE(a[2] == Approx(3.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ArrayHelpersTest::Sum", "[EbsdLib][ArrayHelpersTest]")
{
  VecF a = {1.0f, 2.0f, 3.0f, 4.0f};
  REQUIRE(Helpers::sum(a) == Approx(10.0f));

  // Sum with max
  REQUIRE(Helpers::sum(a, 2) == Approx(3.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ArrayHelpersTest::SumOfSquares", "[EbsdLib][ArrayHelpersTest]")
{
  VecF a = {1.0f, 2.0f, 3.0f};
  // 1 + 4 + 9 = 14
  REQUIRE(Helpers::sumofSquares(a) == Approx(14.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ArrayHelpersTest::SqrtSumOfSquares", "[EbsdLib][ArrayHelpersTest]")
{
  VecF a = {3.0f, 4.0f, 0.0f};
  REQUIRE(Helpers::sqrtSumOfSquares(a) == Approx(5.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ArrayHelpersTest::MaxVal", "[EbsdLib][ArrayHelpersTest]")
{
  VecF a = {1.0f, 5.0f, 3.0f, 2.0f};
  REQUIRE(Helpers::maxval(a) == Approx(5.0f));

  VecF b = {-1.0f, -5.0f, -3.0f};
  REQUIRE(Helpers::maxval(b) == Approx(-1.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ArrayHelpersTest::AbsValue", "[EbsdLib][ArrayHelpersTest]")
{
  VecF a = {-1.0f, 2.0f, -3.0f};
  auto result = Helpers::absValue(a);
  REQUIRE(result[0] == Approx(1.0f));
  REQUIRE(result[1] == Approx(2.0f));
  REQUIRE(result[2] == Approx(3.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ArrayHelpersTest::Power", "[EbsdLib][ArrayHelpersTest]")
{
  VecF a = {2.0f, 3.0f, 4.0f};
  Helpers::power(a, 2.0f);
  REQUIRE(a[0] == Approx(4.0f));
  REQUIRE(a[1] == Approx(9.0f));
  REQUIRE(a[2] == Approx(16.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ArrayHelpersTest::EmptyArray", "[EbsdLib][ArrayHelpersTest]")
{
  VecF empty;
  REQUIRE(Helpers::sum(empty) == Approx(0.0f));
  REQUIRE(Helpers::sumofSquares(empty) == Approx(0.0f));
  REQUIRE(Helpers::sqrtSumOfSquares(empty) == Approx(0.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ArrayHelpersTest::DoubleType", "[EbsdLib][ArrayHelpersTest]")
{
  using VecD = std::vector<double>;
  using HelpersD = ArrayHelpers<VecD, double>;

  VecD a = {1.0, 2.0, 3.0};
  REQUIRE(HelpersD::sum(a) == Approx(6.0));
  REQUIRE(HelpersD::sumofSquares(a) == Approx(14.0));
}
