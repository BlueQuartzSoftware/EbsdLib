#include <catch2/catch.hpp>

#include "EbsdLib/Math/Matrix3X1.hpp"

#include <cmath>
#include <limits>

using namespace ebsdlib;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::DefaultConstruction", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1<float> m;
  REQUIRE(m[0] == 0.0f);
  REQUIRE(m[1] == 0.0f);
  REQUIRE(m[2] == 0.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::ValueConstruction", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1<float> m(1.0f, 2.0f, 3.0f);
  REQUIRE(m[0] == 1.0f);
  REQUIRE(m[1] == 2.0f);
  REQUIRE(m[2] == 3.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::PointerConstruction", "[EbsdLib][Matrix3X1Test]")
{
  float data[3] = {4.0f, 5.0f, 6.0f};
  Matrix3X1<float> m(data);
  REQUIRE(m[0] == 4.0f);
  REQUIRE(m[1] == 5.0f);
  REQUIRE(m[2] == 6.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::CopyAndMove", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1<float> a(1.0f, 2.0f, 3.0f);
  Matrix3X1<float> b(a);
  REQUIRE(b[0] == 1.0f);
  REQUIRE(b[1] == 2.0f);
  REQUIRE(b[2] == 3.0f);

  Matrix3X1<float> c = std::move(b);
  REQUIRE(c[0] == 1.0f);
  REQUIRE(c[1] == 2.0f);
  REQUIRE(c[2] == 3.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::DotProduct", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1<float> a(1.0f, 2.0f, 3.0f);
  Matrix3X1<float> b(4.0f, 5.0f, 6.0f);

  // a dot b = 1*4 + 2*5 + 3*6 = 32
  REQUIRE(a.dot(b) == Approx(32.0f));

  // Self dot product = sum of squares
  REQUIRE(a.dot() == Approx(14.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::CrossProduct", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1<float> a(1.0f, 0.0f, 0.0f);
  Matrix3X1<float> b(0.0f, 1.0f, 0.0f);

  auto c = a.cross(b);
  REQUIRE(c[0] == Approx(0.0f));
  REQUIRE(c[1] == Approx(0.0f));
  REQUIRE(c[2] == Approx(1.0f));

  // Cross product is anti-commutative: b x a = -(a x b)
  auto d = b.cross(a);
  REQUIRE(d[0] == Approx(0.0f));
  REQUIRE(d[1] == Approx(0.0f));
  REQUIRE(d[2] == Approx(-1.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::Magnitude", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1<float> m(3.0f, 4.0f, 0.0f);
  REQUIRE(m.magnitude() == Approx(5.0f));

  Matrix3X1<float> zero;
  REQUIRE(zero.magnitude() == Approx(0.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::NormalizeMember", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1<float> m(3.0f, 4.0f, 0.0f);
  auto n = m.normalize();
  REQUIRE(n[0] == Approx(0.6f));
  REQUIRE(n[1] == Approx(0.8f));
  REQUIRE(n[2] == Approx(0.0f));

  // Magnitude of normalized vector should be 1
  REQUIRE(n.magnitude() == Approx(1.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::NormalizeStatic", "[EbsdLib][Matrix3X1Test]")
{
  float i = 3.0f;
  float j = 4.0f;
  float k = 0.0f;
  bool result = Matrix3X1<float>::normalize(i, j, k);
  REQUIRE(result == true);
  REQUIRE(i == Approx(0.6f));
  REQUIRE(j == Approx(0.8f));
  REQUIRE(k == Approx(0.0f));

  // Zero vector should return false
  float zi = 0.0f;
  float zj = 0.0f;
  float zk = 0.0f;
  result = Matrix3X1<float>::normalize(zi, zj, zk);
  REQUIRE(result == false);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::SortAscending", "[EbsdLib][Matrix3X1Test]")
{
  // Already sorted
  {
    Matrix3X1<float> m(1.0f, 2.0f, 3.0f);
    auto sorted = m.sortAscending();
    REQUIRE(sorted[0] == 1.0f);
    REQUIRE(sorted[1] == 2.0f);
    REQUIRE(sorted[2] == 3.0f);
  }

  // Reverse sorted
  {
    Matrix3X1<float> m(3.0f, 2.0f, 1.0f);
    auto sorted = m.sortAscending();
    REQUIRE(sorted[0] == 1.0f);
    REQUIRE(sorted[1] == 2.0f);
    REQUIRE(sorted[2] == 3.0f);
  }

  // Middle element is smallest
  {
    Matrix3X1<float> m(2.0f, 1.0f, 3.0f);
    auto sorted = m.sortAscending();
    REQUIRE(sorted[0] == 1.0f);
    REQUIRE(sorted[1] == 2.0f);
    REQUIRE(sorted[2] == 3.0f);
  }

  // Last element is smallest
  {
    Matrix3X1<float> m(3.0f, 1.0f, 0.0f);
    auto sorted = m.sortAscending();
    REQUIRE(sorted[0] == 0.0f);
    REQUIRE(sorted[1] == 1.0f);
    REQUIRE(sorted[2] == 3.0f);
  }

  // All equal
  {
    Matrix3X1<float> m(5.0f, 5.0f, 5.0f);
    auto sorted = m.sortAscending();
    REQUIRE(sorted[0] == 5.0f);
    REQUIRE(sorted[1] == 5.0f);
    REQUIRE(sorted[2] == 5.0f);
  }

  // Negative values
  {
    Matrix3X1<float> m(1.0f, -3.0f, 0.0f);
    auto sorted = m.sortAscending();
    REQUIRE(sorted[0] == -3.0f);
    REQUIRE(sorted[1] == 0.0f);
    REQUIRE(sorted[2] == 1.0f);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::CosTheta", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1<double> a(1.0, 0.0, 0.0);
  Matrix3X1<double> b(0.0, 1.0, 0.0);

  // Perpendicular vectors -> cosTheta = 0
  REQUIRE(a.cosTheta(b) == Approx(0.0));

  // Same direction -> cosTheta = 1
  REQUIRE(a.cosTheta(a) == Approx(1.0));

  // Opposite direction -> cosTheta = -1
  Matrix3X1<double> c(-1.0, 0.0, 0.0);
  REQUIRE(a.cosTheta(c) == Approx(-1.0));

  // Zero vector -> cosTheta = 1 (by convention)
  Matrix3X1<double> zero;
  REQUIRE(a.cosTheta(zero) == Approx(1.0));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::MaxValueIndex", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1<float> a(5.0f, 3.0f, 1.0f);
  REQUIRE(a.maxValueIndex() == 0);

  Matrix3X1<float> b(1.0f, 5.0f, 3.0f);
  REQUIRE(b.maxValueIndex() == 1);

  Matrix3X1<float> c(1.0f, 3.0f, 5.0f);
  REQUIRE(c.maxValueIndex() == 2);

  // Uses absolute value
  Matrix3X1<float> d(-10.0f, 3.0f, 5.0f);
  REQUIRE(d.maxValueIndex() == 0);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::Abs", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1<float> m(-1.0f, 2.0f, -3.0f);
  auto a = m.abs();
  REQUIRE(a[0] == 1.0f);
  REQUIRE(a[1] == 2.0f);
  REQUIRE(a[2] == 3.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::OperatorAdd", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1<float> a(1.0f, 2.0f, 3.0f);
  Matrix3X1<float> b(4.0f, 5.0f, 6.0f);

  auto c = a + b;
  REQUIRE(c[0] == 5.0f);
  REQUIRE(c[1] == 7.0f);
  REQUIRE(c[2] == 9.0f);

  // Scalar addition
  auto d = a + 10.0f;
  REQUIRE(d[0] == 11.0f);
  REQUIRE(d[1] == 12.0f);
  REQUIRE(d[2] == 13.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::OperatorSubtract", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1<float> a(4.0f, 5.0f, 6.0f);
  Matrix3X1<float> b(1.0f, 2.0f, 3.0f);

  auto c = a - b;
  REQUIRE(c[0] == 3.0f);
  REQUIRE(c[1] == 3.0f);
  REQUIRE(c[2] == 3.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::OperatorScalarMultiply", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1<float> a(1.0f, 2.0f, 3.0f);
  auto b = a * 2.0f;
  REQUIRE(b[0] == 2.0f);
  REQUIRE(b[1] == 4.0f);
  REQUIRE(b[2] == 6.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::CopyInto", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1<float> m(1.0f, 2.0f, 3.0f);
  double dest[3] = {0.0, 0.0, 0.0};
  m.copyInto(dest);
  REQUIRE(dest[0] == Approx(1.0));
  REQUIRE(dest[1] == Approx(2.0));
  REQUIRE(dest[2] == Approx(3.0));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::ToType", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1<float> m(1.5f, 2.5f, 3.5f);
  Matrix3X1<double> d = m.to<double>();
  REQUIRE(d[0] == Approx(1.5));
  REQUIRE(d[1] == Approx(2.5));
  REQUIRE(d[2] == Approx(3.5));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::DataPointer", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1<float> m(1.0f, 2.0f, 3.0f);
  float* ptr = m.data();
  REQUIRE(ptr != nullptr);
  REQUIRE(ptr[0] == 1.0f);
  REQUIRE(ptr[1] == 2.0f);
  REQUIRE(ptr[2] == 3.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X1Test::DoubleType", "[EbsdLib][Matrix3X1Test]")
{
  Matrix3X1D m(1.0, 2.0, 3.0);
  REQUIRE(m[0] == 1.0);
  REQUIRE(m.magnitude() == Approx(std::sqrt(14.0)));
  auto n = m.normalize();
  REQUIRE(n.magnitude() == Approx(1.0));
}
