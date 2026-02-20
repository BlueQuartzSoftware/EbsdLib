#include <catch2/catch.hpp>

#include "EbsdLib/Math/Matrix3X1.hpp"
#include "EbsdLib/Math/Matrix3X3.hpp"

#include <array>
#include <cmath>

using namespace ebsdlib;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::DefaultConstruction", "[EbsdLib][Matrix3X3Test]")
{
  Matrix3X3<float> m;
  for(size_t i = 0; i < 9; i++)
  {
    REQUIRE(m[i] == 0.0f);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::ValueConstruction", "[EbsdLib][Matrix3X3Test]")
{
  Matrix3X3<float> m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
  REQUIRE(m[0] == 1.0f);
  REQUIRE(m[1] == 2.0f);
  REQUIRE(m[2] == 3.0f);
  REQUIRE(m[3] == 4.0f);
  REQUIRE(m[4] == 5.0f);
  REQUIRE(m[5] == 6.0f);
  REQUIRE(m[6] == 7.0f);
  REQUIRE(m[7] == 8.0f);
  REQUIRE(m[8] == 9.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::PointerConstruction", "[EbsdLib][Matrix3X3Test]")
{
  float data[9] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f};
  Matrix3X3<float> m(data);
  for(size_t i = 0; i < 9; i++)
  {
    REQUIRE(m[i] == data[i]);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::OperatorParens", "[EbsdLib][Matrix3X3Test]")
{
  Matrix3X3<float> m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
  REQUIRE(m(0, 0) == 1.0f);
  REQUIRE(m(0, 1) == 2.0f);
  REQUIRE(m(0, 2) == 3.0f);
  REQUIRE(m(1, 0) == 4.0f);
  REQUIRE(m(1, 1) == 5.0f);
  REQUIRE(m(1, 2) == 6.0f);
  REQUIRE(m(2, 0) == 7.0f);
  REQUIRE(m(2, 1) == 8.0f);
  REQUIRE(m(2, 2) == 9.0f);

  // Out of range should throw
  REQUIRE_THROWS_AS(m(3, 0), std::out_of_range);
  REQUIRE_THROWS_AS(m(0, 3), std::out_of_range);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::Identity", "[EbsdLib][Matrix3X3Test]")
{
  auto id = Matrix3X3<float>::Identity();
  REQUIRE(id(0, 0) == 1.0f);
  REQUIRE(id(0, 1) == 0.0f);
  REQUIRE(id(0, 2) == 0.0f);
  REQUIRE(id(1, 0) == 0.0f);
  REQUIRE(id(1, 1) == 1.0f);
  REQUIRE(id(1, 2) == 0.0f);
  REQUIRE(id(2, 0) == 0.0f);
  REQUIRE(id(2, 1) == 0.0f);
  REQUIRE(id(2, 2) == 1.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::MatrixMultiply", "[EbsdLib][Matrix3X3Test]")
{
  auto id = Matrix3X3<float>::Identity();
  Matrix3X3<float> a(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);

  // A * I = A
  auto result = a * id;
  for(size_t i = 0; i < 9; i++)
  {
    REQUIRE(result[i] == Approx(a[i]));
  }

  // I * A = A
  result = id * a;
  for(size_t i = 0; i < 9; i++)
  {
    REQUIRE(result[i] == Approx(a[i]));
  }

  // Known multiplication
  Matrix3X3<float> b(9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f);
  result = a * b;
  // Row 0: [1*9+2*6+3*3, 1*8+2*5+3*2, 1*7+2*4+3*1] = [30, 24, 18]
  REQUIRE(result(0, 0) == Approx(30.0f));
  REQUIRE(result(0, 1) == Approx(24.0f));
  REQUIRE(result(0, 2) == Approx(18.0f));
  // Row 1: [4*9+5*6+6*3, 4*8+5*5+6*2, 4*7+5*4+6*1] = [84, 69, 54]
  REQUIRE(result(1, 0) == Approx(84.0f));
  REQUIRE(result(1, 1) == Approx(69.0f));
  REQUIRE(result(1, 2) == Approx(54.0f));
  // Row 2: [7*9+8*6+9*3, 7*8+8*5+9*2, 7*7+8*4+9*1] = [138, 114, 90]
  REQUIRE(result(2, 0) == Approx(138.0f));
  REQUIRE(result(2, 1) == Approx(114.0f));
  REQUIRE(result(2, 2) == Approx(90.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::MultiplyInPlace", "[EbsdLib][Matrix3X3Test]")
{
  Matrix3X3<float> a(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
  auto id = Matrix3X3<float>::Identity();
  a.multiplyInPlace(id);
  REQUIRE(a(0, 0) == Approx(1.0f));
  REQUIRE(a(1, 1) == Approx(5.0f));
  REQUIRE(a(2, 2) == Approx(9.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::MatrixVectorMultiply", "[EbsdLib][Matrix3X3Test]")
{
  auto id = Matrix3X3<float>::Identity();
  Matrix3X1<float> v(1.0f, 2.0f, 3.0f);

  // I * v = v
  auto result = id * v;
  REQUIRE(result[0] == Approx(1.0f));
  REQUIRE(result[1] == Approx(2.0f));
  REQUIRE(result[2] == Approx(3.0f));

  // Known multiplication
  Matrix3X3<float> m(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f);
  result = m * v;
  // [1*1+0*2+0*3, 0*1+0*2+(-1)*3, 0*1+1*2+0*3] = [1, -3, 2]
  REQUIRE(result[0] == Approx(1.0f));
  REQUIRE(result[1] == Approx(-3.0f));
  REQUIRE(result[2] == Approx(2.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::MatrixArrayMultiply", "[EbsdLib][Matrix3X3Test]")
{
  auto id = Matrix3X3<float>::Identity();
  std::array<float, 3> v = {1.0f, 2.0f, 3.0f};

  auto result = id * v;
  REQUIRE(result[0] == Approx(1.0f));
  REQUIRE(result[1] == Approx(2.0f));
  REQUIRE(result[2] == Approx(3.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::ScalarMultiply", "[EbsdLib][Matrix3X3Test]")
{
  Matrix3X3<float> m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
  auto result = m * 2.0f;
  REQUIRE(result[0] == Approx(2.0f));
  REQUIRE(result[4] == Approx(10.0f));
  REQUIRE(result[8] == Approx(18.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::Addition", "[EbsdLib][Matrix3X3Test]")
{
  Matrix3X3<float> a(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
  Matrix3X3<float> b(9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f);

  auto c = a + b;
  REQUIRE(c[0] == Approx(10.0f));
  REQUIRE(c[4] == Approx(10.0f));
  REQUIRE(c[8] == Approx(10.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::Subtraction", "[EbsdLib][Matrix3X3Test]")
{
  Matrix3X3<float> a(5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f);
  Matrix3X3<float> b(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);

  auto c = a - b;
  REQUIRE(c[0] == Approx(4.0f));
  REQUIRE(c[4] == Approx(0.0f));
  REQUIRE(c[8] == Approx(-4.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::Transpose", "[EbsdLib][Matrix3X3Test]")
{
  Matrix3X3<float> m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
  auto t = m.transpose();
  REQUIRE(t(0, 0) == 1.0f);
  REQUIRE(t(0, 1) == 4.0f);
  REQUIRE(t(0, 2) == 7.0f);
  REQUIRE(t(1, 0) == 2.0f);
  REQUIRE(t(1, 1) == 5.0f);
  REQUIRE(t(1, 2) == 8.0f);
  REQUIRE(t(2, 0) == 3.0f);
  REQUIRE(t(2, 1) == 6.0f);
  REQUIRE(t(2, 2) == 9.0f);

  // (A^T)^T = A
  auto tt = t.transpose();
  for(size_t i = 0; i < 9; i++)
  {
    REQUIRE(tt[i] == Approx(m[i]));
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::Determinant", "[EbsdLib][Matrix3X3Test]")
{
  // Identity determinant = 1
  auto id = Matrix3X3<float>::Identity();
  REQUIRE(id.determinant() == Approx(1.0f));

  // Known determinant
  Matrix3X3<float> m(1.0f, 2.0f, 3.0f, 0.0f, 1.0f, 4.0f, 5.0f, 6.0f, 0.0f);
  // det = 1*(1*0-4*6) - 2*(0*0-4*5) + 3*(0*6-1*5) = 1*(-24) - 2*(-20) + 3*(-5) = -24+40-15 = 1
  REQUIRE(m.determinant() == Approx(1.0f));

  // det(A^T) = det(A)
  auto t = m.transpose();
  REQUIRE(t.determinant() == Approx(m.determinant()));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::Invert", "[EbsdLib][Matrix3X3Test]")
{
  Matrix3X3<float> m(1.0f, 2.0f, 3.0f, 0.0f, 1.0f, 4.0f, 5.0f, 6.0f, 0.0f);

  auto inv = m.invert();

  // A * A^-1 should be approximately I
  auto product = m * inv;
  REQUIRE(product(0, 0) == Approx(1.0f).margin(1e-5f));
  REQUIRE(product(0, 1) == Approx(0.0f).margin(1e-5f));
  REQUIRE(product(0, 2) == Approx(0.0f).margin(1e-5f));
  REQUIRE(product(1, 0) == Approx(0.0f).margin(1e-5f));
  REQUIRE(product(1, 1) == Approx(1.0f).margin(1e-5f));
  REQUIRE(product(1, 2) == Approx(0.0f).margin(1e-5f));
  REQUIRE(product(2, 0) == Approx(0.0f).margin(1e-5f));
  REQUIRE(product(2, 1) == Approx(0.0f).margin(1e-5f));
  REQUIRE(product(2, 2) == Approx(1.0f).margin(1e-5f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::Minors", "[EbsdLib][Matrix3X3Test]")
{
  Matrix3X3<float> m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
  auto min = m.minors();

  // Minor(0,0) = det([[5,6],[8,9]]) = 45-48 = -3
  REQUIRE(min[0] == Approx(-3.0f));
  // Minor(0,1) = det([[4,6],[7,9]]) = 36-42 = -6
  REQUIRE(min[1] == Approx(-6.0f));
  // Minor(0,2) = det([[4,5],[7,8]]) = 32-35 = -3
  REQUIRE(min[2] == Approx(-3.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::Cofactor", "[EbsdLib][Matrix3X3Test]")
{
  Matrix3X3<float> m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
  auto cof = m.cofactor();

  // Cofactor applies sign checkerboard to minors
  REQUIRE(cof[0] == Approx(-3.0f)); // +minor[0]
  REQUIRE(cof[1] == Approx(6.0f));  // -minor[1]
  REQUIRE(cof[2] == Approx(-3.0f)); // +minor[2]
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::Adjoint", "[EbsdLib][Matrix3X3Test]")
{
  Matrix3X3<float> m(1.0f, 2.0f, 3.0f, 0.0f, 1.0f, 4.0f, 5.0f, 6.0f, 0.0f);
  auto adj = m.adjoint();

  // adjoint = cofactor transposed
  auto cof = m.cofactor();
  auto cofT = cof.transpose();
  for(size_t i = 0; i < 9; i++)
  {
    REQUIRE(adj[i] == Approx(cofT[i]));
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::ColAndRow", "[EbsdLib][Matrix3X3Test]")
{
  Matrix3X3<float> m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);

  auto col0 = m.col(0);
  REQUIRE(col0[0] == 1.0f);
  REQUIRE(col0[1] == 4.0f);
  REQUIRE(col0[2] == 7.0f);

  auto col2 = m.col(2);
  REQUIRE(col2[0] == 3.0f);
  REQUIRE(col2[1] == 6.0f);
  REQUIRE(col2[2] == 9.0f);

  auto row0 = m.row(0);
  REQUIRE(row0[0] == 1.0f);
  REQUIRE(row0[1] == 2.0f);
  REQUIRE(row0[2] == 3.0f);

  auto row2 = m.row(2);
  REQUIRE(row2[0] == 7.0f);
  REQUIRE(row2[1] == 8.0f);
  REQUIRE(row2[2] == 9.0f);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::Normalize", "[EbsdLib][Matrix3X3Test]")
{
  auto id = Matrix3X3<float>::Identity();
  auto norm = id.normalize();

  // Identity columns already have unit length, so normalize should return identity
  REQUIRE(norm(0, 0) == Approx(1.0f));
  REQUIRE(norm(1, 1) == Approx(1.0f));
  REQUIRE(norm(2, 2) == Approx(1.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::CopyInto", "[EbsdLib][Matrix3X3Test]")
{
  Matrix3X3<float> m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
  double dest[9] = {};
  m.copyInto(dest);
  for(size_t i = 0; i < 9; i++)
  {
    REQUIRE(dest[i] == Approx(static_cast<double>(m[i])));
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::DirectStructureMatrix", "[EbsdLib][Matrix3X3Test]")
{
  // Cubic system: a=b=c=1, alpha=beta=gamma=90
  std::array<float, 6> cubicParams = {1.0f, 1.0f, 1.0f, 90.0f, 90.0f, 90.0f};
  auto dsm = Matrix3X3<float>::DirectStructureMatrix(cubicParams);

  // For cubic: DSM should be approximately identity
  REQUIRE(dsm(0, 0) == Approx(1.0f).margin(1e-5f));
  REQUIRE(dsm(1, 1) == Approx(1.0f).margin(1e-5f));
  REQUIRE(dsm(2, 2) == Approx(1.0f).margin(1e-5f));
  REQUIRE(dsm(0, 1) == Approx(0.0f).margin(1e-5f));
  REQUIRE(dsm(0, 2) == Approx(0.0f).margin(1e-5f));
  REQUIRE(dsm(1, 0) == Approx(0.0f).margin(1e-5f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::Matrix3X3Test::DoubleType", "[EbsdLib][Matrix3X3Test]")
{
  auto id = Matrix3X3D::Identity();
  REQUIRE(id.determinant() == Approx(1.0));

  auto inv = id.invert();
  for(size_t i = 0; i < 9; i++)
  {
    REQUIRE(inv[i] == Approx(id[i]));
  }
}
