#include <catch2/catch.hpp>

#include "EbsdLib/LaueOps/HexagonalOps.h"
#include "EbsdLib/Math/Matrix3X1.hpp"
#include "EbsdLib/Math/Matrix3X3.hpp"
#include "EbsdLib/Orientation/AxisAngle.hpp"
#include "EbsdLib/Orientation/Cubochoric.hpp"
#include "EbsdLib/Orientation/Euler.hpp"
#include "EbsdLib/Orientation/Homochoric.hpp"
#include "EbsdLib/Orientation/OrientationMatrix.hpp"
#include "EbsdLib/Orientation/Quaternion.hpp"
#include "EbsdLib/Orientation/Rodrigues.hpp"

#include <cmath>

using namespace ebsdlib;

// =============================================================================
// Euler <-> OrientationMatrix
// =============================================================================

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationTransformationTest::eu2om_Identity", "[EbsdLib][OrientationTransformationTest]")
{
  // Identity Euler (0,0,0) should produce identity 3x3 matrix
  EulerDType eu(0.0, 0.0, 0.0);
  auto om = eu.toOrientationMatrix();

  // Diagonal should be 1
  CHECK(om[0] == Approx(1.0).margin(1.0e-10));
  CHECK(om[4] == Approx(1.0).margin(1.0e-10));
  CHECK(om[8] == Approx(1.0).margin(1.0e-10));

  // Off-diagonal should be 0
  CHECK(om[1] == Approx(0.0).margin(1.0e-10));
  CHECK(om[2] == Approx(0.0).margin(1.0e-10));
  CHECK(om[3] == Approx(0.0).margin(1.0e-10));
  CHECK(om[5] == Approx(0.0).margin(1.0e-10));
  CHECK(om[6] == Approx(0.0).margin(1.0e-10));
  CHECK(om[7] == Approx(0.0).margin(1.0e-10));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationTransformationTest::om2eu_Identity", "[EbsdLib][OrientationTransformationTest]")
{
  // Identity matrix should produce Euler (0,0,0)
  OrientationMatrixDType om(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
  auto eu = om.toEuler();

  CHECK(eu[0] == Approx(0.0).margin(1.0e-10));
  CHECK(eu[1] == Approx(0.0).margin(1.0e-10));
  CHECK(eu[2] == Approx(0.0).margin(1.0e-10));
}

// =============================================================================
// Euler <-> Quaternion
// =============================================================================

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationTransformationTest::eu2qu_Identity", "[EbsdLib][OrientationTransformationTest]")
{
  // Identity Euler -> identity quaternion (0,0,0,1) in VectorScalar layout
  EulerDType eu(0.0, 0.0, 0.0);
  auto qu = eu.toQuaternion();

  // VectorScalar: [x, y, z, w] where identity is (0,0,0,1)
  CHECK(qu[0] == Approx(0.0).margin(1.0e-10));
  CHECK(qu[1] == Approx(0.0).margin(1.0e-10));
  CHECK(qu[2] == Approx(0.0).margin(1.0e-10));
  CHECK(qu[3] == Approx(1.0).margin(1.0e-10));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationTransformationTest::qu2eu_Identity", "[EbsdLib][OrientationTransformationTest]")
{
  // Identity quaternion -> Euler (0,0,0)
  QuatD qu = QuatD::identity();
  auto eu = qu.toEuler();

  CHECK(eu[0] == Approx(0.0).margin(1.0e-10));
  CHECK(eu[1] == Approx(0.0).margin(1.0e-10));
  CHECK(eu[2] == Approx(0.0).margin(1.0e-10));
}

// =============================================================================
// Euler -> AxisAngle
// =============================================================================

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationTransformationTest::eu2ax_Identity", "[EbsdLib][OrientationTransformationTest]")
{
  // Identity Euler -> axis-angle with angle = 0
  EulerDType eu(0.0, 0.0, 0.0);
  auto ax = eu.toAxisAngle();

  // The angle (4th component) should be 0
  CHECK(ax[3] == Approx(0.0).margin(1.0e-10));
}

// =============================================================================
// AxisAngle -> OrientationMatrix
// =============================================================================

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationTransformationTest::ax2om_90degZ", "[EbsdLib][OrientationTransformationTest]")
{
  // 90-degree rotation about Z-axis
  // axis = (0,0,1), angle = pi/2
  AxisAngleDType ax(0.0, 0.0, 1.0, M_PI / 2.0);
  auto om = ax.toOrientationMatrix();

  // Expected rotation matrix for passive 90-deg rotation about Z:
  // |  0 -1  0 |
  // |  1  0  0 |
  // |  0  0  1 |
  CHECK(om[0] == Approx(0.0).margin(1.0e-10));
  CHECK(om[1] == Approx(-1.0).margin(1.0e-10));
  CHECK(om[2] == Approx(0.0).margin(1.0e-10));
  CHECK(om[3] == Approx(1.0).margin(1.0e-10));
  CHECK(om[4] == Approx(0.0).margin(1.0e-10));
  CHECK(om[5] == Approx(0.0).margin(1.0e-10));
  CHECK(om[6] == Approx(0.0).margin(1.0e-10));
  CHECK(om[7] == Approx(0.0).margin(1.0e-10));
  CHECK(om[8] == Approx(1.0).margin(1.0e-10));
}

// =============================================================================
// Rodrigues -> Homochoric
// =============================================================================

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationTransformationTest::ro2ho_Zero", "[EbsdLib][OrientationTransformationTest]")
{
  // Zero Rodrigues vector (identity rotation) -> zero homochoric
  RodriguesDType ro(0.0, 0.0, 1.0, 0.0); // axis=(0,0,1), length=0
  auto ho = ro.toHomochoric();

  CHECK(ho[0] == Approx(0.0).margin(1.0e-10));
  CHECK(ho[1] == Approx(0.0).margin(1.0e-10));
  CHECK(ho[2] == Approx(0.0).margin(1.0e-10));
}

// =============================================================================
// Homochoric -> Cubochoric
// =============================================================================

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationTransformationTest::ho2cu_Zero", "[EbsdLib][OrientationTransformationTest]")
{
  // Zero homochoric -> zero cubochoric
  HomochoricDType ho(0.0, 0.0, 0.0);
  auto cu = ho.toCubochoric();

  CHECK(cu[0] == Approx(0.0).margin(1.0e-10));
  CHECK(cu[1] == Approx(0.0).margin(1.0e-10));
  CHECK(cu[2] == Approx(0.0).margin(1.0e-10));
}

// =============================================================================
// Round-trip tests with known analytical values
// =============================================================================

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationTransformationTest::Euler_OM_RoundTrip", "[EbsdLib][OrientationTransformationTest]")
{
  // Non-trivial Euler angles
  EulerDType eu(0.5, 0.7, 1.2);
  auto om = eu.toOrientationMatrix();
  auto euBack = om.toEuler();

  CHECK(euBack[0] == Approx(eu[0]).margin(1.0e-8));
  CHECK(euBack[1] == Approx(eu[1]).margin(1.0e-8));
  CHECK(euBack[2] == Approx(eu[2]).margin(1.0e-8));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationTransformationTest::Euler_Quat_RoundTrip", "[EbsdLib][OrientationTransformationTest]")
{
  EulerDType eu(1.0, 0.5, 2.0);
  auto qu = eu.toQuaternion();
  auto euBack = qu.toEuler();

  CHECK(euBack[0] == Approx(eu[0]).margin(1.0e-8));
  CHECK(euBack[1] == Approx(eu[1]).margin(1.0e-8));
  CHECK(euBack[2] == Approx(eu[2]).margin(1.0e-8));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationTransformationTest::Euler_AxisAngle_RoundTrip", "[EbsdLib][OrientationTransformationTest]")
{
  EulerDType eu(0.3, 1.2, 0.8);
  auto ax = eu.toAxisAngle();
  auto euBack = ax.toEuler();

  CHECK(euBack[0] == Approx(eu[0]).margin(1.0e-6));
  CHECK(euBack[1] == Approx(eu[1]).margin(1.0e-6));
  CHECK(euBack[2] == Approx(eu[2]).margin(1.0e-6));
}

// -----------------------------------------------------------------------------
// Regression test for 180-degree rotation handling in LaueOps::_calcRodNearestOrigin
// (invoked via getODFFZRod). Euler (180°, 90°, 0°) is a 180° rotation about
// (0, 1, 1)/√2. Because tan(90°) = ∞, naive Rodrigues-space symmetry reduction
// produces NaN/∞ and returns a degenerate FZ representative. The FZ rep must
// represent the same physical orientation as the input (modulo crystal symmetry),
// so the crystal c-axis [0001] expressed in the sample frame must match the
// input's c-axis direction up to sign (hexagonal 6/mmm is centrosymmetric).
TEST_CASE("ebsdlib::OrientationTransformationTest::GetODFFZRod_180DegRotation_Hexagonal", "[EbsdLib][OrientationTransformationTest]")
{
  HexagonalOps hexOps;

  EulerDType euIn(ebsdlib::constants::k_PiD, ebsdlib::constants::k_PiOver2D, 0.0);
  Matrix3X1D cCrystal{0.0, 0.0, 1.0};
  // Crystal c-axis in sample frame = g^T * c_crystal
  Matrix3X1D cAxisIn = euIn.toOrientationMatrix().toGMatrix().transpose() * cCrystal;

  RodriguesDType rodIn = euIn.toRodrigues();
  RodriguesDType rodFZ = hexOps.getODFFZRod(rodIn);

  // Output must not be degenerate (non-finite axis or NaN)
  REQUIRE(std::isfinite(rodFZ[0]));
  REQUIRE(std::isfinite(rodFZ[1]));
  REQUIRE(std::isfinite(rodFZ[2]));

  Matrix3X1D cAxisOut = rodFZ.toOrientationMatrix().toGMatrix().transpose() * cCrystal;

  double dot = cAxisIn[0] * cAxisOut[0] + cAxisIn[1] * cAxisOut[1] + cAxisIn[2] * cAxisOut[2];
  // Parallel or antiparallel: |dot| ≈ 1
  CHECK(std::fabs(dot) == Approx(1.0).margin(1.0e-6));
}
