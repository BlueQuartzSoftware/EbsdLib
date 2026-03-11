#include <catch2/catch.hpp>

#include "EbsdLib/LaueOps/CubicLowOps.h"
#include "EbsdLib/LaueOps/CubicOps.h"
#include "EbsdLib/LaueOps/HexagonalLowOps.h"
#include "EbsdLib/LaueOps/HexagonalOps.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/LaueOps/MonoclinicOps.h"
#include "EbsdLib/LaueOps/OrthoRhombicOps.h"
#include "EbsdLib/LaueOps/TetragonalLowOps.h"
#include "EbsdLib/LaueOps/TetragonalOps.h"
#include "EbsdLib/LaueOps/TriclinicOps.h"
#include "EbsdLib/LaueOps/TrigonalLowOps.h"
#include "EbsdLib/LaueOps/TrigonalOps.h"
#include "EbsdLib/Orientation/OrientationMatrix.hpp"
#include "EbsdLib/Orientation/Rodrigues.hpp"
#include "EbsdLib/Utilities/ColorTable.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace ebsdlib;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::LaueOpsTest::GetAllOrientationOps", "[EbsdLib][LaueOpsTest]")
{
  auto ops = LaueOps::GetAllOrientationOps();
  // Should return exactly 12 entries (one for each Laue group index 0-11)
  REQUIRE(ops.size() == 12);

  for(size_t i = 0; i < ops.size(); i++)
  {
    REQUIRE(ops[i] != nullptr);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::LaueOpsTest::GetNumSymOps", "[EbsdLib][LaueOpsTest]")
{
  auto ops = LaueOps::GetAllOrientationOps();

  // Index order matches CrystalStructure constants:
  // 0=Hexagonal_High, 1=Cubic_High, 2=Hexagonal_Low, 3=Cubic_Low
  // 4=Triclinic, 5=Monoclinic, 6=OrthoRhombic
  // 7=Tetragonal_Low, 8=Tetragonal_High
  // 9=Trigonal_Low, 10=Trigonal_High
  CHECK(ops[0]->getNumSymOps() == 12); // Hexagonal_High
  CHECK(ops[1]->getNumSymOps() == 24); // Cubic_High
  CHECK(ops[2]->getNumSymOps() == 6);  // Hexagonal_Low
  CHECK(ops[3]->getNumSymOps() == 12); // Cubic_Low
  CHECK(ops[4]->getNumSymOps() == 1);  // Triclinic
  CHECK(ops[5]->getNumSymOps() == 2);  // Monoclinic
  CHECK(ops[6]->getNumSymOps() == 4);  // OrthoRhombic
  CHECK(ops[7]->getNumSymOps() == 4);  // Tetragonal_Low
  CHECK(ops[8]->getNumSymOps() == 8);  // Tetragonal_High
  CHECK(ops[9]->getNumSymOps() == 3);  // Trigonal_Low
  CHECK(ops[10]->getNumSymOps() == 6); // Trigonal_High
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::LaueOpsTest::GetSymmetryName", "[EbsdLib][LaueOpsTest]")
{
  auto ops = LaueOps::GetAllOrientationOps();

  for(size_t i = 0; i < ops.size(); i++)
  {
    std::string name = ops[i]->getSymmetryName();
    CHECK(!name.empty());
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::LaueOpsTest::GetLaueNames", "[EbsdLib][LaueOpsTest]")
{
  auto names = LaueOps::GetLaueNames();
  REQUIRE(names.size() == 12);

  for(const auto& name : names)
  {
    CHECK(!name.empty());
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::LaueOpsTest::GetHasInversion", "[EbsdLib][LaueOpsTest]")
{
  auto ops = LaueOps::GetAllOrientationOps();

  for(size_t i = 0; i < ops.size(); i++)
  {
    // All Laue classes are centrosymmetric
    CHECK(ops[i]->getHasInversion() == true);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::LaueOpsTest::CalculateMisorientation_Identity", "[EbsdLib][LaueOpsTest]")
{
  // For CubicOps, misorientation between identical quaternions should be ~0
  CubicOps cubicOps;
  QuatD identity = QuatD::identity();

  auto misori = cubicOps.calculateMisorientation(identity, identity);
  // Angle (misori[3]) should be approximately 0
  REQUIRE(misori[3] == Approx(0.0).margin(1.0e-6));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::LaueOpsTest::GenerateIPFColor_CubicOps", "[EbsdLib][LaueOpsTest]")
{
  CubicOps cubicOps;
  // Identity Euler angles (0,0,0) with Z reference direction
  double eulers[3] = {0.0, 0.0, 0.0};
  double refDir[3] = {0.0, 0.0, 1.0};

  Rgb color = cubicOps.generateIPFColor(eulers, refDir, false);

  // Should produce a valid non-zero color
  int r = RgbColor::dRed(color);
  int g = RgbColor::dGreen(color);
  int b = RgbColor::dBlue(color);
  CHECK((r + g + b) > 0);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::LaueOpsTest::GenerateIPFColor_HexagonalOps", "[EbsdLib][LaueOpsTest]")
{
  HexagonalOps hexOps;
  double eulers[3] = {0.0, 0.0, 0.0};
  double refDir[3] = {0.0, 0.0, 1.0};

  Rgb color = hexOps.generateIPFColor(eulers, refDir, false);

  int r = RgbColor::dRed(color);
  int g = RgbColor::dGreen(color);
  int b = RgbColor::dBlue(color);
  CHECK((r + g + b) > 0);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::LaueOpsTest::FZTypeToString", "[EbsdLib][LaueOpsTest]")
{
  CHECK(!LaueOps::FZTypeToString(LaueOps::FZType::Anorthic).empty());
  CHECK(!LaueOps::FZTypeToString(LaueOps::FZType::Cyclic).empty());
  CHECK(!LaueOps::FZTypeToString(LaueOps::FZType::Dihedral).empty());
  CHECK(!LaueOps::FZTypeToString(LaueOps::FZType::Tetrahedral).empty());
  CHECK(!LaueOps::FZTypeToString(LaueOps::FZType::Octahedral).empty());
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::LaueOpsTest::AxisOrderingTypeToString", "[EbsdLib][LaueOpsTest]")
{
  CHECK(!LaueOps::AxisOrderingTypeToString(LaueOps::AxisOrderingType::None).empty());
  CHECK(!LaueOps::AxisOrderingTypeToString(LaueOps::AxisOrderingType::TwoFold).empty());
  CHECK(!LaueOps::AxisOrderingTypeToString(LaueOps::AxisOrderingType::ThreeFold).empty());
  CHECK(!LaueOps::AxisOrderingTypeToString(LaueOps::AxisOrderingType::FourFold).empty());
  CHECK(!LaueOps::AxisOrderingTypeToString(LaueOps::AxisOrderingType::SixFold).empty());
  CHECK(!LaueOps::AxisOrderingTypeToString(LaueOps::AxisOrderingType::EightFold).empty());
  CHECK(!LaueOps::AxisOrderingTypeToString(LaueOps::AxisOrderingType::TenFold).empty());
  CHECK(!LaueOps::AxisOrderingTypeToString(LaueOps::AxisOrderingType::TwelveFold).empty());
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::LaueOpsTest::GetOrientationOpsFromSpaceGroupNumber", "[EbsdLib][LaueOpsTest]")
{
  // Cubic high: space groups 221-230
  auto cubicHighOps = LaueOps::GetOrientationOpsFromSpaceGroupNumber(225);
  REQUIRE(cubicHighOps != nullptr);
  CHECK(cubicHighOps->getNumSymOps() == 24);

  // Hexagonal high: space groups 191-194
  auto hexHighOps = LaueOps::GetOrientationOpsFromSpaceGroupNumber(194);
  REQUIRE(hexHighOps != nullptr);
  CHECK(hexHighOps->getNumSymOps() == 12);
}

// -----------------------------------------------------------------------------
// Helper: Convert a quaternion (XYZW) to a 3x3 rotation matrix directly,
// independent of the library's toOrientationMatrix() which may depend on
// epsijkd convention. This uses the standard formula for passive rotation.
// Returns a row-major 3x3 matrix as Matrix3X3D.
namespace
{
Matrix3X3D quatToMatrix(const QuatD& q)
{
  double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
  double qq = qw * qw - (qx * qx + qy * qy + qz * qz);
  return Matrix3X3D(qq + 2.0 * qx * qx, 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy), 2.0 * (qy * qx + qw * qz), qq + 2.0 * qy * qy, 2.0 * (qy * qz - qw * qx), 2.0 * (qz * qx - qw * qy),
                    2.0 * (qz * qy + qw * qx), qq + 2.0 * qz * qz);
}

// Helper: Convert a quaternion to Rodrigues vector {nx, ny, nz, tan(angle/2)}
// Matches the library convention: first ensure w >= 0 (negate if needed),
// then if w < 1e-8, tan is infinite (180-degree rotation).
RodriguesDType quatToRodrigues(const QuatD& qIn)
{
  // Ensure w >= 0 (q and -q represent the same rotation)
  double qw = qIn.w(), qx = qIn.x(), qy = qIn.y(), qz = qIn.z();
  if(qw < 0.0)
  {
    qw = -qw;
    qx = -qx;
    qy = -qy;
    qz = -qz;
  }
  constexpr double thr = 1.0e-8;
  if(qw < thr)
  {
    // 180-degree rotation: axis = (x,y,z), tan(angle/2) = infinity
    return RodriguesDType{qx, qy, qz, std::numeric_limits<double>::infinity()};
  }
  double s = std::sqrt(qx * qx + qy * qy + qz * qz);
  if(s < thr)
  {
    // Identity: axis is arbitrary, angle = 0
    return RodriguesDType{0.0, 0.0, 1.0, 0.0};
  }
  double t = s / qw; // tan(angle/2)
  return RodriguesDType{qx / s, qy / s, qz / s, t};
}

// Helper: Compare two quaternions as rotations (q and -q are equivalent).
// Returns true if they represent the same rotation within tolerance.
bool quatsSameRotation(const QuatD& a, const QuatD& b, double tol)
{
  // Try both q and -q
  double diff1 = std::abs(a.x() - b.x()) + std::abs(a.y() - b.y()) + std::abs(a.z() - b.z()) + std::abs(a.w() - b.w());
  double diff2 = std::abs(a.x() + b.x()) + std::abs(a.y() + b.y()) + std::abs(a.z() + b.z()) + std::abs(a.w() + b.w());
  return std::min(diff1, diff2) < tol;
}
} // namespace

// -----------------------------------------------------------------------------
// Validates that all three symmetry operator representations (quaternion,
// Rodrigues, rotation matrix) are mutually consistent at each index for
// every LaueOps subclass. This test will catch reordering mistakes where
// one array is shuffled but another is not.
TEST_CASE("ebsdlib::LaueOpsTest::SymOpRepresentationConsistency", "[EbsdLib][LaueOpsTest]")
{
  auto allOps = LaueOps::GetAllOrientationOps();
  constexpr double matTol = 1.0e-10;
  constexpr double rodTol = 1.0e-6;

  for(size_t laueIdx = 0; laueIdx < allOps.size(); laueIdx++)
  {
    auto& ops = allOps[laueIdx];
    size_t numSym = ops->getNumSymOps();
    std::string className = ops->getNameOfClass();

    SECTION(className)
    {
      for(size_t i = 0; i < numSym; i++)
      {
        QuatD quat = ops->getQuatSymOp(i);
        Matrix3X3D storedMat = ops->getMatSymOpD(i);
        RodriguesDType storedRod = ops->getRodSymOp(i);

        // --- Check 1: Quaternion is unit ---
        double qNorm = std::sqrt(quat.x() * quat.x() + quat.y() * quat.y() + quat.z() * quat.z() + quat.w() * quat.w());
        INFO(className << " op[" << i << "] quaternion norm = " << qNorm);
        CHECK(qNorm == Approx(1.0).margin(1.0e-14));

        // --- Check 2: Rotation matrix is orthogonal (R * R^T = I) ---
        Matrix3X3D rt = storedMat.transpose();
        Matrix3X3D rrt = storedMat * rt;
        for(int r = 0; r < 3; r++)
        {
          for(int c = 0; c < 3; c++)
          {
            double expected = (r == c) ? 1.0 : 0.0;
            INFO(className << " op[" << i << "] R*R^T(" << r << "," << c << ")");
            CHECK(rrt(r, c) == Approx(expected).margin(matTol));
          }
        }

        // --- Check 3: Rotation matrix has det = +1 (proper rotation) ---
        double det = storedMat.determinant();
        INFO(className << " op[" << i << "] det(R) = " << det);
        CHECK(det == Approx(1.0).margin(1.0e-6));

        // --- Check 4: Quaternion -> Matrix matches stored matrix ---
        Matrix3X3D computedMat = quatToMatrix(quat);
        for(int r = 0; r < 3; r++)
        {
          for(int c = 0; c < 3; c++)
          {
            INFO(className << " op[" << i << "] mat(" << r << "," << c << "): computed=" << computedMat(r, c) << " stored=" << storedMat(r, c));
            CHECK(computedMat(r, c) == Approx(storedMat(r, c)).margin(matTol));
          }
        }

        // --- Check 5: Rodrigues -> Matrix matches stored matrix ---
        // For finite Rodrigues, convert to quaternion and compare the resulting
        // rotation matrix against the stored matrix. This catches reordering
        // errors where k_RodSym[i] doesn't match k_QuatSym[i].
        // For infinite Rodrigues (stored as 1e13), the library's toRodrigues()
        // convention treats any quaternion with w < 1e-8 (including negative w)
        // as 180-degree, which is incorrect for rotations > 180°. We skip the
        // rod-to-matrix check in that case since Check 4 already validates the
        // quaternion-to-matrix consistency.
        if(!std::isinf(storedRod[3]) && storedRod[3] < 1.0e10)
        {
          QuatD rodQuat = storedRod.toQuaternion();
          Matrix3X3D rodMat = quatToMatrix(rodQuat);
          for(int r = 0; r < 3; r++)
          {
            for(int c = 0; c < 3; c++)
            {
              INFO(className << " op[" << i << "] rodMat(" << r << "," << c << "): rod=" << rodMat(r, c) << " stored=" << storedMat(r, c));
              CHECK(rodMat(r, c) == Approx(storedMat(r, c)).margin(matTol));
            }
          }
        }
        else
        {
          // Infinite Rodrigues: just verify the axis direction is non-degenerate
          double axisLen = std::sqrt(storedRod[0] * storedRod[0] + storedRod[1] * storedRod[1] + storedRod[2] * storedRod[2]);
          INFO(className << " op[" << i << "] infinite rod axis length = " << axisLen);
          CHECK(axisLen > 1.0e-10);
        }
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Validates group closure: for each LaueOps class, the product of any two
// symmetry operators (as quaternions) must also be in the group.
TEST_CASE("ebsdlib::LaueOpsTest::SymOpGroupClosure", "[EbsdLib][LaueOpsTest]")
{
  auto allOps = LaueOps::GetAllOrientationOps();
  constexpr double tol = 1.0e-10;

  for(size_t laueIdx = 0; laueIdx < allOps.size(); laueIdx++)
  {
    auto& ops = allOps[laueIdx];
    size_t numSym = ops->getNumSymOps();
    std::string className = ops->getNameOfClass();

    // Collect all quaternion operators
    std::vector<QuatD> symOps(numSym);
    for(size_t i = 0; i < numSym; i++)
    {
      symOps[i] = ops->getQuatSymOp(i);
    }

    SECTION(className)
    {
      for(size_t i = 0; i < numSym; i++)
      {
        for(size_t j = 0; j < numSym; j++)
        {
          QuatD product = symOps[i] * symOps[j];

          // Find a match in the group (q and -q represent the same rotation)
          bool found = false;
          for(size_t k = 0; k < numSym; k++)
          {
            if(quatsSameRotation(product, symOps[k], tol))
            {
              found = true;
              break;
            }
          }
          INFO(className << " op[" << i << "] * op[" << j << "] not found in group");
          CHECK(found);
        }
      }
    }
  }
}
