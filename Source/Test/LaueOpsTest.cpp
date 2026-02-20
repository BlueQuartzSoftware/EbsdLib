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
#include "EbsdLib/Utilities/ColorTable.h"

#include <cmath>
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
