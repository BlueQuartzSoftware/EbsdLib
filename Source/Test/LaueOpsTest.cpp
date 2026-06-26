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
// getDefaultPoleFigureNames returns the plotted plane-normal families in brace
// (plane-family) notation. The family identity does NOT depend on HexConvention:
// X||a vs X||a* is a 30 deg rotation of the Cartesian basis about c, which only
// moves where the poles land in the figure (handled in the direction tables),
// not which family is plotted. So the labels are identical for both conventions:
// {0001}, {10-10}, {11-20}. ({2-1-10} is a synonym for the a-axis family {11-20};
// we use the standard {11-20} for both conventions rather than switching titles.)
//
// Trigonal classes have their own prism-family structure; we only assert that the
// conv parameter is plumbed (no crash, three non-empty strings).
TEST_CASE("ebsdlib::LaueOpsTest::GetDefaultPoleFigureNames_HexConvention", "[EbsdLib][LaueOpsTest]")
{
  SECTION("HexagonalHigh: family labels are convention-independent")
  {
    HexagonalOps ops;
    auto labelsA = ops.getDefaultPoleFigureNames(ebsdlib::HexConvention::XParallelA);
    auto labelsAStar = ops.getDefaultPoleFigureNames(ebsdlib::HexConvention::XParallelAStar);
    CHECK(labelsA == labelsAStar); // family identity is convention-independent
    CHECK(labelsA[0] == "{0001}");
    CHECK(labelsA[1] == "{10-10}");
    CHECK(labelsA[2] == "{11-20}");
  }

  SECTION("HexagonalLow: same convention-independent labels as HexagonalHigh")
  {
    HexagonalLowOps ops;
    auto labelsA = ops.getDefaultPoleFigureNames(ebsdlib::HexConvention::XParallelA);
    auto labelsAStar = ops.getDefaultPoleFigureNames(ebsdlib::HexConvention::XParallelAStar);
    CHECK(labelsA == labelsAStar);
    CHECK(labelsA[0] == "{0001}");
    CHECK(labelsA[1] == "{10-10}");
    CHECK(labelsA[2] == "{11-20}");
  }

  SECTION("Trigonal classes: conv parameter is plumbed without crash")
  {
    TrigonalOps tHigh;
    auto tHighA = tHigh.getDefaultPoleFigureNames(ebsdlib::HexConvention::XParallelA);
    auto tHighAStar = tHigh.getDefaultPoleFigureNames(ebsdlib::HexConvention::XParallelAStar);
    CHECK(tHighA.size() == 3ULL);
    for(const auto& s : tHighA)
    {
      CHECK_FALSE(s.empty());
    }
    for(const auto& s : tHighAStar)
    {
      CHECK_FALSE(s.empty());
    }

    TrigonalLowOps tLow;
    auto tLowA = tLow.getDefaultPoleFigureNames(ebsdlib::HexConvention::XParallelA);
    auto tLowAStar = tLow.getDefaultPoleFigureNames(ebsdlib::HexConvention::XParallelAStar);
    for(const auto& s : tLowA)
    {
      CHECK_FALSE(s.empty());
    }
    for(const auto& s : tLowAStar)
    {
      CHECK_FALSE(s.empty());
    }
  }
}

// -----------------------------------------------------------------------------
// HexagonalOps::generatePoleFigure must assign the correct family labels to the
// three rendered figures. Family labels are convention-independent (the X||a vs
// X||a* choice only rotates pole positions, not family identity), so both
// conventions must yield the standard {0001}/{10-10}/{11-20} titles.
TEST_CASE("ebsdlib::LaueOpsTest::GeneratePoleFigure_AssignsFamilyLabels", "[EbsdLib][LaueOpsTest]")
{
  HexagonalOps ops;

  // One-orientation Euler array (identity); we don't care about the rendered
  // intensity, only about the labels assigned to the three returned figures.
  std::vector<float> eulerVec = {0.0F, 0.0F, 0.0F};
  std::vector<size_t> compDims = {3ULL};
  auto eulers = ebsdlib::FloatArrayType::FromStdVector(eulerVec, 1ULL, 3ULL, "Eulers");

  PoleFigureConfiguration_t cfgA;
  cfgA.eulers = eulers.get();
  cfgA.imageDim = 64;
  cfgA.lambertDim = 16;
  cfgA.numColors = 16;
  cfgA.discrete = false;
  cfgA.discreteHeatMap = false;
  cfgA.hexConvention = ebsdlib::HexConvention::XParallelA;

  PoleFigureConfiguration_t cfgAStar = cfgA;
  cfgAStar.hexConvention = ebsdlib::HexConvention::XParallelAStar;

  auto figuresA = ops.generatePoleFigure(cfgA);
  auto figuresAStar = ops.generatePoleFigure(cfgAStar);

  // The renderer assigns the names array to figuresA[i]->getName(); the family
  // labels are convention-independent, so both conventions yield {11-20}.
  REQUIRE(figuresA.size() == 3ULL);
  REQUIRE(figuresAStar.size() == 3ULL);
  CHECK(figuresA[2]->getName() == "{11-20}");
  CHECK(figuresAStar[2]->getName() == "{11-20}");
}

// -----------------------------------------------------------------------------
// PR 2h regression test: confirm that generateIPFTriangleLegend honors the
// HexConvention parameter for HexagonalOps. The colored SST region is
// convention-invariant for 6/mmm (the inversion fold + c-axis rotations
// reach the SST regardless of basal-plane sym-op choice — see the
// GenerateIPFColor_HexConvention_HexagonalOps comment below). What MUST
// differ between conventions is the Miller-index labels drawn around the
// unit circle: under X||a the cartesian +X axis is the a-vector
// ([2-1-10]); under X||a* it is the a*-vector ([10-10]). The legend
// rasters must therefore differ byte-for-byte. If they don't, the
// HexConvention parameter is being silently dropped through
// annotateIPFImage / drawIPFAnnotations.
TEST_CASE("ebsdlib::LaueOpsTest::GenerateIPFTriangleLegend_HexConvention_HexagonalOps", "[EbsdLib][LaueOpsTest]")
{
  HexagonalOps ops;
  constexpr int k_LegendDim = 256;
  auto legendAStar = ops.generateIPFTriangleLegend(k_LegendDim, false, ebsdlib::HexConvention::XParallelAStar);
  auto legendA = ops.generateIPFTriangleLegend(k_LegendDim, false, ebsdlib::HexConvention::XParallelA);

  REQUIRE(legendAStar != nullptr);
  REQUIRE(legendA != nullptr);
  REQUIRE(legendAStar->getSize() == legendA->getSize());

  // Sanity: both legends are non-trivial (not all-white) so we know the
  // SST coloring actually rendered.
  bool aStarHasContent = false;
  bool aHasContent = false;
  const size_t total = legendAStar->getSize();
  for(size_t i = 0; i < total; ++i)
  {
    if(legendAStar->getValue(i) != 0xFF)
    {
      aStarHasContent = true;
    }
    if(legendA->getValue(i) != 0xFF)
    {
      aHasContent = true;
    }
    if(aStarHasContent && aHasContent)
    {
      break;
    }
  }
  REQUIRE(aStarHasContent);
  REQUIRE(aHasContent);

  // The two legends must differ somewhere — the corner labels at angles 0°
  // and 330° (the SST eta=0 and eta=30° corners for 6/mmm) carry different
  // Miller-index strings under the two conventions, so the rasterized text
  // pixels must differ.
  bool different = false;
  for(size_t i = 0; i < total; ++i)
  {
    if(legendAStar->getValue(i) != legendA->getValue(i))
    {
      different = true;
      break;
    }
  }
  CHECK(different);
}

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
// PR 2c regression test: confirm that generateIPFColor honors the
// HexConvention parameter on HexagonalOps and that the convention-aware
// code path is exercised without crashing or returning garbage.
//
// For HexagonalOps (Laue class 6/mmm), IPF coloring is genuinely
// convention-invariant: the SST (0° <= eta <= 30°, 0° <= chi <= 90°) is
// reached via c-axis rotations plus the inversion fold (p[2] < 0 -> flip),
// and c-axis rotations are basis-invariant. The basal-plane 180° sym ops
// that differ between X||a and X||a* are operationally redundant for this
// Laue class -- any orientation reachable via a basal op is also reachable
// via the c-rotation/inversion combo.
//
// So the expected behavior here is: both conventions produce the SAME
// non-trivial RGB. PR 2d will exercise HexagonalLowOps / TrigonalOps /
// TrigonalLowOps where the SST geometry is different and the convention
// parameter MAY produce different IPF colors; that's where any color-shift
// regression would manifest.
// HexagonalOps::generateIPFColor is convention-invariant (the SST is invariant
// under the X||a ↔ X||a* basis rotation). The new API no longer takes a
// HexConvention argument; this test confirms the call returns a non-trivial
// color and behaves identically whether kind is default-omitted or TSL-explicit.
TEST_CASE("ebsdlib::LaueOpsTest::GenerateIPFColor_TSL_HexagonalOps", "[EbsdLib][LaueOpsTest]")
{
  HexagonalOps hexOps;
  // Phi=90° tilts the c-axis into the basal plane, putting a basal-plane
  // direction along sample-z (the IPF-Z reference direction). A non-trivial
  // orientation that exercises the FZ-reduction loop.
  double eulers[3] = {0.0, 90.0 * ebsdlib::constants::k_PiOver180D, 0.0};
  double refDir[3] = {0.0, 0.0, 1.0};

  Rgb colorDefault = hexOps.generateIPFColor(eulers, refDir, false);
  Rgb colorTslExplicit = hexOps.generateIPFColor(eulers, refDir, false, ebsdlib::ColorKeyKind::TSL);

  CHECK(colorDefault == colorTslExplicit);

  const int r = RgbColor::dRed(colorDefault);
  const int g = RgbColor::dGreen(colorDefault);
  const int b = RgbColor::dBlue(colorDefault);
  CHECK((r + g + b) > 0);
}

// -----------------------------------------------------------------------------
// PR 2d regression tests: same convention dispatch and same closed-form
// expectation as PR 2b but for HexagonalLowOps, TrigonalOps, and
// TrigonalLowOps. For all four hex/trig classes, family-1's first
// canonical (X||a*) cartesian direction rotates by R_z(+30°) under the
// X||a derivation. The c-axis ({0001}) is invariant.

#include "EbsdLib/LaueOps/HexagonalLowOps.h"
#include "EbsdLib/LaueOps/TrigonalLowOps.h"
#include "EbsdLib/LaueOps/TrigonalOps.h"

namespace
{
template <typename OpsT>
void checkSphereCoordsConvention(const ebsdlib::Matrix3X1D& expectedFamily1FirstAStar)
{
  OpsT ops;
  std::vector<float> eulerVec = {0.0F, 0.0F, 0.0F}; // identity orientation
  std::vector<size_t> dims = {3ULL};
  ebsdlib::FloatArrayType::Pointer eulers = ebsdlib::FloatArrayType::FromStdVector(eulerVec, 1ULL, 3ULL, "Eulers");
  ebsdlib::FloatArrayType::Pointer xyz0001 = ebsdlib::FloatArrayType::CreateArray(2ULL, dims, "f0", true);
  ebsdlib::FloatArrayType::Pointer xyz1010_aStar = ebsdlib::FloatArrayType::CreateArray(6ULL, dims, "f1aStar", true);
  ebsdlib::FloatArrayType::Pointer xyz1010_a = ebsdlib::FloatArrayType::CreateArray(6ULL, dims, "f1a", true);
  ebsdlib::FloatArrayType::Pointer xyz1120 = ebsdlib::FloatArrayType::CreateArray(6ULL, dims, "f2", true);

  // Default X||a*: family-1 first member matches the expected canonical value.
  ops.generateSphereCoordsFromEulers(eulers.get(), xyz0001.get(), xyz1010_aStar.get(), xyz1120.get(), ebsdlib::HexConvention::XParallelAStar);
  CHECK(xyz1010_aStar->getValue(0) == Approx(expectedFamily1FirstAStar[0]).margin(1e-5));
  CHECK(xyz1010_aStar->getValue(1) == Approx(expectedFamily1FirstAStar[1]).margin(1e-5));
  CHECK(xyz1010_aStar->getValue(2) == Approx(expectedFamily1FirstAStar[2]).margin(1e-5));

  // X||a: family-1 first member is the canonical rotated by R_z(+30°).
  // (cos30, -sin30; sin30, cos30) applied to (x, y, 0).
  const double c30 = std::cos(30.0 * ebsdlib::constants::k_PiOver180D);
  const double s30 = std::sin(30.0 * ebsdlib::constants::k_PiOver180D);
  const double expA_x = c30 * expectedFamily1FirstAStar[0] - s30 * expectedFamily1FirstAStar[1];
  const double expA_y = s30 * expectedFamily1FirstAStar[0] + c30 * expectedFamily1FirstAStar[1];

  ops.generateSphereCoordsFromEulers(eulers.get(), xyz0001.get(), xyz1010_a.get(), xyz1120.get(), ebsdlib::HexConvention::XParallelA);
  CHECK(xyz1010_a->getValue(0) == Approx(expA_x).margin(1e-5));
  CHECK(xyz1010_a->getValue(1) == Approx(expA_y).margin(1e-5));
  CHECK(xyz1010_a->getValue(2) == Approx(0.0).margin(1e-5));
}
} // namespace

TEST_CASE("ebsdlib::LaueOpsTest::GenerateSphereCoords_HexConvention_HexagonalLowOps", "[EbsdLib][LaueOpsTest]")
{
  // HexagonalLow family-1 ({10-10}) canonical first member: (1, 0, 0).
  checkSphereCoordsConvention<HexagonalLowOps>(ebsdlib::Matrix3X1D(1.0, 0.0, 0.0));
}

TEST_CASE("ebsdlib::LaueOpsTest::GenerateSphereCoords_HexConvention_TrigonalOps", "[EbsdLib][LaueOpsTest]")
{
  // TrigonalOps family-1 (<0-110>-style) canonical first member: (-0.5, -sqrt(3)/2, 0).
  checkSphereCoordsConvention<TrigonalOps>(ebsdlib::Matrix3X1D(-0.5, -ebsdlib::constants::k_Root3Over2D, 0.0));
}

TEST_CASE("ebsdlib::LaueOpsTest::GenerateSphereCoords_HexConvention_TrigonalLowOps", "[EbsdLib][LaueOpsTest]")
{
  // TrigonalLowOps family-1 (<-1-120>-style) canonical first member: (-sqrt(3)/2, -0.5, 0).
  checkSphereCoordsConvention<TrigonalLowOps>(ebsdlib::Matrix3X1D(-ebsdlib::constants::k_Root3Over2D, -0.5, 0.0));
}

// -----------------------------------------------------------------------------
// PR 2b regression test: confirm that generateSphereCoordsFromEulers honors
// the HexConvention parameter for HexagonalOps. For an identity orientation,
// the {10-10} family-1 first member sits at:
//   - X||a* (default): cartesian (1, 0, 0)        -- a*1 along X
//   - X||a:            cartesian (cos30°, sin30°) -- a*1 at +30° from a (= X)
TEST_CASE("ebsdlib::LaueOpsTest::GenerateSphereCoords_HexConvention_HexagonalOps", "[EbsdLib][LaueOpsTest]")
{
  HexagonalOps hexOps;

  // Identity orientation -> sample-frame projection equals crystal-frame direction.
  std::vector<float> eulerVec = {0.0F, 0.0F, 0.0F};
  std::vector<size_t> dims = {3ULL};
  ebsdlib::FloatArrayType::Pointer eulers = ebsdlib::FloatArrayType::FromStdVector(eulerVec, 1ULL, 3ULL, "Eulers");
  ebsdlib::FloatArrayType::Pointer xyz0001 = ebsdlib::FloatArrayType::CreateArray(2ULL, dims, "f0", true);
  ebsdlib::FloatArrayType::Pointer xyz1010_aStar = ebsdlib::FloatArrayType::CreateArray(6ULL, dims, "f1aStar", true);
  ebsdlib::FloatArrayType::Pointer xyz1010_a = ebsdlib::FloatArrayType::CreateArray(6ULL, dims, "f1a", true);
  ebsdlib::FloatArrayType::Pointer xyz1120 = ebsdlib::FloatArrayType::CreateArray(6ULL, dims, "f2", true);

  // Default (X||a*) path: family-1 first member should be (1, 0, 0).
  hexOps.generateSphereCoordsFromEulers(eulers.get(), xyz0001.get(), xyz1010_aStar.get(), xyz1120.get(), ebsdlib::HexConvention::XParallelAStar);
  CHECK(xyz1010_aStar->getValue(0) == Approx(1.0F).margin(1e-5));
  CHECK(xyz1010_aStar->getValue(1) == Approx(0.0F).margin(1e-5));
  CHECK(xyz1010_aStar->getValue(2) == Approx(0.0F).margin(1e-5));

  // Explicit X||a: family-1 first member should be (cos30°, sin30°, 0) ≈ (0.866, 0.5, 0).
  hexOps.generateSphereCoordsFromEulers(eulers.get(), xyz0001.get(), xyz1010_a.get(), xyz1120.get(), ebsdlib::HexConvention::XParallelA);
  CHECK(xyz1010_a->getValue(0) == Approx(std::cos(30.0 * ebsdlib::constants::k_PiOver180D)).margin(1e-5));
  CHECK(xyz1010_a->getValue(1) == Approx(std::sin(30.0 * ebsdlib::constants::k_PiOver180D)).margin(1e-5));
  CHECK(xyz1010_a->getValue(2) == Approx(0.0F).margin(1e-5));

  // Sanity: the two outputs differ (the XParallelA opt-in must produce a
  // different sample-frame projection than the default).
  const float dx = xyz1010_aStar->getValue(0) - xyz1010_a->getValue(0);
  const float dy = xyz1010_aStar->getValue(1) - xyz1010_a->getValue(1);
  CHECK(std::sqrt(dx * dx + dy * dy) > 0.1F);

  // c-axis ({0001} family) is convention-invariant: should match.
  ebsdlib::FloatArrayType::Pointer xyz0001_a = ebsdlib::FloatArrayType::CreateArray(2ULL, dims, "f0a", true);
  hexOps.generateSphereCoordsFromEulers(eulers.get(), xyz0001_a.get(), xyz1010_a.get(), xyz1120.get(), ebsdlib::HexConvention::XParallelA);
  for(size_t i = 0; i < 6; ++i)
  {
    CHECK(xyz0001->getValue(i) == Approx(xyz0001_a->getValue(i)).margin(1e-5));
  }
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

// -----------------------------------------------------------------------------
// Strict IPF-color assertions for the standard-stereographic-triangle (SST)
// corners of EVERY Laue class. At the identity orientation the crystal frame
// equals the sample frame, so a reference direction maps directly onto that
// crystal direction; the IPF color key then paints the three SST corners as the
// pure primaries. The mapping is universal:
//
//   IPF legend corner -> color (TSL key)
//   -------------------------------------------------------------------------
//   SST apex (principal axis, chi = 0)        -> RED   (255,  0,  0)
//   eta = etaMin edge corner (chi = chiMax)   -> GREEN (  0,255,  0)
//   eta = etaMax edge corner (chi = chiMax)   -> BLUE  (  0,  0,255)
//
//   Laue class            eta[min,max] deg   apex    green(etaMin)  blue(etaMax)
//   Cubic       m-3m       [   0,  45]        [001]   [101]          [111]
//   Cubic-Low   m-3        [   0,  90]        [001]   [011]          eta-max edge
//   Hexagonal   6/mmm      [   0,  30]        [0001]  [2-1-10]       [10-10]
//   Hexagonal-L 6/m        [   0,  60]        [0001]  [2-1-10]       [11-20]
//   Tetragonal  4/mmm      [   0,  45]        [001]   [100]          [110]
//   Tetragonal-L 4/m       [   0,  90]        [001]   [100]          [010]
//   Trigonal    -3m        [ -90, -30]        [0001]  eta-min edge   eta-max edge
//   Trigonal-L  -3         [-120,   0]        [0001]  eta-min edge   eta-max edge
//   Orthorhombic mmm       [   0,  90]        [001]   [100]          [010]
//   Monoclinic  2/m        [   0, 180]        [001]   eta-min edge   eta-max edge
//   Triclinic   -1         [   0, 180]        [001]   eta-min edge   eta-max edge
//
// The all-class section below derives each class's eta range and chiMax from
// getIpfColorAngleLimits() and asserts the universal mapping. Exact pure-primary
// values are additionally asserted at the canonical corners of the high-symmetry
// cubic (m-3m) and hexagonal (6/mmm) triangles. (The exact corners lie on a
// symmetry fold boundary, so the all-class checks sample a hair inside each edge
// and assert the expected primary strictly dominates.) Confirmed against the
// built library; the IPF color is independent of the X||a / X||a* convention.
namespace
{
ebsdlib::Rgb ipfColorAtDir(const ebsdlib::LaueOps& ops, double x, double y, double z)
{
  double e[3] = {0.0, 0.0, 0.0}; // identity orientation
  const double n = std::sqrt(x * x + y * y + z * z);
  double d[3] = {x / n, y / n, z / n};
  return ops.generateIPFColor(e, d, false); // default TSL color key
}

// Direction at azimuth eta and polar angle chi from the c-axis (radians).
ebsdlib::Rgb ipfColorAtEtaChi(const ebsdlib::LaueOps& ops, double eta, double chi)
{
  return ipfColorAtDir(ops, std::sin(chi) * std::cos(eta), std::sin(chi) * std::sin(eta), std::cos(chi));
}

void checkExact(ebsdlib::Rgb c, int r, int g, int b, const std::string& what)
{
  INFO(what << " -> R=" << RgbColor::dRed(c) << " G=" << RgbColor::dGreen(c) << " B=" << RgbColor::dBlue(c));
  CHECK(RgbColor::dRed(c) == r);
  CHECK(RgbColor::dGreen(c) == g);
  CHECK(RgbColor::dBlue(c) == b);
}

// Assert channel `ch` (0=R, 1=G, 2=B) is the strong, strict dominant.
void checkDominant(ebsdlib::Rgb c, int ch, const std::string& what)
{
  const int v[3] = {RgbColor::dRed(c), RgbColor::dGreen(c), RgbColor::dBlue(c)};
  INFO(what << " -> R=" << v[0] << " G=" << v[1] << " B=" << v[2]);
  CHECK(v[ch] >= 200);
  CHECK(v[ch] > v[(ch + 1) % 3]);
  CHECK(v[ch] > v[(ch + 2) % 3]);
}
} // namespace

TEST_CASE("ebsdlib::LaueOpsTest::IPFColor_SSTCorners", "[EbsdLib][LaueOpsTest]")
{
  const double c30 = std::sqrt(3.0) / 2.0; // cos(30) == sin(60)

  SECTION("All Laue classes: SST apex=red, eta-min edge=green, eta-max edge=blue")
  {
    std::vector<LaueOps::Pointer> allOps = LaueOps::GetAllOrientationOps();
    int checked = 0;
    for(const auto& op : allOps)
    {
      if(op == nullptr)
      {
        continue;
      }
      const std::string name = op->getNameOfClass();
      const std::array<double, 3> lim = op->getIpfColorAngleLimits(0.0);
      const double etaMin = lim[0];
      const double etaMax = lim[1];
      if(!(etaMax > etaMin))
      {
        continue; // class has no eta-parameterized SST (skip)
      }
      // Sample a hair inside each edge to avoid the fold-boundary ambiguity.
      const double d = (etaMax - etaMin) * 0.03;
      const double gEta = etaMin + d;
      const double bEta = etaMax - d;
      const double gChi = op->getIpfColorAngleLimits(gEta)[2] * 0.97;
      const double bChi = op->getIpfColorAngleLimits(bEta)[2] * 0.97;

      checkExact(ipfColorAtEtaChi(*op, 0.0, 0.0), 255, 0, 0, name + " apex");
      checkDominant(ipfColorAtEtaChi(*op, gEta, gChi), 1, name + " eta-min edge (green)");
      checkDominant(ipfColorAtEtaChi(*op, bEta, bChi), 2, name + " eta-max edge (blue)");
      ++checked;
    }
    INFO("Laue classes checked: " << checked);
    CHECK(checked >= 11); // every Laue class must be exercised
  }

  SECTION("Cubic m-3m canonical corners are exact primaries: [001]=R, [011]=G, [111]=B")
  {
    CubicOps ops;
    checkExact(ipfColorAtDir(ops, 0, 0, 1), 255, 0, 0, "[001]");
    checkExact(ipfColorAtDir(ops, 0, 1, 1), 0, 255, 0, "[011]");
    checkExact(ipfColorAtDir(ops, 1, 1, 1), 0, 0, 255, "[111]");
  }

  SECTION("Hexagonal 6/mmm canonical corners are exact primaries: [0001]=R, [2-1-10]=G, [10-10]=B")
  {
    HexagonalOps ops;
    checkExact(ipfColorAtDir(ops, 0, 0, 1), 255, 0, 0, "[0001]");
    checkExact(ipfColorAtDir(ops, 1, 0, 0), 0, 255, 0, "[2-1-10] eta=0");
    checkExact(ipfColorAtDir(ops, c30, 0.5, 0), 0, 0, 255, "[10-10] eta=30");
  }

  // Cross-class contrast: a basal direction near eta=0 is the GREEN side of the
  // 6/m wedge [0,60] but the BLUE side of the -3 wedge [-120,0].
  SECTION("Same eta region, different class -> different color")
  {
    const double oneDeg = ebsdlib::constants::k_PiOver180D;
    const double chi90 = ebsdlib::constants::k_PiOver2D;
    checkDominant(ipfColorAtEtaChi(HexagonalLowOps(), 1.0 * oneDeg, chi90), 1, "HexLow eta~0 (green)");
    checkDominant(ipfColorAtEtaChi(TrigonalLowOps(), -1.0 * oneDeg, chi90), 2, "TrigLow eta~0 (blue)");
  }
}
