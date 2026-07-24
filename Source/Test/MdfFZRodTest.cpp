#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdLibConstants.h"
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
#include "EbsdLib/Orientation/AxisAngle.hpp"
#include "EbsdLib/Orientation/Rodrigues.hpp"

#include <array>
#include <cmath>
#include <random>
#include <string>
#include <vector>

using namespace ebsdlib;

// -----------------------------------------------------------------------------
// getMDFFZRod() must map every symmetry-equivalent description of a misorientation
// to a single canonical Rodrigues vector in the MDF fundamental zone. Two
// misorientation axes n and n' are equivalent when
//
//   n' = R(S_i) * n      (conjugation by a rotational symmetry operator S_i), or
//   n' = -R(S_i) * n     (the above combined with switching symmetry: a boundary
//                          has no preferred direction, so a misorientation and its
//                          inverse -- which negates the axis -- are the same), and
//
// the misorientation angle w is invariant under both. These tests verify, for all
// eleven Laue classes:
//
//   1. Completeness: every equivalent axis folds to the same canonical rod.
//   2. Angle preservation: the rotation angle survives the fold unchanged.
//   3. Idempotence: the canonical rod is a fixed point of getMDFFZRod().
//   4. Distinctness: axes that are NOT symmetry-equivalent fold to different rods
//      (guards against over-folding; each pair below was wrongly merged by a
//      historical implementation).
//
// The rotation angles used are kept below 30 degrees -- half the smallest nonzero
// symmetry rotation (60 degrees for the 6-fold) -- so the minimum-angle
// representative selection inside getMDFFZRod() is the identity for every variant
// and the tests isolate the axis-fold logic itself.
namespace
{
std::array<double, 3> matVec(const Matrix3X3D& m, const std::array<double, 3>& v)
{
  return {m(0, 0) * v[0] + m(0, 1) * v[1] + m(0, 2) * v[2], m(1, 0) * v[0] + m(1, 1) * v[1] + m(1, 2) * v[2], m(2, 0) * v[0] + m(2, 1) * v[1] + m(2, 2) * v[2]};
}

std::array<double, 3> normalize(std::array<double, 3> v)
{
  const double mag = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  return {v[0] / mag, v[1] / mag, v[2] / mag};
}

RodriguesDType rodFromAxisAngle(const std::array<double, 3>& axis, double angle)
{
  return AxisAngleDType(axis[0], axis[1], axis[2], angle).toRodrigues();
}

// Compare two Rodrigues vectors through their axis-angle representation.
void checkSameRod(const RodriguesDType& computed, const RodriguesDType& expected, const std::string& what)
{
  AxisAngleDType axComputed = computed.toAxisAngle();
  AxisAngleDType axExpected = expected.toAxisAngle();
  INFO(what << " computed=(" << axComputed[0] << ", " << axComputed[1] << ", " << axComputed[2] << ", " << axComputed[3] << ")"
            << " expected=(" << axExpected[0] << ", " << axExpected[1] << ", " << axExpected[2] << ", " << axExpected[3] << ")");
  CHECK(axComputed[0] == Approx(axExpected[0]).margin(1.0e-5));
  CHECK(axComputed[1] == Approx(axExpected[1]).margin(1.0e-5));
  CHECK(axComputed[2] == Approx(axExpected[2]).margin(1.0e-5));
  CHECK(axComputed[3] == Approx(axExpected[3]).margin(1.0e-6));
}

bool rodsDiffer(const RodriguesDType& a, const RodriguesDType& b)
{
  AxisAngleDType axA = a.toAxisAngle();
  AxisAngleDType axB = b.toAxisAngle();
  double dist = 0.0;
  for(size_t i = 0; i < 3; i++)
  {
    dist += std::fabs(axA[i] - axB[i]);
  }
  return dist > 1.0e-3;
}

std::array<double, 3> randomAxis(std::mt19937& gen)
{
  std::normal_distribution<double> dist(0.0, 1.0);
  while(true)
  {
    std::array<double, 3> v = {dist(gen), dist(gen), dist(gen)};
    const double mag = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if(mag > 1.0e-3)
    {
      return {v[0] / mag, v[1] / mag, v[2] / mag};
    }
  }
}

// An axis at basal-plane azimuth (degrees) with the given z component.
std::array<double, 3> axisAtAzimuth(double azimuthDegrees, double z)
{
  const double azimuth = azimuthDegrees * ebsdlib::constants::k_PiOver180D;
  const double mag = std::sqrt(1.0 - z * z);
  return {mag * std::cos(azimuth), mag * std::sin(azimuth), z};
}

// Verify two inequivalent axes fold to different canonical rods.
void checkDistinct(const LaueOps& ops, const std::array<double, 3>& axisA, const std::array<double, 3>& axisB, double angle, const std::string& what)
{
  RodriguesDType foldA = ops.getMDFFZRod(rodFromAxisAngle(axisA, angle));
  RodriguesDType foldB = ops.getMDFFZRod(rodFromAxisAngle(axisB, angle));
  AxisAngleDType axA = foldA.toAxisAngle();
  AxisAngleDType axB = foldB.toAxisAngle();
  INFO(what << " foldA=(" << axA[0] << ", " << axA[1] << ", " << axA[2] << ")"
            << " foldB=(" << axB[0] << ", " << axB[1] << ", " << axB[2] << ")");
  CHECK(rodsDiffer(foldA, foldB));
}
} // namespace

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::MdfFZRodTest::EquivalentAxesCollapse", "[EbsdLib][MdfFZRodTest]")
{
  constexpr size_t k_NumRandomAxes = 25;
  const std::vector<double> k_Angles = {2.0 * ebsdlib::constants::k_PiOver180D, 25.0 * ebsdlib::constants::k_PiOver180D};

  auto allOps = LaueOps::GetAllOrientationOps();
  size_t classesChecked = 0;

  for(size_t laueIdx = 0; laueIdx < 11; laueIdx++)
  {
    const LaueOps& ops = *allOps[laueIdx];
    const std::string className = ops.getNameOfClass();
    const size_t numSym = ops.getNumSymOps();

    SECTION(className)
    {
      std::mt19937 gen(20260722);
      for(double angle : k_Angles)
      {
        for(size_t trial = 0; trial < k_NumRandomAxes; trial++)
        {
          const std::array<double, 3> axis = randomAxis(gen);
          const RodriguesDType reference = ops.getMDFFZRod(rodFromAxisAngle(axis, angle));

          // Angle preservation: the fold must not alter the misorientation angle
          {
            AxisAngleDType axRef = reference.toAxisAngle();
            INFO(className << " angle preservation, trial " << trial);
            CHECK(axRef[3] == Approx(angle).margin(1.0e-6));
          }

          // Idempotence: the canonical rod must be a fixed point
          checkSameRod(ops.getMDFFZRod(reference), reference, className + " idempotence");

          // Completeness: every conjugated and/or switched axis folds to the reference
          for(size_t symIdx = 0; symIdx < numSym; symIdx++)
          {
            const std::array<double, 3> conjugated = matVec(ops.getMatSymOpD(symIdx), axis);
            const std::array<double, 3> switched = {-conjugated[0], -conjugated[1], -conjugated[2]};

            checkSameRod(ops.getMDFFZRod(rodFromAxisAngle(conjugated, angle)), reference, className + " conjugation by op " + std::to_string(symIdx));
            checkSameRod(ops.getMDFFZRod(rodFromAxisAngle(switched, angle)), reference, className + " switching + op " + std::to_string(symIdx));
          }
        }
      }
      classesChecked++;
    }
  }
}

// -----------------------------------------------------------------------------
// Axes lying exactly on symmetry elements (the c-axis, a 2-fold axis, a body
// diagonal) sit on fold-sector boundaries. The fold must handle them without
// producing NaN and must still collapse their +/- variants.
TEST_CASE("ebsdlib::MdfFZRodTest::BoundaryAxes", "[EbsdLib][MdfFZRodTest]")
{
  const double angle = 10.0 * ebsdlib::constants::k_PiOver180D;
  const double k_Root3Inverse = 1.0 / std::sqrt(3.0);
  const std::vector<std::array<double, 3>> boundaryAxes = {
      {0.0, 0.0, 1.0},
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {k_Root3Inverse, k_Root3Inverse, k_Root3Inverse},
  };

  auto allOps = LaueOps::GetAllOrientationOps();

  for(size_t laueIdx = 0; laueIdx < 11; laueIdx++)
  {
    const LaueOps& ops = *allOps[laueIdx];
    const std::string className = ops.getNameOfClass();

    SECTION(className)
    {
      for(const std::array<double, 3>& axis : boundaryAxes)
      {
        const RodriguesDType folded = ops.getMDFFZRod(rodFromAxisAngle(axis, angle));
        AxisAngleDType axFolded = folded.toAxisAngle();

        INFO(className << " boundary axis (" << axis[0] << ", " << axis[1] << ", " << axis[2] << ")");
        for(size_t i = 0; i < 4; i++)
        {
          CHECK(std::isfinite(axFolded[i]));
        }
        CHECK(axFolded[3] == Approx(angle).margin(1.0e-6));

        // The negated axis is the switched (inverse) misorientation: must collapse
        const std::array<double, 3> negated = {-axis[0], -axis[1], -axis[2]};
        checkSameRod(ops.getMDFFZRod(rodFromAxisAngle(negated, angle)), folded, className + " negated boundary axis");
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Axes that are NOT symmetry-equivalent must fold to DIFFERENT canonical rods.
// Every pair below was wrongly merged by a historical implementation of
// getMDFFZRod(), so these are regression guards against over-folding.
TEST_CASE("ebsdlib::MdfFZRodTest::InequivalentAxesStayDistinct", "[EbsdLib][MdfFZRodTest]")
{
  const double angle = 10.0 * ebsdlib::constants::k_PiOver180D;

  SECTION("CubicLowOps: m-3 has no 2-component transposition")
  {
    // The tetrahedral group's <111> 3-folds give only cyclic permutations of the
    // axis components; a transposition of the two smaller components is not a
    // symmetry (the old full descending sort merged these).
    CubicLowOps ops;
    checkDistinct(ops, normalize({0.8, 0.5, 0.33}), normalize({0.8, 0.33, 0.5}), angle, "CubicLow transposition");
  }

  SECTION("TetragonalOps: 422 has no 3-fold")
  {
    TetragonalOps ops;
    checkDistinct(ops, normalize({0.8, 0.5, 0.33}), normalize({0.33, 0.8, 0.5}), angle, "Tetragonal cyclic permutation");
  }

  SECTION("TetragonalLowOps: 4/m has no in-plane mirror")
  {
    // Azimuth +20 and -20 degrees are mirror images; 4/m has no in-plane 2-folds
    // so they are inequivalent (the old fabs fold merged them).
    TetragonalLowOps ops;
    checkDistinct(ops, axisAtAzimuth(20.0, 0.7), axisAtAzimuth(-20.0, 0.7), angle, "TetragonalLow azimuth mirror");
  }

  SECTION("HexagonalLowOps: 6/m has no in-plane mirror")
  {
    HexagonalLowOps ops;
    checkDistinct(ops, axisAtAzimuth(20.0, 0.7), axisAtAzimuth(-20.0, 0.7), angle, "HexagonalLow azimuth mirror");
  }

  SECTION("TrigonalLowOps: -3 has no in-plane mirror")
  {
    TrigonalLowOps ops;
    checkDistinct(ops, axisAtAzimuth(20.0, 0.7), axisAtAzimuth(-20.0, 0.7), angle, "TrigonalLow azimuth mirror");
  }

  SECTION("TrigonalOps: mirror lines sit at 30+60k degrees, not 60k")
  {
    // Azimuths 70 and 50 degrees are NOT related by the 32 group's mirrors at
    // 30/90/150 degrees (the old fan-fold with boundaries at 60k merged them).
    TrigonalOps ops;
    checkDistinct(ops, axisAtAzimuth(70.0, 0.7), axisAtAzimuth(50.0, 0.7), angle, "Trigonal 70 vs 50 degrees");
  }

  SECTION("TriclinicOps: only switching symmetry")
  {
    TriclinicOps ops;
    checkDistinct(ops, normalize({0.5, 0.6, 0.62}), normalize({-0.5, 0.6, 0.62}), angle, "Triclinic single-component flip");
  }

  SECTION("MonoclinicOps: single-component flip of n1 is not a symmetry")
  {
    MonoclinicOps ops;
    checkDistinct(ops, normalize({0.5, 0.6, 0.62}), normalize({-0.5, 0.6, 0.62}), angle, "Monoclinic single-component flip");
  }

  SECTION("OrthoRhombicOps: mmm has no permutation symmetry")
  {
    OrthoRhombicOps ops;
    checkDistinct(ops, normalize({0.8, 0.5, 0.33}), normalize({0.5, 0.8, 0.33}), angle, "OrthoRhombic transposition");
  }
}

// -----------------------------------------------------------------------------
// The misorientation angle discarded by historical CubicLowOps / TrigonalOps /
// TrigonalLowOps implementations: verify explicitly for every class that a
// nonzero input angle never comes back as zero.
TEST_CASE("ebsdlib::MdfFZRodTest::AngleNeverDiscarded", "[EbsdLib][MdfFZRodTest]")
{
  const double angle = 15.0 * ebsdlib::constants::k_PiOver180D;

  auto allOps = LaueOps::GetAllOrientationOps();

  for(size_t laueIdx = 0; laueIdx < 11; laueIdx++)
  {
    const LaueOps& ops = *allOps[laueIdx];
    const std::string className = ops.getNameOfClass();

    SECTION(className)
    {
      std::mt19937 gen(42);
      for(size_t trial = 0; trial < 5; trial++)
      {
        const std::array<double, 3> axis = randomAxis(gen);
        AxisAngleDType axFolded = ops.getMDFFZRod(rodFromAxisAngle(axis, angle)).toAxisAngle();
        INFO(className << " trial " << trial);
        CHECK(axFolded[3] == Approx(angle).margin(1.0e-6));
      }
    }
  }
}
