#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Orientation/AxisAngle.hpp"
#include "EbsdLib/Orientation/Quaternion.hpp"
#include "EbsdLib/Orientation/Rodrigues.hpp"
#include "EbsdLib/Texture/MisorientationKDE.h"
#include "EbsdLib/Texture/RandomAngleDistribution.h"
#include "EbsdLib/Texture/SO3DeLaValleePoussinKernel.h"

#include <array>
#include <cmath>
#include <random>
#include <vector>

using namespace ebsdlib;

namespace
{
constexpr double k_DegToRad = ebsdlib::constants::k_PiOver180D;

// Build a unit quaternion from a (not necessarily unit) axis and an angle in radians.
QuatD quatFromAxisAngle(double ax, double ay, double az, double angleRadians)
{
  const double mag = std::sqrt(ax * ax + ay * ay + az * az);
  const double nx = ax / mag;
  const double ny = ay / mag;
  const double nz = az / mag;
  const double s = std::sin(angleRadians / 2.0);
  return QuatD(nx * s, ny * s, nz * s, std::cos(angleRadians / 2.0));
}

} // namespace

// -----------------------------------------------------------------------------
// Triclinic has a single (identity) symmetry operator, so the crystal-symmetry
// average is trivial and the kernel density around one isolated misorientation
// reduces to the raw De la Vallee Poussin kernel scaled by the 0.5 antipodal
// normalization factor. Its modal height is K(0) / 2 = psi.evaluate(1.0) / 2, it
// falls to a quarter peak a halfwidth away, and it is exactly zero beyond the cutoff.
// (The 0.5 factor was pinned by the MTEX cross-check in CubicAngleCurveVsMTEX.)
TEST_CASE("ebsdlib::MisorientationKDE::SingleCenterTriclinic", "[EbsdLib][MisorientationKDE]")
{
  auto opsList = ebsdlib::LaueOps::GetAllOrientationOps();
  auto ops = opsList[ebsdlib::CrystalStructure::Triclinic];
  const double hw = 10.0 * k_DegToRad;
  ebsdlib::MisorientationKDE kde(ops, ebsdlib::CrystalStructure::Triclinic, hw);

  // center: 30 degrees about z
  ebsdlib::QuatD c(0.0, 0.0, std::sin(15.0 * k_DegToRad), std::cos(15.0 * k_DegToRad));
  kde.addMisorientation(c, 3.0); // non-unit weight; must normalize to 1
  kde.finalize();
  REQUIRE(kde.totalWeight() == Approx(3.0));

  ebsdlib::SO3DeLaValleePoussinKernel psi(hw);

  // The gridify step snaps the center to its bin center, so evaluate the *bin
  // center*, not the original quat.
  const int bin = ops->getMisoBin(ops->getMDFFZRod(c.toRodrigues()));
  ebsdlib::QuatD snapped = kde.binCenter(bin);

  // Modal peak: density at the snapped center equals K(0) / 2 (the 0.5 antipodal factor).
  REQUIRE(kde.evaluate(snapped) == Approx(psi.evaluate(1.0) / 2.0).epsilon(0.01));

  // Rotate the snapped center by hw about an orthogonal axis -> quarter peak (half of K(0)/2).
  ebsdlib::QuatD dq(std::sin(hw / 2.0), 0.0, 0.0, std::cos(hw / 2.0));
  REQUIRE(kde.evaluate(dq * snapped) == Approx(psi.evaluate(1.0) / 4.0).epsilon(0.02));

  // Beyond the cutoff -> exactly zero.
  ebsdlib::QuatD far(0.0, std::sin(60.0 * k_DegToRad), 0.0, std::cos(60.0 * k_DegToRad));
  REQUIRE(kde.evaluate(far * snapped) == 0.0);
}

// -----------------------------------------------------------------------------
// Cubic (m-3m) exercises the full crystal-symmetry averaging and the antipodal
// (grain-exchange) folding: the density must be invariant under s_1 * q * s_2 for
// every symmetry pair, invariant under grain exchange q -> q^-1, and normalized so
// that its mean over the fundamental zone is close to 1.
TEST_CASE("ebsdlib::MisorientationKDE::CubicInvarianceAndMean", "[EbsdLib][MisorientationKDE]")
{
  auto ops = ebsdlib::LaueOps::GetAllOrientationOps()[ebsdlib::CrystalStructure::Cubic_High];
  ebsdlib::MisorientationKDE kde(ops, ebsdlib::CrystalStructure::Cubic_High, 10.0 * k_DegToRad);

  // Three arbitrary misorientations with unequal weights.
  kde.addMisorientation(quatFromAxisAngle(0.0, 0.0, 1.0, 25.0 * k_DegToRad), 1.0);
  kde.addMisorientation(quatFromAxisAngle(1.0, 1.0, 1.0, 40.0 * k_DegToRad), 2.0);
  kde.addMisorientation(quatFromAxisAngle(0.0, 1.0, 2.0, 55.0 * k_DegToRad), 3.0);
  kde.finalize();

  REQUIRE(kde.totalWeight() == Approx(6.0));

  // A generic query misorientation.
  ebsdlib::QuatD query = quatFromAxisAngle(1.0, 2.0, 3.0, 33.0 * k_DegToRad);
  const double reference = kde.evaluate(query);
  REQUIRE(reference > 0.0);

  const size_t numSymOps = ops->getNumSymOps();

  SECTION("crystal symmetry invariance: f(s1 * q * s2) == f(q)")
  {
    const std::array<size_t, 4> leftIdx = {0, 1, 5, 11};
    const std::array<size_t, 4> rightIdx = {0, 2, 7, 13};
    for(size_t li : leftIdx)
    {
      for(size_t ri : rightIdx)
      {
        if(li >= numSymOps || ri >= numSymOps)
        {
          continue;
        }
        ebsdlib::QuatD s1 = ops->getQuatSymOp(li);
        ebsdlib::QuatD s2 = ops->getQuatSymOp(ri);
        ebsdlib::QuatD equivalent = s1 * query * s2;
        INFO("left op " << li << " right op " << ri);
        CHECK(kde.evaluate(equivalent) == Approx(reference).epsilon(1.0e-6));
      }
    }
  }

  SECTION("grain-exchange invariance: f(q^-1) == f(q)")
  {
    CHECK(kde.evaluate(query.conjugate()) == Approx(reference).epsilon(1.0e-6));
  }

  SECTION("normalization: mean density over non-identity bin centers is O(1)")
  {
    std::vector<double> densities = kde.evaluateAtBinCenters();
    REQUIRE(densities.size() == ops->getMDFSize());

    double sum = 0.0;
    size_t count = 0;
    for(size_t binIndex = 0; binIndex < densities.size(); binIndex++)
    {
      ebsdlib::QuatD center = kde.binCenter(static_cast<int>(binIndex));
      // Disorientation angle of the bin center from identity: omega = 2*acos(|w|).
      const double cosHalf = std::fabs(center.w());
      const double omega = 2.0 * std::acos(std::min(1.0, cosHalf));
      if(omega > 1.0e-3)
      {
        sum += densities[binIndex];
        count++;
      }
    }
    REQUIRE(count > 0);
    const double mean = sum / static_cast<double>(count);
    INFO("mean density over " << count << " bins = " << mean);
    CHECK(mean > 0.5);
    CHECK(mean < 2.0);
  }
}

// -----------------------------------------------------------------------------
// Numerical cross-check against MTEX 6.1.0. The same 3-center cubic KDE
// (weights 1,2,3) is built, and computeAngleCurve(200) is compared against
// MTEX's calcDensity(...,'exact') -> calcAngleDistribution reference at 20
// sampled angles. This is the authority that pins the KDE's absolute scale.
//
// MTEX reference generated with:
//   cs=crystalSymmetry('m-3m');
//   ax=[vector3d(0,0,1), vector3d(1,1,1)/norm(vector3d(1,1,1)), vector3d(0,1,2)/norm(vector3d(0,1,2))];
//   om=[25 40 55]*degree; mori=orientation('axis',ax,'angle',om,cs,cs); w=[1 2 3];
//   mdf=calcDensity(mori,'weights',w,'halfwidth',10*degree,'exact');
//   [d,omega]=calcAngleDistribution(mdf);
// MTEX omega(k) == maxAngle*(k-1)/199, so MTEX index k maps to C++ curve index k-1.
TEST_CASE("ebsdlib::MisorientationKDE::CubicAngleCurveVsMTEX", "[EbsdLib][MisorientationKDE]")
{
  auto ops = ebsdlib::LaueOps::GetAllOrientationOps()[ebsdlib::CrystalStructure::Cubic_High];
  ebsdlib::MisorientationKDE kde(ops, ebsdlib::CrystalStructure::Cubic_High, 10.0 * k_DegToRad);
  kde.addMisorientation(quatFromAxisAngle(0.0, 0.0, 1.0, 25.0 * k_DegToRad), 1.0);
  kde.addMisorientation(quatFromAxisAngle(1.0, 1.0, 1.0, 40.0 * k_DegToRad), 2.0);
  kde.addMisorientation(quatFromAxisAngle(0.0, 1.0, 2.0, 55.0 * k_DegToRad), 3.0);
  kde.finalize();

  const size_t numPoints = 200;
  ebsdlib::MisorientationKDE::AngleCurve curve = kde.computeAngleCurve(numPoints);

  REQUIRE(curve.Angles.size() == numPoints);
  REQUIRE(curve.Density.size() == numPoints);
  REQUIRE(curve.RandomDensity.size() == numPoints);

  // Angle grid endpoints.
  const double maxAngle = ebsdlib::random_angle_distribution::MaxMisorientationAngle(ebsdlib::CrystalStructure::Cubic_High);
  REQUIRE(curve.Angles.front() == Approx(0.0).margin(1.0e-12));
  REQUIRE(curve.Angles.back() == Approx(maxAngle));

  // RandomDensity must match the analytic reference exactly.
  std::vector<double> expectedRandom = ebsdlib::random_angle_distribution::Compute(ebsdlib::CrystalStructure::Cubic_High, curve.Angles);
  REQUIRE(expectedRandom.size() == numPoints);
  for(size_t i = 0; i < numPoints; i++)
  {
    INFO("RandomDensity mismatch at index " << i);
    CHECK(curve.RandomDensity[i] == Approx(expectedRandom[i]).margin(1.0e-12));
  }

  // 20 sampled MTEX (1-based index k, omega, density) reference pairs.
  const std::array<std::array<double, 3>, 20> mtexRef = {{{{1, 0.0000000000, 0.0000000000}},
                                                          {{11, 0.0550782319, 0.0020168845}},
                                                          {{21, 0.1101564638, 0.0111024402}},
                                                          {{31, 0.1652346958, 0.0366089735}},
                                                          {{41, 0.2203129277, 0.0928770466}},
                                                          {{51, 0.2753911596, 0.1866315579}},
                                                          {{61, 0.3304693915, 0.3349090171}},
                                                          {{71, 0.3855476235, 0.5090465258}},
                                                          {{81, 0.4406258554, 0.7013347724}},
                                                          {{91, 0.4957040873, 0.9018987543}},
                                                          {{101, 0.5507823192, 1.1704777051}},
                                                          {{111, 0.6058605511, 1.5494982458}},
                                                          {{121, 0.6609387831, 2.0076201248}},
                                                          {{131, 0.7160170150, 2.4918589687}},
                                                          {{141, 0.7710952469, 2.8632147418}},
                                                          {{151, 0.8261734788, 2.9262518556}},
                                                          {{161, 0.8812517107, 2.1954465862}},
                                                          {{171, 0.9363299427, 1.1739709599}},
                                                          {{181, 0.9914081746, 0.5369126565}},
                                                          {{191, 1.0464864065, 0.2176428572}}}};

  // Tolerance: the KDE math itself matches MTEX exactly. Evaluating the density at the
  // *exact* (un-gridified) misorientation centers reproduces MTEX's mdf to a constant
  // ratio of 2.000 at every angle (and mean(mdf) == 1 over SO(3)), which is what pinned
  // the 0.5 antipodal normalization factor in MisorientationKDE::evaluate(). The residual
  // pointwise deviation seen here (up to ~45% on the low-omega tail, ~18% on the rising
  // flank) is entirely the Task 3 MDF-bin gridify: addMisorientation() snaps each center
  // to its ~5-degree MDF fundamental-zone bin center, shifting the 25/40/55-degree inputs
  // by 1-2.5 degrees, which redistributes the steep 10-degree-halfwidth kernel across
  // omega. Removing the snap collapses the deviation to <0.1%. The tolerance below is
  // therefore set to the gridify band (20% relative + 0.10 absolute floor), not to the
  // KDE's intrinsic accuracy. See task-4-report.md for the exact-center evidence.
  for(const std::array<double, 3>& ref : mtexRef)
  {
    const size_t idx = static_cast<size_t>(std::lround(ref[0])) - 1; // 1-based -> 0-based
    const double mtexOmega = ref[1];
    const double mtexDensity = ref[2];
    INFO("MTEX k=" << ref[0] << " omega=" << mtexOmega << " -> curve index " << idx << " angle=" << curve.Angles[idx] << " density=" << curve.Density[idx]);
    CHECK(curve.Angles[idx] == Approx(mtexOmega).margin(1.0e-6));
    CHECK(curve.Density[idx] == Approx(mtexDensity).epsilon(0.20).margin(0.10));
  }
}

// -----------------------------------------------------------------------------
// Regression guard for a correlated MDF whose true peak is at a KNOWN non-45-degree
// angle. A tight cluster of Sigma3 (60 degree / <111>) misorientations on top of a
// random background must produce an MDF whose bin-array peak folds to a 60/<111>
// misorientation AND whose angle-distribution curve peaks near 60 degrees -- clearly
// distinct from the ~45 degree cubic Mackenzie (random-reference) maximum. This is the
// discriminating case the earlier 45-degree bicrystal cross-check could not catch: a
// density that collapsed to the random distribution would still peak at ~45 and pass a
// weaker test, but fails here.
TEST_CASE("ebsdlib::MisorientationKDE::CorrelatedTwinPeaksAtSixty", "[EbsdLib][MisorientationKDE]")
{
  auto ops = ebsdlib::LaueOps::GetAllOrientationOps()[ebsdlib::CrystalStructure::Cubic_High];
  const double hw = 10.0 * k_DegToRad;
  ebsdlib::MisorientationKDE kde(ops, ebsdlib::CrystalStructure::Cubic_High, hw);

  // Tight cluster of 60 degree / <111> Sigma3 twins.
  const size_t numTwins = 600;
  for(size_t i = 0; i < numTwins; i++)
  {
    kde.addMisorientation(quatFromAxisAngle(1.0, 1.0, 1.0, 60.0 * k_DegToRad), 1.0);
  }
  // Uniform-ish random background (deterministic) via Shoemake's method.
  std::mt19937 gen(12345);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  const size_t numRandom = 400;
  for(size_t i = 0; i < numRandom; i++)
  {
    const double u1 = uni(gen);
    const double u2 = uni(gen);
    const double u3 = uni(gen);
    const double s1 = std::sqrt(1.0 - u1);
    const double s2 = std::sqrt(u1);
    ebsdlib::QuatD q(s1 * std::sin(2.0 * constants::k_PiD * u2), s1 * std::cos(2.0 * constants::k_PiD * u2), s2 * std::sin(2.0 * constants::k_PiD * u3), s2 * std::cos(2.0 * constants::k_PiD * u3));
    kde.addMisorientation(q, 1.0);
  }
  kde.finalize();

  // 1. The MDF bin-array peak folds to a 60 degree / <111> misorientation.
  std::vector<double> densities = kde.evaluateAtBinCenters();
  size_t argMax = 0;
  for(size_t i = 1; i < densities.size(); i++)
  {
    if(densities[i] > densities[argMax])
    {
      argMax = i;
    }
  }
  double seed[3] = {0.5, 0.5, 0.5};
  ebsdlib::RodriguesDType peakRod = ops->determineRodriguesVector(seed, static_cast<int>(argMax));
  ebsdlib::AxisAngleDType peakAxisAngle = peakRod.toAxisAngle();
  const double peakAngleDeg = peakAxisAngle[3] / k_DegToRad;
  INFO("MDF peak angle (deg) = " << peakAngleDeg << " axis (" << peakAxisAngle[0] << ", " << peakAxisAngle[1] << ", " << peakAxisAngle[2] << ")");
  CHECK(peakAngleDeg > 56.0);
  CHECK(peakAngleDeg < 63.0);
  const double invSqrt3 = 1.0 / std::sqrt(3.0);
  CHECK(std::fabs(std::fabs(peakAxisAngle[0]) - invSqrt3) < 0.1);
  CHECK(std::fabs(std::fabs(peakAxisAngle[1]) - invSqrt3) < 0.1);
  CHECK(std::fabs(std::fabs(peakAxisAngle[2]) - invSqrt3) < 0.1);

  // 2. The angle-distribution curve peaks near 60 degrees, not at the ~45 degree random peak.
  ebsdlib::MisorientationKDE::AngleCurve curve = kde.computeAngleCurve(200);
  size_t curveArgMax = 0;
  for(size_t i = 1; i < curve.Density.size(); i++)
  {
    if(curve.Density[i] > curve.Density[curveArgMax])
    {
      curveArgMax = i;
    }
  }
  size_t randomArgMax = 0;
  for(size_t i = 1; i < curve.RandomDensity.size(); i++)
  {
    if(curve.RandomDensity[i] > curve.RandomDensity[randomArgMax])
    {
      randomArgMax = i;
    }
  }
  const double curvePeakDeg = curve.Angles[curveArgMax] / k_DegToRad;
  const double randomPeakDeg = curve.Angles[randomArgMax] / k_DegToRad;
  INFO("angle-curve peak (deg) = " << curvePeakDeg << ", random-reference peak (deg) = " << randomPeakDeg);
  CHECK(curvePeakDeg > 53.0);
  CHECK(curvePeakDeg < 63.0);
  // The random reference itself peaks near 45 degrees; the measured curve must be well above it.
  CHECK(curvePeakDeg - randomPeakDeg > 5.0);
}
