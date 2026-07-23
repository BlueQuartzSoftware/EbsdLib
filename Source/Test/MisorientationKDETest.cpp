#include <catch2/catch.hpp>

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Orientation/Quaternion.hpp"
#include "EbsdLib/Texture/MisorientationKDE.h"
#include "EbsdLib/Texture/SO3DeLaValleePoussinKernel.h"

#include <array>
#include <cmath>
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

// Disorientation-style similarity of two densities.
} // namespace

// -----------------------------------------------------------------------------
// Triclinic has a single (identity) symmetry operator, so the crystal-symmetry
// average is trivial and the kernel density around one isolated misorientation
// reduces to the raw De la Vallee Poussin kernel. Its modal height is the kernel
// constant K(0) = psi.evaluate(1.0), it falls to half a halfwidth away, and it is
// exactly zero beyond the cutoff.
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

  // Modal peak: density at the snapped center equals the kernel constant K(0).
  REQUIRE(kde.evaluate(snapped) == Approx(psi.evaluate(1.0)).epsilon(0.01));

  // Rotate the snapped center by hw about an orthogonal axis -> half peak.
  ebsdlib::QuatD dq(std::sin(hw / 2.0), 0.0, 0.0, std::cos(hw / 2.0));
  REQUIRE(kde.evaluate(dq * snapped) == Approx(psi.evaluate(1.0) / 2.0).epsilon(0.02));

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
