#include <catch2/catch.hpp>



#include "EbsdLib/Core/DirectionalStats.hpp"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Orientation/Quaternion.hpp"

#include <fmt/format.h>

#include "UnitTestSupport.hpp"

#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

#include <cmath>
#include <cstdio>
#include <limits>

using namespace ebsdlib;

//clang-format off
namespace detail
{
std::vector<QuatD> k_TestQuats = {
    {0.6719963424253053, 0.6129423860730265, 0.1364166710960659, 0.3925723359703083},    {0.295631110116223, 0.8806176765212745, -0.1670640703896814, 0.330460816004792},
    {0.293789811168501, 0.8831169225237752, -0.1660320799434538, 0.3259223779297207},    {0.2947005642746592, 0.8832293192391356, -0.1668647237149576, 0.3243666305773475},
    {0.1769340339862296, 0.9269220251506355, 0.1767900887688091, 0.2797412579887368},    {-0.4895854071041807, 0.7937642457108052, 0.0101229019146293, 0.3607519622103487},
    {-0.4885846329994489, 0.7943361326081295, 0.007316124555987299, 0.3609177733936573}, {-0.4891812284025829, 0.7943711408640211, 0.005622187254144007, 0.3600619493245755},
    {-0.4904884701659861, 0.7944995174028157, 0.01142771077728101, 0.3578560952496329},  {0.6682755620419383, 0.6167302380229949, 0.1345230991224496, 0.3936433950774778},
    {0.669694655986085, 0.6188030782527685, 0.1372821020479966, 0.3869695628158121},     {0.6725537285776072, 0.6116846895234272, 0.1362780204804377, 0.3936262490141125},
    {0.6712100069793456, 0.6122801449826168, 0.1349260392167796, 0.3954555784561961},    {0.1724037140729195, 0.9289624416784542, 0.1788682254529524, 0.27443013545888},
    {0.1753504855620956, 0.9267758042757149, 0.1810262782021287, 0.2785108658966909},    {0.1781196317732192, 0.9243210069735922, 0.1813607833836466, 0.284626666169506},
    {-0.4860300998793424, 0.7962186305444112, 0.01011143900233245, 0.3601505146276495},  {-0.4888952933345148, 0.7960889874984892, 0.007050530465747282, 0.3566146465852751},
    {0.2949445932779656, 0.8840049176186158, -0.16492632529573, 0.3230205871870195},     {0.1773081200034897, 0.9267173624983174, 0.1786384658179278, 0.2790072743768655},
    {-0.4859900547906299, 0.7949907303647343, 0.009415733770101149, 0.3629252667372687}, {-0.4927679779186163, 0.7939850383340995, 0.005047892871246353, 0.3560084235199459}};

}
//clang-format on

// Port of the Fortran orav_ subroutine from mod_orav.f90
// Tests VMF and Watson directional statistics averaging
TEST_CASE("DirectionalStatsTest:VMF", "[DirectionalStatsTest]")
{
  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
  LaueOps::Pointer cubicOps = ops[1]; // Cubic High

  // Reduce input quaternions to the Rodrigues Fundamental Zone
  // (mirrors Fortran: call SO%ReducelisttoRFZ(qsym) in mod_orav.f90 line 315)
  std::vector<QuatD> fzQuats;
  fzQuats.reserve(detail::k_TestQuats.size());
  for(const auto& q : detail::k_TestQuats)
  {
   fzQuats.push_back(cubicOps->getFZQuat(q));
  }

  // VMF averaging (mirrors Fortran: dictVMF = DirStat_T(DStype='VMF', pgnum=pgnum))
  DirectionalStats dictVMF("VMF", cubicOps);
  int numEmIterations = 5;
  int numIterations = 10;
  dictVMF.setNumEM(numEmIterations);
  dictVMF.setNumIter(numIterations);
  dictVMF.setQuatArray(fzQuats);

  uint32_t seed = 43514;
  QuatD muhat = QuatD::identity();
  double kappahat = 0.0;

  dictVMF.EMforDS(seed, muhat, kappahat, false);

  constexpr double k_Pi = 3.141592653589793238462643383279502884;
  double eqDeg = 180.0 * std::acos(1.0 - 1.0 / kappahat) / k_Pi;

  std::printf(" Quaternion von Mises-Fisher average\n");
  std::printf(" num EM Iterations: %d\n", numEmIterations);
  std::printf(" num Iterations: %d\n", numIterations);
  std::printf(" <q> wxyz     : %20.16f %20.16f %20.16f %20.16f\n", muhat.w(), muhat.x(), muhat.y(), muhat.z());
  std::printf(" kappa    : %20.16f\n", kappahat);
  std::printf(" eq. deg. : %20.16f\n", eqDeg);

  REQUIRE(muhat.w() == Approx(0.8893749825279105));
  REQUIRE(muhat.x() == Approx(0.3322000547718371));
  REQUIRE(muhat.y() == Approx(-0.1964639452260062));
  REQUIRE(muhat.z() == Approx(0.2450656693404858));
  REQUIRE(kappahat == Approx(88.9943042750539774));
  REQUIRE(eqDeg == Approx(8.5973386361977155));
}

TEST_CASE("DirectionalStatsTest:Watson", "[DirectionalStatsTest]")
{
  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
  LaueOps::Pointer cubicOps = ops[1]; // Cubic High

  // Reduce input quaternions to the Rodrigues Fundamental Zone
  std::vector<QuatD> fzQuats;
  fzQuats.reserve(detail::k_TestQuats.size());
  for(const auto& q : detail::k_TestQuats)
  {
    fzQuats.push_back(cubicOps->getFZQuat(q));
  }

  // Watson averaging (mirrors Fortran: dictWAT = DirStat_T(DStype='WAT', pgnum=pgnum))
  DirectionalStats dictWAT("WAT", cubicOps);
  int numEmIterations = 5;
  int numIterations = 10;

  dictWAT.setNumEM(numEmIterations);
  dictWAT.setNumIter(numIterations);
  dictWAT.setQuatArray(fzQuats);

  uint32_t seed = 43514;
  QuatD muhat = QuatD::identity();
  double kappahat = 0.0;

  dictWAT.EMforDS(seed, muhat, kappahat, false);

  constexpr double k_Pi = 3.141592653589793238462643383279502884;
  double eqDeg = 180.0 * std::acos(1.0 - 1.0 / kappahat) / k_Pi;

  std::printf(" Quaternion Watson average\n");

  std::printf(" num EM Iterations: %d\n", numEmIterations);
  std::printf(" num Iterations: %d\n", numIterations);
  std::printf(" <q>wxyz      : %20.16f %20.16f %20.16f %20.16f\n", muhat.w(), muhat.x(), muhat.y(), muhat.z());
  std::printf(" kappa    : %20.16f\n", kappahat);
  std::printf(" eq. deg. : %20.16f\n", eqDeg);

  REQUIRE(muhat.w() == Approx(0.9011878668560466));
  REQUIRE(muhat.x() == Approx(0.2948298270586034));
  REQUIRE(muhat.y() == Approx(-0.2106011604618418));
  REQUIRE(muhat.z() == Approx(0.2378717152588106));
  REQUIRE(kappahat == Approx(30.5730272919979669));
  REQUIRE(eqDeg == Approx(14.6946529653613620));
}

// TEST_CASE("DirectionalStatsTest:SpaceGroupTest", "[DirectionalStatsTest]")
// {
//   for(size_t sgNum = 1; sgNum <= 230; ++sgNum)
//   {
//     auto ops = LaueOps::GetOrientationOpsFromSpaceGroupNumber(sgNum);
//   }
// }
