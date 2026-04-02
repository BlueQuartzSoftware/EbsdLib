#include <catch2/catch.hpp>

#include "EbsdLib/Core/DirectionalStats.hpp"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Orientation/Quaternion.hpp"
#include "EbsdLib/Test/EbsdLibTestFileLocations.h"
#include "UnitTestCommon.hpp"

#include <H5Support/H5Lite.h>
#include <H5Support/H5ScopedSentinel.h>
#include <H5Support/H5Utilities.h>

#include <fmt/format.h>

#include "UnitTestSupport.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <set>
#include <sstream>
#include <string>

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

void TestDistribution(const std::string& phaseName, LaueOps::Pointer op, const std::string& distributionType)
{
  constexpr size_t k_NumSamplingGroups = 8;
  constexpr size_t k_NumQuats = 10000;
  constexpr size_t k_QuatSize = 4;

  std::string inputFilePath = fmt::format("{}/Laue_Orientation_Clusters_v6/{}.h5", ebsdlib::unit_test::k_TestFilesDir, phaseName);
  hid_t fid = H5Support::H5Utilities::openFile(inputFilePath, true);
  REQUIRE(fid > 0);
  H5Support::H5ScopedFileSentinel fileSentinel(fid, false);

  // Read averaging parameters from HDF5
  int32_t numEmIterations = 0;
  herr_t err = H5Support::H5Lite::readScalarDataset(fid, "/EMData/Sampler/NumEM", numEmIterations);
  REQUIRE(err == 0);

  int32_t numIterations = 0;
  err = H5Support::H5Lite::readScalarDataset(fid, "/EMData/Sampler/NumIter", numIterations);
  REQUIRE(err == 0);

  // Read the seedarray: shape (2, 2, 8) in Fortran column-major
  // seedarray(seedIdx, distType, groupIdx) where seedIdx=1,2 distType=1(VMF),2(WAT) groupIdx=1..8
  std::vector<int32_t> seedarray;
  err = H5Support::H5Lite::readVectorDataset(fid, "/EMData/Sampler/seedarray", seedarray);
  REQUIRE(err == 0);

  // Read reference data: prefix is "vMF" or "WAT"
  std::string prefix = (distributionType == "VMF") ? "vMF" : "WAT";
  std::vector<double> refMuhat;
  err = H5Support::H5Lite::readVectorDataset(fid, fmt::format("/EMData/Sampler/{}muhat", prefix), refMuhat);
  REQUIRE(err == 0);

  std::vector<double> refKappahat;
  err = H5Support::H5Lite::readVectorDataset(fid, fmt::format("/EMData/Sampler/{}kappahat", prefix), refKappahat);
  REQUIRE(err == 0);

  std::vector<double> quatarray;
  err = H5Support::H5Lite::readVectorDataset(fid, fmt::format("/EMData/Sampler/{}quatarray", prefix), quatarray);
  REQUIRE(err == 0);

  // distType index for seedarray: VMF=0, WAT=1 (Fortran 1-based: 1,2)
  int distTypeIdx = (distributionType == "VMF") ? 0 : 1;

  for(size_t sampleId = 0; sampleId < k_NumSamplingGroups; ++sampleId)
  {
    // Fill FZ-corrected quaternion array
    std::vector<QuatD> fzQuats;
    fzQuats.reserve(k_NumQuats);
    for(size_t quatIdx = 0; quatIdx < k_NumQuats; ++quatIdx)
    {
      // HDF5 stores quaternions as WXYZ (EMsoft), convert to XYZW (EbsdLib)
      size_t idx = (sampleId * k_NumQuats * k_QuatSize) + (quatIdx * k_QuatSize);
      QuatD q(quatarray[idx + 1], quatarray[idx + 2], quatarray[idx + 3], quatarray[idx]);
      fzQuats.push_back(op->getFZQuat(q));
    }

    DirectionalStats dict(distributionType, op);
    dict.setNumEM(numEmIterations);
    dict.setNumIter(numIterations);
    dict.setQuatArray(fzQuats);

    // Read the seed for this specific group from seedarray
    // Fortran column-major (2, 2, 8): index = seedIdx + 2*distTypeIdx + 4*sampleId
    // seed2 is at seedIdx=1 (0-based)
    uint32_t seed = static_cast<uint32_t>(seedarray[1 + 2 * distTypeIdx + 4 * sampleId]);
    QuatD muhat = QuatD::identity();
    double kappahat = 0.0;

    dict.EMforDS(seed, muhat, kappahat, false);
    muhat = muhat.normalize();

    // Reference from HDF5 (WXYZ -> XYZW conversion)
    QuatD refMu = QuatD(refMuhat[sampleId * 4 + 1], refMuhat[sampleId * 4 + 2], refMuhat[sampleId * 4 + 3], refMuhat[sampleId * 4]).normalize();
    double refKappa = refKappahat[sampleId];

    std::printf("  %s group %zu: kappa EbsdLib=%12.6f EMsoft=%12.6f  muW EbsdLib=%10.7f EMsoft=%10.7f\n", distributionType.c_str(), sampleId, kappahat, refKappa, muhat.w(), refMu.w());

    REQUIRE(muhat.w() == Approx(refMu.w()).margin(1e-6));
    REQUIRE(muhat.x() == Approx(refMu.x()).margin(1e-6));
    REQUIRE(muhat.y() == Approx(refMu.y()).margin(1e-6));
    REQUIRE(muhat.z() == Approx(refMu.z()).margin(1e-6));
    REQUIRE(kappahat == Approx(refKappa).epsilon(1e-4));
  }
}

TEST_CASE("DirectionalStatsTest:AverageOrientation", "[DirectionalStatsTest]")
{
  const ebsdlib::unit_test::TestFileSentinel testDataSentinel(ebsdlib::unit_test::k_TestFilesDir, "Laue_Orientation_Clusters_v6.tar.gz", "Laue_Orientation_Clusters_v6", true, true);
  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();

  std::set<std::string> tested;
  for(const auto& op : ops)
  {
    const std::string rpg = op->getRotationPointGroup();
    // Skip Triclinic (no FZ boundary) and duplicates (OrthoRhombicOps appears twice)
    if(rpg == "1" || tested.count(rpg) > 0)
    {
      continue;
    }
    tested.insert(rpg);

    const std::string phaseName = fmt::format("Laue_{}", rpg);
    SECTION(phaseName + " WAT")
    {
      TestDistribution(phaseName, op, "WAT");
    }
    SECTION(phaseName + " VMF")
    {
      TestDistribution(phaseName, op, "VMF");
    }
  }
}

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

namespace detail
{
/**
 * @brief Reads a text file of quaternions in WXYZ order (EMsoft format) and
 * returns them as QuatD objects (XYZW order).
 *
 * Expected file format:
 *   qu                    (orientation type header)
 *   <num_orientations>    (integer count)
 *   w x y z              (space-separated, one per line)
 *   ...
 */
std::vector<QuatD> readQuatsFromFile(const std::string& filePath)
{
  std::vector<QuatD> quats;
  std::ifstream inFile(filePath);
  REQUIRE(inFile.is_open());

  // Read orientation type header (e.g., "qu")
  std::string angleMode;
  inFile >> angleMode;
  REQUIRE(angleMode == "qu");

  // Read number of orientations
  int numOrientations = 0;
  inFile >> numOrientations;
  REQUIRE(numOrientations > 0);

  quats.reserve(numOrientations);
  for(int i = 0; i < numOrientations; ++i)
  {
    double w = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    inFile >> w >> x >> y >> z;

    // File is WXYZ, QuatD constructor is XYZW; normalize to ensure unit quaternion
    quats.push_back(QuatD(x, y, z, w).normalize());
  }

  return quats;
}
} // namespace detail

TEST_CASE("DirectionalStatsTest:VMF_FromCSV", "[DirectionalStatsTest]")
{
  std::string csvPath = ebsdlib::unit_test::DirectionalStatsTest::QuatsWXYZ_9260_File;
  std::vector<QuatD> inputQuats = detail::readQuatsFromFile(csvPath);
  REQUIRE(inputQuats.size() == 9260);

  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
  LaueOps::Pointer cubicOps = ops[1]; // Cubic High

  // Reduce input quaternions to the Rodrigues Fundamental Zone
  std::vector<QuatD> fzQuats;
  fzQuats.reserve(inputQuats.size());
  for(const auto& q : inputQuats)
  {
    fzQuats.push_back(cubicOps->getFZQuat(q));
  }

  // VMF averaging
  DirectionalStats dictVMF("VMF", cubicOps);
  int numEmIterations = 5;
  int numIterations = 10;
  dictVMF.setNumEM(numEmIterations);
  dictVMF.setNumIter(numIterations);
  dictVMF.setQuatArray(fzQuats);

  uint32_t seed = 43514;
  QuatD muhat = QuatD::identity();
  double kappahat = 0.0;

  dictVMF.EMforDS(seed, muhat, kappahat, true);

  constexpr double k_Pi = 3.141592653589793238462643383279502884;
  double eqDeg = 180.0 * std::acos(1.0 - 1.0 / kappahat) / k_Pi;

  std::printf(" Quaternion VMF average (CSV input, %zu quats)\n", inputQuats.size());
  std::printf(" num EM Iterations: %d\n", numEmIterations);
  std::printf(" num Iterations: %d\n", numIterations);
  std::printf(" <q> wxyz     : %20.16f %20.16f %20.16f %20.16f\n", muhat.w(), muhat.x(), muhat.y(), muhat.z());
  std::printf(" kappa    : %20.16f\n", kappahat);
  std::printf(" eq. deg. : %20.16f\n", eqDeg);

  // EMsoftOO reference values:
  // best fit Mu (WXYZ, RFZ-reduced): 0.92397289 0.38244926 0.00091773 0.00241359
  // best fit kappa: 589.79047395
}

TEST_CASE("DirectionalStatsTest:Watson_FromCSV", "[DirectionalStatsTest]")
{
  std::string csvPath = ebsdlib::unit_test::DirectionalStatsTest::QuatsWXYZ_9260_File;
  std::vector<QuatD> inputQuats = detail::readQuatsFromFile(csvPath);
  REQUIRE(inputQuats.size() == 9260);

  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
  LaueOps::Pointer cubicOps = ops[1]; // Cubic High

  // Reduce input quaternions to the Rodrigues Fundamental Zone
  std::vector<QuatD> fzQuats;
  fzQuats.reserve(inputQuats.size());
  for(const auto& q : inputQuats)
  {
    fzQuats.push_back(cubicOps->getFZQuat(q));
  }

  // Watson averaging
  DirectionalStats dictWAT("WAT", cubicOps);
  int numEmIterations = 5;
  int numIterations = 10;
  dictWAT.setNumEM(numEmIterations);
  dictWAT.setNumIter(numIterations);
  dictWAT.setQuatArray(fzQuats);

  uint32_t seed = 43514;
  QuatD muhat = QuatD::identity();
  double kappahat = 0.0;

  dictWAT.EMforDS(seed, muhat, kappahat, true);

  constexpr double k_Pi = 3.141592653589793238462643383279502884;
  double eqDeg = 180.0 * std::acos(1.0 - 1.0 / kappahat) / k_Pi;

  std::printf(" Quaternion Watson average (CSV input, %zu quats)\n", inputQuats.size());
  std::printf(" num EM Iterations: %d\n", numEmIterations);
  std::printf(" num Iterations: %d\n", numIterations);
  std::printf(" <q> wxyz     : %20.16f %20.16f %20.16f %20.16f\n", muhat.w(), muhat.x(), muhat.y(), muhat.z());
  std::printf(" kappa    : %20.16f\n", kappahat);
  std::printf(" eq. deg. : %20.16f\n", eqDeg);

  // EMsoftOO reference values:
  // best fit Mu (WXYZ, RFZ-reduced): 0.92395702 0.38249445 0.00041948 0.00111833
  // best fit kappa: 315.60762339
}

TEST_CASE("DirectionalStatsTest:VMF_FromTXT", "[DirectionalStatsTest]")
{
  std::string txtPath = ebsdlib::unit_test::DirectionalStatsTest::QuatsWXYZ_29791_File;
  std::vector<QuatD> inputQuats = detail::readQuatsFromFile(txtPath);
  REQUIRE(inputQuats.size() == 29791);

  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
  LaueOps::Pointer cubicOps = ops[1]; // Cubic High

  // Reduce input quaternions to the Rodrigues Fundamental Zone
  std::vector<QuatD> fzQuats;
  fzQuats.reserve(inputQuats.size());
  for(const auto& q : inputQuats)
  {
    fzQuats.push_back(cubicOps->getFZQuat(q));
  }

  // VMF averaging
  DirectionalStats dictVMF("VMF", cubicOps);
  int numEmIterations = 5;
  int numIterations = 10;
  dictVMF.setNumEM(numEmIterations);
  dictVMF.setNumIter(numIterations);
  dictVMF.setQuatArray(fzQuats);

  uint32_t seed = 43514;
  QuatD muhat = QuatD::identity();
  double kappahat = 0.0;

  dictVMF.EMforDS(seed, muhat, kappahat, true);

  constexpr double k_Pi = 3.141592653589793238462643383279502884;
  double eqDeg = 180.0 * std::acos(1.0 - 1.0 / kappahat) / k_Pi;

  std::printf(" Quaternion VMF average (TXT input, %zu quats)\n", inputQuats.size());
  std::printf(" num EM Iterations: %d\n", numEmIterations);
  std::printf(" num Iterations: %d\n", numIterations);
  std::printf(" <q> wxyz     : %20.16f %20.16f %20.16f %20.16f\n", muhat.w(), muhat.x(), muhat.y(), muhat.z());
  std::printf(" kappa    : %20.16f\n", kappahat);
  std::printf(" eq. deg. : %20.16f\n", eqDeg);
}

TEST_CASE("DirectionalStatsTest:Watson_FromTXT", "[DirectionalStatsTest]")
{
  std::string txtPath = ebsdlib::unit_test::DirectionalStatsTest::QuatsWXYZ_29791_File;
  std::vector<QuatD> inputQuats = detail::readQuatsFromFile(txtPath);
  REQUIRE(inputQuats.size() == 29791);

  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
  LaueOps::Pointer cubicOps = ops[1]; // Cubic High

  // Reduce input quaternions to the Rodrigues Fundamental Zone
  std::vector<QuatD> fzQuats;
  fzQuats.reserve(inputQuats.size());
  for(const auto& q : inputQuats)
  {
    fzQuats.push_back(cubicOps->getFZQuat(q));
  }

  // Watson averaging
  DirectionalStats dictWAT("WAT", cubicOps);
  int numEmIterations = 5;
  int numIterations = 10;
  dictWAT.setNumEM(numEmIterations);
  dictWAT.setNumIter(numIterations);
  dictWAT.setQuatArray(fzQuats);

  uint32_t seed = 43514;
  QuatD muhat = QuatD::identity();
  double kappahat = 0.0;

  dictWAT.EMforDS(seed, muhat, kappahat, true);

  constexpr double k_Pi = 3.141592653589793238462643383279502884;
  double eqDeg = 180.0 * std::acos(1.0 - 1.0 / kappahat) / k_Pi;

  std::printf(" Quaternion Watson average (TXT input, %zu quats)\n", inputQuats.size());
  std::printf(" num EM Iterations: %d\n", numEmIterations);
  std::printf(" num Iterations: %d\n", numIterations);
  std::printf(" <q> wxyz     : %20.16f %20.16f %20.16f %20.16f\n", muhat.w(), muhat.x(), muhat.y(), muhat.z());
  std::printf(" kappa    : %20.16f\n", kappahat);
  std::printf(" eq. deg. : %20.16f\n", eqDeg);
}
