/* ============================================================================
 * Copyright (c) 2009-2016 BlueQuartz Software, LLC
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of BlueQuartz Software, the US Air Force, nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The code contained herein was partially funded by the following contracts:
 *    United States Air Force Prime Contract FA8650-07-D-5800
 *    United States Air Force Prime Contract FA8650-10-D-5210
 *    United States Prime Contract Navy N00173-07-C-2068
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include <limits>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>

#include "EbsdLib/Core/OrientationRepresentation.h"
#include "EbsdLib/Core/OrientationTransformation.hpp"
#include "EbsdLib/Core/Quaternion.hpp"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Math/EbsdMatrixMath.h"
#include "EbsdLib/Math/Matrix3X1.hpp"
#include "EbsdLib/Math/Matrix3X3.hpp"

#include "TestPrintFunctions.h"
#include "UnitTestSupport.hpp"

#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

//clang-format off
namespace Detail
{
std::vector<std::array<double, 3>> k_TestRodrigues = {{0.55, 0.55, 0.20},   {0.55, 0.55, 0.35},   {0.55, 0.55, 0.5},    {0.55, 0.55, 0.75},   {0.0, 0.0, 0.75},     {0.0, 0.0, 1.25},
                                                      {-0.25, -0.25, 0.20}, {-0.25, -0.25, 0.35}, {-0.25, -0.25, 0.49}, {-0.25, -0.25, 0.75}, {-0.25, -0.25, 0.51}, {0.0, -1.25, 0.20},
                                                      {0.0, -1.25, 0.35},   {0.0, -1.25, 0.5},    {0.0, -1.25, 0.75},   {0.0, -1.25, 1.25},   {0.15, 0.30, 0.0},    {0.3, 0.6, 0.0},
                                                      {0.375, 0.75, 0.0},   {0.45, 0.90, 0.0},    {0.4875, 0.975, 0.0}, {0.0, 1.1, 0.0}};

std::vector<std::array<bool, 22>> k_FZValues = {
    {true, false, false, false, false, false, true, false, false, false, false, false, false, false, false, false, true, true, true, false, false, false},   // 622
    {false, false, false, false, false, false, true, true, false, false, false, false, false, false, false, false, true, false, false, false, false, false}, // 432,
    {true, false, false, false, false, false, true, false, false, false, false, true, false, false, false, false, true, true, true, true, true, true},       // 6,
    {false, false, false, false, true, false, true, true, true, false, false, false, false, false, false, false, true, true, false, false, false, false},    // 23,
    {true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true},                    // 1
    {true, true, true, true, true, true, true, true, true, true, true, false, false, false, false, false, true, true, true, true, true, false},              // 2,
    {true, true, true, true, true, false, true, true, true, true, true, false, false, false, false, false, true, true, true, true, true, false},             // 222,
    {true, true, false, false, false, false, true, true, false, false, false, true, true, false, false, false, true, true, true, true, true, true},          // 4,
    {true, true, false, false, false, false, true, true, false, false, false, false, false, false, false, false, true, true, true, true, false, false},      // 422,
    {true, true, true, false, false, false, true, true, true, false, true, true, true, true, false, false, true, true, true, true, true, true},              // 3,
    {true, true, true, false, false, false, true, true, true, false, true, false, false, false, false, false, true, true, true, false, false, true}          // ,32,
};
//clang-format on

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

std::vector<std::array<double, 3>> k_TestAltRod = {

    {-0.411684, -0.199352, 0.045959}, {0.189712, -0.375260, 0.335709},  {0.188007, -0.369059, 0.332674},  {0.188926, -0.367251, 0.333663},  {-0.190728, -0.301796, 0.190883},
    {0.273214, -0.288990, 0.237019},  {0.275622, -0.287028, 0.238325},  {0.276140, -0.284900, 0.237770},  {0.269597, -0.287383, 0.236587},  {-0.411023, -0.201649, 0.040113},
    {-0.406870, -0.193782, 0.039497}, {-0.412621, -0.200390, 0.047397}, {-0.413234, -0.202985, 0.045914}, {-0.192546, -0.295416, 0.185587}, {-0.195329, -0.300516, 0.189205},
    {-0.196210, -0.307931, 0.192703}, {0.272988, -0.288760, 0.241910},  {0.272038, -0.283011, 0.239064},  {0.186567, -0.365406, 0.333646},  {-0.192765, -0.301071, 0.191329},
    {0.275968, -0.290669, 0.241222},  {0.272749, -0.280595, 0.234091},

};

} // namespace Detail

class ConvertToFundamentalZoneTest
{
public:
  ConvertToFundamentalZoneTest() = default;
  virtual ~ConvertToFundamentalZoneTest() = default;

  EBSD_GET_NAME_OF_CLASS_DECL(ConvertToFZTest)

  // -----------------------------------------------------------------------------
  void RemoveTestFiles()
  {
#if REMOVE_TEST_FILES
// fs::remove();
#endif
  }

  OrientationD ConvertRodrigues(const std::array<double, 3>& rod)
  {
    const float length = sqrt(rod[0] * rod[0] + rod[1] * rod[1] + rod[2] * rod[2]);
    return OrientationD(rod[0] / length, rod[1] / length, rod[2] / length, length);
  }

  void TestAltRodrigues()
  {
    auto ops = LaueOps::GetAllOrientationOps();

    {
      QuatD quat = Detail::k_TestQuats[0];
      std::cout << std::setprecision(12) << "Input Quat\n";
      OrientationPrinters::Print_QU(quat);
      QuatD quatNorm = quat_pos::makePositive(quat);
      std::cout << "Normalized Quat\n";
      OrientationPrinters::Print_QU(quatNorm);

      bool isInsideFZ = LaueOps::IsInsideFZ(quat, ops[1]->getFZType(), ops[1]->getAxisOrderingType());
      std::cout << std::setprecision(12) << "isInsideFZ: " << static_cast<bool>(isInsideFZ) << "\n";

      QuatD fzQuat = ops[1]->getFZQuat(quatNorm); // Cubic 432 FZ Quat
      std::cout << "FZ Quat\n";
      OrientationPrinters::Print_QU(fzQuat);

      QuatD FZQuatNorm = quat_pos::makePositive(fzQuat);
      std::cout << "FZ Quat Normalized\n";
      OrientationPrinters::Print_QU(FZQuatNorm);

      OrientationD quatToRod = OrientationTransformation::qu2ro<QuatD, OrientationD>(FZQuatNorm);
      std::cout << "FZ Quat Norm Converted to Rodrigues\n";
      OrientationPrinters::Print_RO(quatToRod);
      isInsideFZ = LaueOps::IsInsideFZ(quatToRod, ops[1]->getFZType(), ops[1]->getAxisOrderingType());
      std::cout << std::setprecision(12) << "isInsideFZ: " << static_cast<bool>(isInsideFZ) << "\n";
    }

    {
      std::cout << "Marc's Values\n";
      std::array<double, 3> marcRod = Detail::k_TestAltRod[0];
      OrientationPrinters::Print_RO3(marcRod);
      OrientationD testRod = ConvertRodrigues(marcRod);
      OrientationPrinters::Print_RO(testRod);

      bool isInsideFZ = LaueOps::IsInsideFZ(testRod, ops[1]->getFZType(), ops[1]->getAxisOrderingType());
      std::cout << std::setprecision(12) << "isInsideFZ: " << static_cast<bool>(isInsideFZ) << "\n";
    }

    for(size_t opsIdx = 1; opsIdx < 2; ++opsIdx)
    {
      std::cout << "############################################################\n";
      std::cout << ops[opsIdx]->getRotationPointGroup() << ", " << ops[opsIdx]->getSymmetryName() << ", " << ops[opsIdx]->getPointGroup() << ", "
                << ops[opsIdx]->FZTypeToString(ops[opsIdx]->getFZType()) << ", " << ops[opsIdx]->AxisOrderingTypeToString(ops[opsIdx]->getAxisOrderingType()) << std::endl;
      // const std::array<bool, 22>& testValues = Detail::k_FZValues[opsIdx];

      for(size_t testIdx = 0; testIdx < Detail::k_TestAltRod.size(); testIdx++)
      {
        bool isInside = LaueOps::IsInsideFZ(Detail::k_TestQuats[testIdx], ops[opsIdx]->getFZType(), ops[opsIdx]->getAxisOrderingType());

        // DREAM3D_REQUIRE_EQUAL(isInside, true)
        //   if(testValues[testIdx] != isInside)
        //   {
        //  std::stringstream ss;
        //  ss << testIdx << ": "
        //     << "(" << rod[0] << ", " << rod[1] << ", " << rod[2] << "), " << isInside;
        //  std::cout << ss.str() << std::endl;
        //  }

        if(!isInside)
        {
          QuatD fzQuat = ops[opsIdx]->getFZQuat(Detail::k_TestQuats[testIdx]);
          isInside = LaueOps::IsInsideFZ(fzQuat, ops[opsIdx]->getFZType(), ops[opsIdx]->getAxisOrderingType());
          DREAM3D_REQUIRE_EQUAL(isInside, true);
        }
      }
    }
  }

  void TestQuats()
  {
    auto ops = LaueOps::GetAllOrientationOps();

    for(size_t opsIdx = 1; opsIdx < 2; ++opsIdx)
    {
      std::cout << "############################################################\n";
      std::cout << ops[opsIdx]->getRotationPointGroup() << ", " << ops[opsIdx]->getSymmetryName() << ", " << ops[opsIdx]->getPointGroup() << ", "
                << ops[opsIdx]->FZTypeToString(ops[opsIdx]->getFZType()) << ", " << ops[opsIdx]->AxisOrderingTypeToString(ops[opsIdx]->getAxisOrderingType()) << std::endl;
      const std::array<bool, 22>& testValues = Detail::k_FZValues[opsIdx];

      for(const QuatD& quat : Detail::k_TestQuats)
      {
        OrientationD rod = OrientationTransformation::qu2ro<QuatD, OrientationD>(quat.normalize());
        bool isInside = LaueOps::IsInsideFZ(rod, ops[opsIdx]->getFZType(), ops[opsIdx]->getAxisOrderingType());

        DREAM3D_REQUIRE_EQUAL(isInside, false)
        // if(testValues[testIdx] != isInside)
        // {
        //   std::stringstream ss;
        //   ss << testIdx << ": " << "(" << rod[0] << ", " << rod[1] << ", " << rod[2] << "), " << testValues[testIdx] << ", " << isInside;
        //   std::cout << ss.str() << std::endl;
        // }

        if(!isInside)
        {
          QuatD fzQuat = ops[opsIdx]->getFZQuat(quat.normalize());
          DREAM3D_REQUIRE(fzQuat[0] != std::numeric_limits<double>::infinity())
          OrientationD fzRod = OrientationTransformation::qu2ro<QuatD, OrientationD>(fzQuat);
          isInside = LaueOps::IsInsideFZ(fzRod, ops[opsIdx]->getFZType(), ops[opsIdx]->getAxisOrderingType());
          DREAM3D_REQUIRE_EQUAL(isInside, true);
        }
      }
    }
  }

  void TestRodrigues()
  {
    auto ops = LaueOps::GetAllOrientationOps();

    for(size_t opsIdx = 0; opsIdx < ops.size() - 1; ++opsIdx) // We ONLY want Cubic 432 rotation group
    {
      std::cout << "############################################################\n";
      std::cout << "OpsIndex: " << opsIdx << "  " << ops[opsIdx]->getRotationPointGroup() << ", " << ops[opsIdx]->getSymmetryName() << ", " << ops[opsIdx]->getPointGroup() << ", "
                << ops[opsIdx]->FZTypeToString(ops[opsIdx]->getFZType()) << ", " << ops[opsIdx]->AxisOrderingTypeToString(ops[opsIdx]->getAxisOrderingType()) << std::endl;
      const std::array<bool, 22>& testValues = Detail::k_FZValues[opsIdx];

      for(size_t testIdx = 0; testIdx < testValues.size(); testIdx++)
      {
        // OrientationD testRod = OrientationTransformation::qu2ro<QuatD, OrientationD>(Detail::k_InputQuat);
        std::array<double, 3> rod = Detail::k_TestRodrigues[testIdx];
        OrientationD testRod = ConvertRodrigues(rod);
        bool isInside = LaueOps::IsInsideFZ(testRod, ops[opsIdx]->getFZType(), ops[opsIdx]->getAxisOrderingType());

        DREAM3D_REQUIRE_EQUAL(isInside, testValues[testIdx])
        // if(testValues[testIdx] != isInside)
        // {
        //   std::stringstream ss;
        //   ss << testIdx << ": " << "(" << rod[0] << ", " << rod[1] << ", " << rod[2] << "), " << testValues[testIdx] << ", " << isInside;
        //   std::cout << ss.str() << std::endl;
        // }

        if(!isInside)
        {
          QuatD quat = OrientationTransformation::ro2qu<OrientationD, QuatD>(testRod);
          QuatD fzQuat = ops[opsIdx]->getFZQuat(quat);
          OrientationD fzRod = OrientationTransformation::qu2ro<QuatD, OrientationD>(fzQuat);
          isInside = LaueOps::IsInsideFZ(fzRod, ops[opsIdx]->getFZType(), ops[opsIdx]->getAxisOrderingType());
          DREAM3D_REQUIRE_EQUAL(isInside, true);
        }
      }
    }
  }

  // -----------------------------------------------------------------------------
  void operator()()
  {
    std::cout << "<===== Start " << getNameOfClass() << std::endl;
    int err = EXIT_SUCCESS;
    // DREAM3D_REGISTER_TEST(TestAltRodrigues())
    DREAM3D_REGISTER_TEST(TestQuats())
    DREAM3D_REGISTER_TEST(TestRodrigues())
  }

public:
  ConvertToFundamentalZoneTest(const ConvertToFundamentalZoneTest&) = delete;            // Copy Constructor Not Implemented
  ConvertToFundamentalZoneTest(ConvertToFundamentalZoneTest&&) = delete;                 // Move Constructor Not Implemented
  ConvertToFundamentalZoneTest& operator=(const ConvertToFundamentalZoneTest&) = delete; // Copy Assignment Not Implemented
  ConvertToFundamentalZoneTest& operator=(ConvertToFundamentalZoneTest&&) = delete;      // Move Assignment Not Implemented
};
