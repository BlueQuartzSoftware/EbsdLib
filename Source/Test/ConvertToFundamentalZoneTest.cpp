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

#include "EbsdLib/Core/OrientationRepresentation.h"
#include "EbsdLib/Core/OrientationTransformation.hpp"
#include "EbsdLib/Core/Quaternion.hpp"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

#include "TestPrintFunctions.h"
#include "UnitTestSupport.hpp"

#include <Eigen/Core>

#include <limits>

//clang-format off
namespace detail
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

} // namespace Detail

class ConvertToFundamentalZoneTest
{
public:
  ConvertToFundamentalZoneTest() = default;
  virtual ~ConvertToFundamentalZoneTest() = default;

  EBSD_GET_NAME_OF_CLASS_DECL(ConvertToFundamentalZoneTest)

  // -----------------------------------------------------------------------------
  void RemoveTestFiles()
  {
#if REMOVE_TEST_FILES
// fs::remove();
#endif
  }

  static OrientationD convertRodrigues(const std::array<double, 3>& rod)
  {
    const float length = sqrt(rod[0] * rod[0] + rod[1] * rod[1] + rod[2] * rod[2]);
    return {rod[0] / length, rod[1] / length, rod[2] / length, length};
  }

  void TestRodrigues()
  {
    auto ops = LaueOps::GetAllOrientationOps();
    std::cout << "############################################################\n";
    for(size_t opsIdx = 0; opsIdx < ops.size() - 1; ++opsIdx) // We ONLY want Cubic 432 rotation group
    {
      std::cout << "OpsIndex: " << opsIdx << "  " << ops[opsIdx]->getRotationPointGroup() << ", " << ops[opsIdx]->getSymmetryName() << ", " << ops[opsIdx]->getPointGroup() << ", "
                << ops[opsIdx]->FZTypeToString(ops[opsIdx]->getFZType()) << ", " << ops[opsIdx]->AxisOrderingTypeToString(ops[opsIdx]->getAxisOrderingType()) << std::endl;
      const std::array<bool, 22>& testValues = detail::k_FZValues[opsIdx];

      for(size_t testIdx = 0; testIdx < testValues.size(); testIdx++)
      {
        // OrientationD testRod = OrientationTransformation::qu2ro<QuatD, OrientationD>(Detail::k_InputQuat);
        std::array<double, 3> rod = detail::k_TestRodrigues[testIdx];
        OrientationD testRod = convertRodrigues(rod);
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
    DREAM3D_REGISTER_TEST(TestRodrigues())
  }

public:
  ConvertToFundamentalZoneTest(const ConvertToFundamentalZoneTest&) = delete;            // Copy Constructor Not Implemented
  ConvertToFundamentalZoneTest(ConvertToFundamentalZoneTest&&) = delete;                 // Move Constructor Not Implemented
  ConvertToFundamentalZoneTest& operator=(const ConvertToFundamentalZoneTest&) = delete; // Copy Assignment Not Implemented
  ConvertToFundamentalZoneTest& operator=(ConvertToFundamentalZoneTest&&) = delete;      // Move Assignment Not Implemented
};
