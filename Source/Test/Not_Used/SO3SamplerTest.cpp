/* ============================================================================
 * Copyright (c) 2015 BlueQuartz Software, LLC
 * All rights reserved.
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
 * Neither the names of any of the BlueQuartz Software contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
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
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include "EbsdLib/LaueOps/SO3Sampler.h"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Orientation/OrientationFwd.hpp"
#include "EbsdLib/Orientation/Quaternion.hpp"
#include "EbsdLib/Utilities/ModifiedLambertProjection3D.hpp"

#include "UnitTestSupport.hpp"

#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

#include <random>

class SO3SamplerTest
{
public:
  SO3SamplerTest() = default;
  virtual ~SO3SamplerTest() = default;

  SO3SamplerTest(const SO3SamplerTest&) = delete;            // Copy Constructor Not Implemented
  SO3SamplerTest(SO3SamplerTest&&) = delete;                 // Move Constructor Not Implemented
  SO3SamplerTest& operator=(const SO3SamplerTest&) = delete; // Copy Assignment Not Implemented
  SO3SamplerTest& operator=(SO3SamplerTest&&) = delete;      // Move Assignment Not Implemented

  EBSD_GET_NAME_OF_CLASS_DECL(SO3SamplerTest)

  // -----------------------------------------------------------------------------
  void RemoveTestFiles()
  {
#if REMOVE_TEST_FILES
// fs::remove();
#endif
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void SO3CountTest()
  {
    SO3Sampler::Pointer sampler = SO3Sampler::New();
    SO3Sampler::OrientationListArrayType orientations = sampler->SampleRFZ(10, 32);
    DREAM3D_REQUIRE_EQUAL(361, orientations.size());

    orientations = sampler->SampleRFZ(100, 32);
    DREAM3D_REQUIRE_EQUAL(333227, orientations.size());
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void InsideCubicFZTest()
  {

    SO3Sampler::Pointer sampler = SO3Sampler::New();
    RodriguesDType rod;

    CubochoricDType cu(-0.3217544095666538, 0.2145029397111025, -0.4290058794222050);
    rod = cu.toRodrigues();
    bool inside = sampler->insideCubicFZ(rod, 4);
    DREAM3D_REQUIRE_EQUAL(inside, false);

    cu = CubochoricDType(-0.42900587942220514, -0.21450293971110265, 0.42900587942220514);
    rod = cu.toRodrigues();
    inside = sampler->insideCubicFZ(rod, 4);
    DREAM3D_REQUIRE_EQUAL(inside, true);
  }

  // -----------------------------------------------------------------------------
  //
  // -----------------------------------------------------------------------------
  void TestPyramid()
  {

    std::array<double, 3> xyz = {0.0, 0.0, 1.0};

    int pSection = ModifiedLambertProjection3D<std::array<double, 3>, double>::GetPyramid(xyz);
    DREAM3D_REQUIRE_EQUAL(pSection, 1)

    xyz = {0.0, 0.0, -1.0};
    pSection = ModifiedLambertProjection3D<std::array<double, 3>, double>::GetPyramid(xyz);
    DREAM3D_REQUIRE_EQUAL(pSection, 2)

    xyz = {1.0, 0.0, 0.0};
    pSection = ModifiedLambertProjection3D<std::array<double, 3>, double>::GetPyramid(xyz);
    DREAM3D_REQUIRE_EQUAL(pSection, 3)

    xyz = {-1.0, 0.0, 0.0};
    pSection = ModifiedLambertProjection3D<std::array<double, 3>, double>::GetPyramid(xyz);
    DREAM3D_REQUIRE_EQUAL(pSection, 4)

    xyz = {0.0, 1.0, 0.0};
    pSection = ModifiedLambertProjection3D<std::array<double, 3>, double>::GetPyramid(xyz);
    DREAM3D_REQUIRE_EQUAL(pSection, 5)

    xyz = {0.0, -1.0, 0.0};
    pSection = ModifiedLambertProjection3D<std::array<double, 3>, double>::GetPyramid(xyz);
    DREAM3D_REQUIRE_EQUAL(pSection, 6)
  }

  void operator()()
  {
    std::cout << "<===== Start " << getNameOfClass() << std::endl;

    int err = EXIT_SUCCESS;
    DREAM3D_REGISTER_TEST(InsideCubicFZTest())
    DREAM3D_REGISTER_TEST(TestPyramid())
    DREAM3D_REGISTER_TEST(SO3CountTest())
    DREAM3D_REGISTER_TEST(RemoveTestFiles())
  }
};
