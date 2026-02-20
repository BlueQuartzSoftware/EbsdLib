/* ============================================================================
 * Copyright (c) 2009-2025 BlueQuartz Software, LLC
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
#include <catch2/catch.hpp>

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/LaueOps/CubicOps.h"
#include "EbsdLib/Orientation/Euler.hpp"
#include "EbsdLib/Orientation/Quaternion.hpp"
#include "EbsdLib/OrientationMath/OrientationConverter.hpp"

#include "UnitTestSupport.hpp"

using namespace ebsdlib;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationConverterTest::TestEuler2Quaternion", "[EbsdLib][OrientationConverterTest]")
{
  size_t nTuples = 2;
  // int qStride = 4;
  std::vector<size_t> cDims(1, 3);
  ebsdlib::FloatArrayType::Pointer eulers = ebsdlib::FloatArrayType::CreateArray(nTuples, cDims, "Eulers", true);
  // Initialize the Eulers with some values
  eulers->setComponent(0, 0, 302.84f * ebsdlib::constants::k_PiOver180F);
  eulers->setComponent(0, 1, 51.282f * ebsdlib::constants::k_PiOver180F);
  eulers->setComponent(0, 2, 37.969f * ebsdlib::constants::k_PiOver180F);
  eulers->setComponent(1, 0, 45.0f * ebsdlib::constants::k_PiOver180F);
  eulers->setComponent(1, 1, 0.0f * ebsdlib::constants::k_PiOver180F);
  eulers->setComponent(1, 2, 0.0f * ebsdlib::constants::k_PiOver180F);

  // float rad = 302.84f * static_cast<float>(ebsdlib::constants::k_PiOver180);
  // std::cout << "Rad: " << rad << std::endl;
  // std::cout << "2Pi: " << ebsdlib::constants::k_2Pi << std::endl;

  // std::cout << "Remainer (302.84/360): " << remainder(rad, ebsdlib::constants::k_2Pi) << std::endl;
  // std::cout << "fmod (5.28556 / 2Pi): " << fmod(rad, ebsdlib::constants::k_2Pi) << std::endl;

  OrientationConverter<ebsdlib::FloatArrayType, float>::Pointer ocEulers = EulerConverter<ebsdlib::FloatArrayType, float>::New();
  ocEulers->setInputData(eulers);
  ocEulers->convertRepresentationTo(ebsdlib::orientations::Type::Quaternion);

  ebsdlib::FloatArrayType::Pointer output = ocEulers->getOutputData();

  std::vector<float> exemplar = {-0.2919894754886627F, 0.319372F, 0.1502762138843536F, 0.8889099955558777F, 0.0000000000000000F, -0.000000F, -0.3826834559440613F, 0.9238795042037964F};

  for(size_t i = 0; i < 8; i++)
  {
    float delta = std::abs(exemplar[i] - (*output)[i]);
    REQUIRE(delta < 1.0E-6);
  }
}

// -----------------------------------------------------------------------------
void TestEulerAngle(float phi1, float phi, float phi2)
{
  //  std::cout << "TESTING EULER ANGLE (Rad): " << phi1 << ", " << phi << ", " << phi2 << std::endl;

  size_t nTuples = 1;
  int qStride = 4;
  std::vector<size_t> cDims(1, 3);
  ebsdlib::FloatArrayType::Pointer eulers = ebsdlib::FloatArrayType::CreateArray(nTuples, cDims, "Eulers", true);
  // Initialize the Eulers with some values
  eulers->setComponent(0, 0, phi1);
  eulers->setComponent(0, 1, phi);
  eulers->setComponent(0, 2, phi2);

  using StringContainerType = std::vector<std::string>;
  using OCType = OrientationConverter<ebsdlib::FloatArrayType, float>;
  std::vector<ebsdlib::orientations::Type> ocTypes = OCType::GetOrientationTypes();
  auto tStrings = OCType::GetOrientationTypeStrings<StringContainerType>();
  std::vector<OCType::Pointer> converters(6);
  converters[0] = EulerConverter<ebsdlib::FloatArrayType, float>::New();
  converters[1] = OrientationMatrixConverter<ebsdlib::FloatArrayType, float>::New();
  converters[2] = QuaternionConverter<ebsdlib::FloatArrayType, float>::New();
  converters[3] = AxisAngleConverter<ebsdlib::FloatArrayType, float>::New();
  converters[4] = RodriguesConverter<ebsdlib::FloatArrayType, float>::New();
  converters[5] = HomochoricConverter<ebsdlib::FloatArrayType, float>::New();
  // converters[6] = CubochoricConverter<ebsdlib::FloatArrayType,float>::New();

  auto strides = OCType::GetComponentCounts<std::vector<int>>();

  for(size_t t0 = 0; t0 < 1; t0++)
  {
    for(size_t t1 = 1; t1 < converters.size(); t1++)
    {
      if(t0 == t1)
      {
        continue;
      }

      converters[t0]->setInputData(eulers);
      converters[t0]->convertRepresentationTo(ocTypes[t1]);
      ebsdlib::FloatArrayType::Pointer t1_output = converters[t0]->getOutputData();

      converters[t1]->setInputData(t1_output);
      converters[t1]->convertRepresentationTo(ocTypes[t0]);
      ebsdlib::FloatArrayType::Pointer t0_output = converters[t1]->getOutputData();

      // Compare via orientation matrices to avoid Euler angle ambiguities
      // (periodicity and gimbal lock at Phi=0 or Phi=pi)
      for(size_t i = 0; i < nTuples; i++)
      {
        float* orig = eulers->getPointer(i * 3);
        float* converted = t0_output->getPointer(i * 3);

        ebsdlib::Euler<float> origEu(orig[0], orig[1], orig[2]);
        ebsdlib::Euler<float> convEu(converted[0], converted[1], converted[2]);
        auto origOm = origEu.toOrientationMatrix();
        auto convOm = convEu.toOrientationMatrix();

        for(size_t j = 0; j < 9; j++)
        {
          float delta = std::abs(origOm[j] - convOm[j]);
          REQUIRE(delta < 1.0E-4);
        }
      }
    }
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::OrientationConverterTest::TestEulerConversion", "[EbsdLib][OrientationConverterTest]")
{

  float numSteps = 4.0F;
  float phi1Inc = 360.0F / numSteps;
  float phiInc = 180.0F / numSteps;
  float phi2Inc = 360.0F / numSteps;

  for(float p2 = 0.0; p2 < 361.0; p2 = p2 + phi2Inc)
  {
    for(float p = 0.0; p < 181.0; p = p + phiInc)
    {
      for(float p1 = 0.0; p1 < 361.0; p1 = p1 + phi1Inc)
      {
        //        std::cout << "TESTING EULER ANGLE (Degrees): " << p1 << ", " << p << ", " << p2 << std::endl;
        TestEulerAngle(p1 * ebsdlib::constants::k_PiOver180F, p * ebsdlib::constants::k_PiOver180F, p2 * ebsdlib::constants::k_PiOver180F);
      }
    }
  }
}
