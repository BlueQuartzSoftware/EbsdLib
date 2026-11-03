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
#include <catch2/catch.hpp>

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/LaueOps/CubicOps.h"
#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Orientation/Quaternion.hpp"
#include "EbsdLib/Orientation/Rodrigues.hpp"
#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

#include "UnitTestSupport.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <random>

using namespace ebsdlib;

/* clang-format off */
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
/* clang-format on */

} // namespace detail

static RodriguesDType convertRodrigues(const std::array<double, 3>& rod)
{
  const float length = sqrt(rod[0] * rod[0] + rod[1] * rod[1] + rod[2] * rod[2]);
  return {rod[0] / length, rod[1] / length, rod[2] / length, length};
}

TEST_CASE("ebsdlib::ConvertToFundamentalZoneTest::CubicBoundaryUnique", "[EbsdLib][ConvertToFundamentalZoneTest]")
{
  const auto ops = CubicOps::New();
  const double halfAngle = std::numbers::pi / 8.0;
  const QuatD positive(0.0, 0.0, std::sin(halfAngle), std::cos(halfAngle));
  const QuatD negative(0.0, 0.0, -std::sin(halfAngle), std::cos(halfAngle));

  bool equivalent = false;
  for(size_t symIdx = 0; symIdx < ops->getNumSymOps(); ++symIdx)
  {
    const QuatD image = (ops->getQuatSymOp(symIdx) * positive).normalize();
    equivalent |= std::min((image - negative).length(), (image + negative).length()) < 1.0e-12;
  }
  REQUIRE(equivalent);

  const QuatD a = ops->getFZQuat(positive);
  const QuatD b = ops->getFZQuat(negative);
  CHECK(std::min((a - b).length(), (a + b).length()) < 1.0e-12);
}

TEST_CASE("ebsdlib::ConvertToFundamentalZoneTest::SymmetryOrbitUnique", "[EbsdLib][ConvertToFundamentalZoneTest]")
{
  std::vector<QuatD> orientations = {{0.0, 0.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, {0.6, 0.8, 0.0, 0.0}, {0.0, 0.6, 0.8, 0.0}};
  const double diagonal = 1.0 / std::sqrt(3.0);
  const std::array<std::array<double, 4>, 5> axisAngles = {{{0.0, 0.0, 1.0, std::numbers::pi / 4.0},
                                                            {diagonal, diagonal, diagonal, std::numbers::pi / 3.0},
                                                            {0.0, 0.0, 1.0, std::numbers::pi / 6.0},
                                                            {1.0, 0.0, 0.0, std::numbers::pi / 2.0},
                                                            {diagonal, diagonal, diagonal, std::numbers::pi}}};
  for(const auto& axisAngle : axisAngles)
  {
    for(const double offset : {0.0, -1.0e-12, 1.0e-12, -1.0e-9, 1.0e-9})
    {
      const double halfAngle = (axisAngle[3] + offset) / 2.0;
      const double sine = std::sin(halfAngle);
      orientations.emplace_back(axisAngle[0] * sine, axisAngle[1] * sine, axisAngle[2] * sine, std::cos(halfAngle));
    }
  }

  std::mt19937_64 generator(0x63F2);
  std::normal_distribution<double> distribution(0.0, 1.0);
  for(size_t i = 0; i < 128; ++i)
  {
    const double x = distribution(generator);
    const double y = distribution(generator);
    const double z = distribution(generator);
    const double w = distribution(generator);
    orientations.push_back(QuatD(x, y, z, w).normalize());
  }

  for(const auto& ops : LaueOps::GetAllOrientationOps())
  {
    INFO("Laue class: " << ops->getSymmetryName());
    for(size_t orientationIdx = 0; orientationIdx < orientations.size(); ++orientationIdx)
    {
      CAPTURE(orientationIdx);
      const QuatD q = orientations[orientationIdx];
      const QuatD expected = ops->getFZQuat(q);
      REQUIRE(std::isfinite(expected.length()));
      CHECK(LaueOps::IsInsideFZ(expected, ops->getFZType(), ops->getAxisOrderingType()));
      double orbitDistance = std::numeric_limits<double>::infinity();
      for(size_t symIdx = 0; symIdx < ops->getNumSymOps(); ++symIdx)
      {
        CAPTURE(symIdx);
        const QuatD image = (ops->getQuatSymOp(symIdx) * q).normalize();
        orbitDistance = std::min({orbitDistance, (image - expected).length(), (image + expected).length()});
        for(const QuatD& input : {image, -image})
        {
          const QuatD actual = ops->getFZQuat(input);
          CHECK(std::min((actual - expected).length(), (actual + expected).length()) < 1.0e-9);
          CHECK(LaueOps::IsInsideFZ(actual, ops->getFZType(), ops->getAxisOrderingType()));
        }
      }
      CHECK(orbitDistance < 1.0e-9);
    }
  }
}

TEST_CASE("ebsdlib::ConvertToFundamentalZoneTest", "[EbsdLib][ConvertToFundamentalZoneTest]")
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
      RodriguesDType testRod = convertRodrigues(rod);
      bool isInside = LaueOps::IsInsideFZ(testRod, ops[opsIdx]->getFZType(), ops[opsIdx]->getAxisOrderingType());

      REQUIRE(isInside == testValues[testIdx]);
      // if(testValues[testIdx] != isInside)
      // {
      //   std::stringstream ss;
      //   ss << testIdx << ": " << "(" << rod[0] << ", " << rod[1] << ", " << rod[2] << "), " << testValues[testIdx] << ", " << isInside;
      //   std::cout << ss.str() << std::endl;
      // }

      if(!isInside)
      {
        QuatD quat = testRod.toQuaternion();
        QuatD fzQuat = ops[opsIdx]->getFZQuat(quat);
        RodriguesDType fzRod = QuaternionDType(fzQuat).toRodrigues();
        isInside = LaueOps::IsInsideFZ(fzRod, ops[opsIdx]->getFZType(), ops[opsIdx]->getAxisOrderingType());
        REQUIRE(isInside == true);
      }
    }
  }
}
