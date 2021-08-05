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

#include "LaueOps.h"

#include <chrono>
#include <limits>
#include <random>

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Core/EbsdMacros.h"
#include "EbsdLib/LaueOps/CubicLowOps.h"
#include "EbsdLib/LaueOps/CubicOps.h"
#include "EbsdLib/LaueOps/HexagonalLowOps.h"
#include "EbsdLib/LaueOps/HexagonalOps.h"
#include "EbsdLib/LaueOps/MonoclinicOps.h"
#include "EbsdLib/LaueOps/OrthoRhombicOps.h"
#include "EbsdLib/LaueOps/TetragonalLowOps.h"
#include "EbsdLib/LaueOps/TetragonalOps.h"
#include "EbsdLib/LaueOps/TriclinicOps.h"
#include "EbsdLib/LaueOps/TrigonalLowOps.h"
#include "EbsdLib/LaueOps/TrigonalOps.h"
#include "EbsdLib/Math/EbsdLibRandom.h"
#include "EbsdLib/Utilities/ColorTable.h"

namespace Detail
{

// const static double m_OnePointThree = 1.33333333333f;

// const static double sin_wmin_neg_1_over_2 = static_cast<double>(std::sin(EbsdLib::Constants::k_ACosNeg1 / 2.0f));
// const static double sin_wmin_pos_1_over_2 = static_cast<double>(std::sin(EbsdLib::Constants::k_ACos1 / 2.0f));
// const static double sin_of_acos_neg_1 = std::sin(EbsdLib::Constants::k_ACosNeg1);
// const static double sin_of_acos_pos_1 = std::sin(EbsdLib::Constants::k_ACos1);

//  const double recip_sin_of_acos_neg_1 = 1.0f / sin_of_acos_neg_1;
//  const double recip_sin_of_acos_pos_1 = 1.0f / sin_of_acos_pos_1;

// const static double SinOfHalf = std::sin(0.5f);
// const static double CosOfHalf = cosf(0.5f);
// const static double SinOfZero = std::sin(0.0f);
// const static double CosOfZero = cosf(0.0f);
} // namespace Detail

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
LaueOps::LaueOps() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
LaueOps::~LaueOps() = default;

// -----------------------------------------------------------------------------
QuatD LaueOps::getFZQuat(const QuatD& qr) const
{
  EBSD_METHOD_NOT_IMPLEMENTED()
  return QuatD();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
OrientationD LaueOps::calculateMisorientationInternal(const std::vector<QuatD>& quatsym, const QuatD& q1, const QuatD& q2) const
{
  OrientationD axisAngleMin(0.0, 0.0, 0.0, std::numeric_limits<double>::max());
  QuatD qc;
  QuatD qr = q1 * (q2.conjugate());
  size_t numsym = quatsym.size();
  for(size_t i = 0; i < numsym; i++)
  {
    qc = quatsym[i] * qr;

    if(qc.w() < -1)
    {
      qc.w() = -1.0;
    }
    else if(qc.w() > 1)
    {
      qc.w() = 1.0;
    }

    OrientationD axisAngle = OrientationTransformation::qu2ax<QuatD, OrientationType>(qc);
    if(axisAngle[3] > EbsdLib::Constants::k_PiD)
    {
      axisAngle[3] = EbsdLib::Constants::k_2PiD - axisAngle[3];
    }
    if(axisAngle[3] < axisAngleMin[3])
    {
      axisAngleMin = axisAngle;
    }
  }
  double denom = sqrt((axisAngleMin[0] * axisAngleMin[0] + axisAngleMin[1] * axisAngleMin[1] + axisAngleMin[2] * axisAngleMin[2]));
  axisAngleMin[0] = axisAngleMin[0] / denom;
  axisAngleMin[1] = axisAngleMin[1] / denom;
  axisAngleMin[2] = axisAngleMin[2] / denom;
  if(denom == 0.0 || axisAngleMin[3] == 0.0)
  {
    axisAngleMin[0] = 0.0;
    axisAngleMin[1] = 0.0;
    axisAngleMin[2] = 1.0;
  }

  return axisAngleMin;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
OrientationType LaueOps::_calcRodNearestOrigin(const std::vector<OrientationD>& rodsym, const OrientationType& inRod) const
{
  double denom = 0.0f, dist = 0.0f;
  double smallestdist = 100000000.0f;
  double rc1 = 0.0f, rc2 = 0.0f, rc3 = 0.0f;
  OrientationType outRod(4, 0.0f);
  // Turn into an actual 3 Comp Rodrigues Vector
  OrientationType rod = inRod;
  rod[0] *= rod[3];
  rod[1] *= rod[3];
  rod[2] *= rod[3];
  size_t numsym = rodsym.size();
  for(size_t i = 0; i < numsym; i++)
  {
    denom = 1 - (rod[0] * rodsym[i][0] + rod[1] * rodsym[i][1] + rod[2] * rodsym[i][2]);
    rc1 = (rod[0] + rodsym[i][0] - (rod[1] * rodsym[i][2] - rod[2] * rodsym[i][1])) / denom;
    rc2 = (rod[1] + rodsym[i][1] - (rod[2] * rodsym[i][0] - rod[0] * rodsym[i][2])) / denom;
    rc3 = (rod[2] + rodsym[i][2] - (rod[0] * rodsym[i][1] - rod[1] * rodsym[i][0])) / denom;
    dist = rc1 * rc1 + rc2 * rc2 + rc3 * rc3;
    if(dist < smallestdist)
    {
      smallestdist = dist;
      outRod[0] = rc1;
      outRod[1] = rc2;
      outRod[2] = rc3;
    }
  }
  double mag = sqrt(outRod[0] * outRod[0] + outRod[1] * outRod[1] + outRod[2] * outRod[2]);
  if(mag == 0.0f)
  {
    outRod[3] = std::numeric_limits<double>::infinity();
  }
  else
  {
    outRod[3] = mag;
    outRod[0] = outRod[0] / outRod[3];
    outRod[1] = outRod[1] / outRod[3];
    outRod[2] = outRod[2] / outRod[3];
  }
  return outRod;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QuatD LaueOps::_calcNearestQuat(const std::vector<QuatD>& quatsym, const QuatD& q1, const QuatD& q2) const
{
  QuatD out;
  double dist = 0.0;
  double smallestdist = 1000000.0f;
  QuatD qmax;
  size_t numsym = quatsym.size();
  for(size_t i = 0; i < numsym; i++)
  {
    QuatD qc = quatsym[i] * q2;
    if(qc.w() < 0)
    {
      qc.negate();
    }
    dist = static_cast<double>(1 - (qc.w() * q1.w() + qc.x() * q1.x() + qc.y() * q1.y() + qc.z() * q1.z()));
    if(dist < smallestdist)
    {
      smallestdist = dist;
      qmax = qc;
    }
  }
  out = qmax;
  if(out.w() < 0)
  {
    out.negate();
  }
  return out;
}

QuatD LaueOps::_calcQuatNearestOrigin(const std::vector<QuatD>& quatsym, const QuatD& qr) const
{
  double dist = 0.0;
  double smallestdist = 1000000.0f;
  QuatD qmax;
  size_t numsym = quatsym.size();
  for(size_t i = 0; i < numsym; i++)
  {
    QuatD qc = quatsym[i] * qr;

    dist = 1 - (qc.w() * qc.w());
    if(dist < smallestdist)
    {
      smallestdist = dist;
      qmax = qc;
    }
  }
  QuatD out = qmax;

  if(out.w() < 0)
  {
    out.negate();
  }
  return out;
}

int LaueOps::_calcMisoBin(double dim[3], double bins[3], double step[3], const OrientationType& ho) const
{
  int miso1bin = int((ho[0] + dim[0]) / step[0]);
  int miso2bin = int((ho[1] + dim[1]) / step[1]);
  int miso3bin = int((ho[2] + dim[2]) / step[2]);
  if(miso1bin >= bins[0])
  {
    miso1bin = static_cast<int>(bins[0] - 1);
  }
  if(miso2bin >= bins[1])
  {
    miso2bin = static_cast<int>(bins[1] - 1);
  }
  if(miso3bin >= bins[2])
  {
    miso3bin = static_cast<int>(bins[2] - 1);
  }
  if(miso1bin < 0)
  {
    miso1bin = 0;
  }
  if(miso2bin < 0)
  {
    miso2bin = 0;
  }
  if(miso3bin < 0)
  {
    miso3bin = 0;
  }
  return (static_cast<int>((bins[0] * bins[1] * miso3bin) + (bins[0] * miso2bin) + miso1bin));
}

void LaueOps::_calcDetermineHomochoricValues(double random[3], double init[3], double step[3], int32_t phi[3], double& r1, double& r2, double& r3) const
{
  r1 = (step[0] * phi[0]) + (step[0] * random[0]) - (init[0]);
  r2 = (step[1] * phi[1]) + (step[1] * random[1]) - (init[1]);
  r3 = (step[2] * phi[2]) + (step[2] * random[2]) - (init[2]);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int LaueOps::_calcODFBin(double dim[3], double bins[3], double step[3], const OrientationType& ho) const
{
  int g1euler1bin;
  int g1euler2bin;
  int g1euler3bin;
  int g1odfbin;
  g1euler1bin = int((ho[0] + dim[0]) / step[0]);
  g1euler2bin = int((ho[1] + dim[1]) / step[1]);
  g1euler3bin = int((ho[2] + dim[2]) / step[2]);
  if(g1euler1bin >= bins[0])
  {
    g1euler1bin = static_cast<int>(bins[0] - 1);
  }
  if(g1euler2bin >= bins[1])
  {
    g1euler2bin = static_cast<int>(bins[1] - 1);
  }
  if(g1euler3bin >= bins[2])
  {
    g1euler3bin = static_cast<int>(bins[2] - 1);
  }
  if(g1euler1bin < 0)
  {
    g1euler1bin = 0;
  }
  if(g1euler2bin < 0)
  {
    g1euler2bin = 0;
  }
  if(g1euler3bin < 0)
  {
    g1euler3bin = 0;
  }
  g1odfbin = static_cast<int>((g1euler3bin * bins[0] * bins[1]) + (g1euler2bin * bins[0]) + (g1euler1bin));
  return g1odfbin;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::vector<LaueOps::Pointer> LaueOps::GetAllOrientationOps()
{
  std::vector<LaueOps::Pointer> m_OrientationOps;
  m_OrientationOps.push_back(HexagonalOps::New());

  m_OrientationOps.push_back(CubicOps::New());

  m_OrientationOps.push_back(HexagonalLowOps::New()); // Hex Low
  m_OrientationOps.push_back(CubicLowOps::New());     // Cubic Low
  m_OrientationOps.push_back(TriclinicOps::New());    // Triclinic
  m_OrientationOps.push_back(MonoclinicOps::New());   // Monoclinic

  m_OrientationOps.push_back(OrthoRhombicOps::New()); // OrthoRhombic

  m_OrientationOps.push_back(TetragonalLowOps::New()); // Tetragonal-low
  m_OrientationOps.push_back(TetragonalOps::New());    // Tetragonal-high

  m_OrientationOps.push_back(TrigonalLowOps::New()); // Trigonal-low
  m_OrientationOps.push_back(TrigonalOps::New());    // Trigonal-High

  m_OrientationOps.push_back(OrthoRhombicOps::New()); // Axis OrthorhombicOps

  return m_OrientationOps;
}

// -----------------------------------------------------------------------------
LaueOps::Pointer LaueOps::GetOrientationOpsFromSpaceGroupNumber(size_t sgNumber)
{
  std::array<size_t, 32> sgpg = {1, 2, 3, 6, 10, 16, 25, 47, 75, 81, 83, 89, 99, 111, 123, 143, 147, 149, 156, 162, 168, 174, 175, 177, 183, 187, 191, 195, 200, 207, 215, 221};
  std::array<size_t, 32> pgLaue = {1, 1, 2, 2, 2, 22, 22, 22, 4, 4, 4, 42, 42, 42, 42, 3, 3, 32, 32, 32, 6, 6, 6, 62, 62, 62, 62, 23, 23, 43, 43, 43};

  size_t pgNumber = sgpg.size() - 1;
  for(size_t i = 0; i < sgpg.size(); i++)
  {
    if(sgpg[i] > sgNumber)
    {
      pgNumber = i;
      break;
    }
  }

  size_t value = pgLaue.at(pgNumber);
  switch(value)
  {
  case 1:
    return TriclinicOps::New();
  case 2:
    return MonoclinicOps::New();
  case 22:
    return OrthoRhombicOps::New();
  case 4:
    return TetragonalLowOps::New();
  case 42:
    return TetragonalOps::New();
  case 3:
    return TrigonalLowOps::New();
  case 32:
    return TrigonalOps::New();
  case 6:
    return HexagonalLowOps::New();
  case 62:
    return HexagonalOps::New();
  case 23:
    return CubicLowOps::New();
  case 43:
    return CubicOps::New();
  default:
    return LaueOps::NullPointer();
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::vector<std::string> LaueOps::GetLaueNames()
{
  std::vector<std::string> names;

  std::vector<LaueOps::Pointer> ops = GetAllOrientationOps();
  names.reserve(ops.size());
  for(const auto& op : ops)
  {
    names.push_back(op->getSymmetryName());
  }

  return names;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
size_t LaueOps::getRandomSymmetryOperatorIndex(int numSymOps) const
{

  using SizeTDistributionType = std::uniform_int_distribution<size_t>;

  const SizeTDistributionType::result_type rangeMin = 0;
  const SizeTDistributionType::result_type rangeMax = static_cast<SizeTDistributionType::result_type>(numSymOps - 1);

  std::random_device randomDevice;           // Will be used to obtain a seed for the random number engine
  std::mt19937_64 generator(randomDevice()); // Standard mersenne_twister_engine seeded with rd()
  std::mt19937_64::result_type seed = static_cast<std::mt19937_64::result_type>(std::chrono::steady_clock::now().time_since_epoch().count());
  generator.seed(seed);
  SizeTDistributionType distribution(rangeMin, rangeMax);

  size_t symOp = distribution(generator); // Random remaining position.
  return symOp;
}

// -----------------------------------------------------------------------------
LaueOps::Pointer LaueOps::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
std::string LaueOps::getNameOfClass() const
{
  return std::string("LaueOps");
}

// -----------------------------------------------------------------------------
std::string LaueOps::ClassName()
{
  return std::string("LaueOps");
}
