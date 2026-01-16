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

#include "LaueOps.h"

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
#include "EbsdLib/Orientation/Quaternion.hpp"
#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/ComputeStereographicProjection.h"

#include <algorithm> // for std::max
#include <chrono>
#include <exception>
#include <limits>
#include <random>

/**
| Index | Verified | Class           | Rotation Point Group | Num Sym Ops |
|-------|----------|-----------------|----------------------|-------------|
|   X    |   X     | TriclinicOps    | 1     | 1           |
|    X   |   X     | MonoclinicOps   | 2     | 2           |
|    X   |   X     | Orthorhombic    | 222   | 4           |
|  X     |   X     | TetragonalLow   | 4     | 4           |
|   X    |   X     | TetragonalOps   | 422    | 8           |
|   X    |   X     | TrigonalLow     | 3     | 3           |
|   X    |   X     | TrigonalOps     | 32    | 6           |
|    X   |   X     | HexagonalLow    | 6     | 6           |
|   X    |   X     | HexagonalOps    | 622   | 12          |
|   X    |   X     | CubicLow        | 23    | 12          |
|   X    |   X     | CubicOps        | 432   | 24          |
*/
using namespace ebsdlib;

namespace
{

// Based on P1830R1
template <class...>
inline constexpr bool dependent_false = false;

// Based on std::to_underlying from c++23
template <class Enum>
constexpr std::underlying_type_t<Enum> to_underlying(Enum e) noexcept
{
  return static_cast<std::underlying_type_t<Enum>>(e);
}

} // namespace

// -----------------------------------------------------------------------------
LaueOps::LaueOps() = default;

// -----------------------------------------------------------------------------
LaueOps::~LaueOps() = default;

// -----------------------------------------------------------------------------
std::string LaueOps::FZTypeToString(const FZType value)
{
  switch(value)
  {
  case FZType::Anorthic:
    return "Anorthic (Triclinic)";
  case FZType::Cyclic:
    return "Cyclic";
  case FZType::Dihedral:
    return "Dihedral";
  case FZType::Tetrahedral:
    return "Tetrahedral";
  case FZType::Octahedral:
    return "Octahedral";
  default:
    return "Unknown FZType";
  }
}

// -----------------------------------------------------------------------------
std::string LaueOps::AxisOrderingTypeToString(const AxisOrderingType value)
{
  switch(value)
  {
  case AxisOrderingType::None:
    return "None";
  case AxisOrderingType::TwoFold:
    return "TwoFold";
  case AxisOrderingType::ThreeFold:
    return "ThreeFold";
  case AxisOrderingType::FourFold:
    return "FourFold";
  case AxisOrderingType::SixFold:
    return "SixFold";
  case AxisOrderingType::EightFold:
    return "EightFold";
  case AxisOrderingType::TenFold:
    return "TenFold";
  case AxisOrderingType::TwelveFold:
    return "TwelveFold";
  default:
    return "Unknown AxisOrderingType";
  }
}

// -----------------------------------------------------------------------------
LaueOps::FZType LaueOps::getFZType() const
{
  return laue_ops::FZtarray[getPointGroup() - 1];
}

// -----------------------------------------------------------------------------
LaueOps::AxisOrderingType LaueOps::getAxisOrderingType() const
{
  return laue_ops::FZoarray[getPointGroup() - 1];
}

// -----------------------------------------------------------------------------
ebsdlib::Rgb LaueOps::computeIPFColor(double* eulers, double* refDir, bool degToRad) const
{

  const ebsdlib::Matrix3X1D refDirection(refDir);
  double chi = 0.0f;
  double eta = 0.0f;
  double _rgb[3] = {0.0, 0.0, 0.0};

  EulerDType eu(eulers[0], eulers[1], eulers[2]);
  if(degToRad)
  {
    eu[0] = eu[0] * ebsdlib::constants::k_DegToRadD;
    eu[1] = eu[1] * ebsdlib::constants::k_DegToRadD;
    eu[2] = eu[2] * ebsdlib::constants::k_DegToRadD;
  }
  OrientationMatrixDType om; // Reusable for the loop
  QuatD q1 = eu.toQuaternion();

  for(int j = 0; j < getNumSymOps(); j++)
  {
    // QuatD qu = getQuatSymOp(j) * q1;
    QuaternionDType qu(getQuatSymOp(j) * q1);
    om = qu.toOrientationMatrix();
    ebsdlib::Matrix3X3D g(om.data());
    ebsdlib::Matrix3X1D p = (g * refDirection).normalize();

    if(!getHasInversion() && p[2] < 0)
    {
      continue;
    }
    if(getHasInversion() && p[2] < 0)
    {
      p = p * -1.0;
    }
    chi = std::acos(p[2]);
    eta = std::atan2(p[1], p[0]);
    if(!inUnitTriangle(eta, chi))
    {
      continue;
    }
    break;
  }

  const std::array<double, 3> angleLimits = getIpfColorAngleLimits(eta);

  _rgb[0] = 1.0 - chi / angleLimits[2];
  _rgb[2] = std::fabs(eta - angleLimits[0]) / (angleLimits[1] - angleLimits[0]);
  _rgb[1] = 1 - _rgb[2];
  _rgb[1] *= chi / angleLimits[2];
  _rgb[2] *= chi / angleLimits[2];
  _rgb[0] = std::sqrt(_rgb[0]);
  _rgb[1] = std::sqrt(_rgb[1]);
  _rgb[2] = std::sqrt(_rgb[2]);

  double max = _rgb[0];
  if(_rgb[1] > max)
  {
    max = _rgb[1];
  }
  if(_rgb[2] > max)
  {
    max = _rgb[2];
  }

  _rgb[0] = _rgb[0] / max;
  _rgb[1] = _rgb[1] / max;
  _rgb[2] = _rgb[2] / max;

  return ebsdlib::RgbColor::dRgb(static_cast<int32_t>(_rgb[0] * 255), static_cast<int32_t>(_rgb[1] * 255), static_cast<int32_t>(_rgb[2] * 255), 255);
}

// -----------------------------------------------------------------------------
void LaueOps::RodriguesComposition(RodriguesDType sigma, RodriguesDType& rod)
{
  std::array<double, 3> rho;
  rho[0] = -rod[0] * rod[3];
  rho[1] = -rod[1] * rod[3];
  rho[2] = -rod[2] * rod[3];

  // perform the Rodrigues rotation composition with sigma to get rhomis
  double denom = 1.0 + (sigma[0] * rho[0] + sigma[1] * rho[1] + sigma[2] * rho[2]);
  if(denom == 0.0)
  {
    const double len = sqrt(sigma[0] * sigma[0] + sigma[1] * sigma[1] + sigma[2] * sigma[2]);
    rod[0] = sigma[0] / len;
    rod[1] = sigma[1] / len;
    rod[2] = sigma[2] / len;
    rod[3] = std::numeric_limits<double>::infinity(); // set this to infinity
  }
  else
  {
    std::array<double, 3> rhomis;
    rhomis[0] = (rho[0] - sigma[0] + (rho[1] * sigma[2] - rho[2] * sigma[1])) / denom;
    rhomis[1] = (rho[1] - sigma[1] + (rho[2] * sigma[0] - rho[0] * sigma[2])) / denom;
    rhomis[2] = (rho[2] - sigma[2] + (rho[0] * sigma[1] - rho[1] * sigma[0])) / denom;
    // revert rhomis to a four-component Rodrigues vector
    double len = sqrt(rhomis[0] * rhomis[0] + rhomis[1] * rhomis[1] + rhomis[2] * rhomis[2]);
    if(len != 0.0)
    {
      rod[0] = -rhomis[0] / len;
      rod[1] = -rhomis[1] / len;
      rod[2] = -rhomis[2] / len;
      rod[3] = len;
    }
    else
    {
      rod[0] = 0.0;
      rod[1] = 0.0;
      rod[2] = 0.0;
      rod[3] = 0.0;
    }
  }
}

// -----------------------------------------------------------------------------
bool LaueOps::InsideCyclicFZ(const RodriguesDType& rod, FZType fzType, AxisOrderingType order)
{
  bool res = false;
  bool doM = false;

  // if (M.has_value() && *M)
  //   doM = true;

  int32_t orderAsArrayIndex = to_underlying(order) - 1;
  auto x = rod; // Make a copy of the input Rodrigues vector .r_copyd();

  // Case: finite x(4)
  if(x[3] != std::numeric_limits<double>::infinity())
  {
    if(doM)
    {
      if(fzType == FZType::Cyclic && order == AxisOrderingType::TwoFold)
      {
        // Check y-component vs tan(pi/2n)
        res = std::abs(x[1] * x[3]) <= LPs::BP[orderAsArrayIndex];
      }
      else
      {
        // Check z-component vs tan(pi/2n)
        res = std::abs(x[2] * x[3]) <= LPs::BP[orderAsArrayIndex];
      }
    }
    else
    {
      if(fzType == FZType::Cyclic && order == AxisOrderingType::TwoFold)
      {
        res = std::abs(x[1] * x[3]) <= LPs::BP[orderAsArrayIndex];
      }
      else
      {
        res = std::abs(x[2] * x[3]) <= LPs::BP[orderAsArrayIndex];
      }
    }
  }
  // Case: infinite x(4)
  else
  {
    if(doM)
    {
      if(fzType == FZType::Cyclic && order == AxisOrderingType::TwoFold)
      {
        if(x[1] == 0.0)
          res = true;
      }
      else
      {
        if(x[2] == 0.0)
          res = true;
      }
    }
    else
    {
      if(fzType == FZType::Cyclic && order == AxisOrderingType::TwoFold)
      {
        if(x[1] == 0.0)
          res = true;
      }
      else
      {
        if(x[2] == 0.0)
          res = true;
      }
    }
  }

  return res;
}

// -----------------------------------------------------------------------------
bool LaueOps::InsideDihedralFZ(const RodriguesDType& rod, const AxisOrderingType order)
{
  constexpr double eps = 1.0e-10;

  bool res = false;
  bool c1 = false;

  if(rod[3] > LPs::rtt) // sqrt(3.0)
  {
    return false;
  }

  if(AxisOrderingType::None == order)
  {
    return false;
  }

  const std::array<double, 4> x = {rod[0], rod[1], rod[2], rod[3]}; // Make a copy of rod
  const std::array<double, 3> r = {x[0] * x[3], x[1] * x[3], x[2] * x[3]};

  // first, check the z-component vs. tan(pi/2n)  (same as insideCyclicFZ)
  const int32_t orderAsArrayIndex = to_underlying(order) - 1;
  c1 = std::fabs(r[2]) <= (LPs::BP[orderAsArrayIndex] + eps);
  res = false;
  //! check the square boundary planes if c1=.TRUE.
  if(c1)
  {
    constexpr double r1 = 1.0;
    bool c2 = false;
    switch(order)
    {
    case AxisOrderingType::TwoFold: {
      c2 = (std::max({std::fabs(r[0]), std::fabs(r[1]), std::fabs(r[2])}) <= r1 + eps);
      break;
    }
    case AxisOrderingType::ThreeFold:
      c2 = std::fabs(LPs::srt * r[1] + 0.5 * r[0]) <= (r1 + eps);
      c2 &= std::fabs(LPs::srt * r[1] - 0.5 * r[0]) <= (r1 + eps);
      c2 &= std::fabs(r[0]) <= (r1 + eps);
      break;

    case AxisOrderingType::FourFold:
      c2 = (std::fabs(r[0]) <= r1) && (std::fabs(r[1]) <= r1);
      c2 &= (LPs::r22 * std::fabs(r[0] + r[1]) <= r1) && (LPs::r22 * std::fabs(r[0] - r[1]) <= r1);
      break;

    case AxisOrderingType::SixFold:
      c2 = std::fabs(0.5 * r[0] + LPs::srt * r[1]) <= (r1 + eps);
      c2 &= std::fabs(LPs::srt * r[0] + 0.5 * r[1]) <= (r1 + eps);
      c2 &= std::fabs(LPs::srt * r[0] - 0.5 * r[1]) <= (r1 + eps);
      c2 &= std::fabs(0.5 * r[0] - LPs::srt * r[1]) <= (r1 + eps);
      c2 &= std::fabs(r[1]) <= (r1 + eps);
      c2 &= std::fabs(r[0]) <= (r1 + eps);
      break;

    default:
      return false;
    }
    res = c2;
  }

  return res;
}

// -----------------------------------------------------------------------------
bool LaueOps::InsideCubicFZ(const RodriguesDType& rod, const FZType fzType)
{
  bool res = false;
  bool c1 = false;
  bool c2 = false;

  constexpr double r1 = 1.0;
  constexpr double eps = 1.0e-10;

  const std::array<double, 4> x = {rod[0], rod[1], rod[2], rod[3]}; // Make a copy of rod
  const std::array<double, 3> r = {x[0] * x[3], x[1] * x[3], x[2] * x[3]};

  // primary cube planes (only needed for octahedral case)
  if(fzType == FZType::Octahedral)
  {
    double max = std::max({std::fabs(r[0]), std::fabs(r[1]), std::fabs(r[2])});
    c1 = (max - LPs::BP[4 - 1]) <= eps;
  }
  else
  {
    c1 = true;
  }

  // octahedral truncation planes, both for tetrahedral and octahedral point groups
  c2 = ((std::fabs(r[0]) + std::fabs(r[1]) + std::fabs(r[2])) - r1) <= eps;

  // if both c1 and c2, then the point is inside
  if(c1 && c2)
  {
    res = true;
  }

  return res;
}

// -----------------------------------------------------------------------------
bool LaueOps::IsInsideFZ(const RodriguesDType& rod, FZType fzType, AxisOrderingType order)
{
  bool insideFZ = false;
  // dealing with 180 rotations is needed only for
  // FZtypes 0 and 1; the other FZs are always finite.
  switch(fzType)
  {
  case FZType::Anorthic:
    insideFZ = true; // all points are inside the FZ
    break;
  case FZType::Cyclic:
    insideFZ = InsideCyclicFZ(rod, fzType, order); // infinity is checked inside this function
    break;
  case FZType::Dihedral:
    if(!std::isinf(rod[3]))
    {
      insideFZ = InsideDihedralFZ(rod, order);
    }
    break;
  case FZType::Tetrahedral:
    if(!std::isinf(rod[3]))
    {
      insideFZ = InsideCubicFZ(rod, FZType::Tetrahedral);
    }
    break;
  case FZType::Octahedral:
    if(!std::isinf(rod[3]))
    {
      insideFZ = InsideCubicFZ(rod, FZType::Octahedral);
    }
    break;
  default:
    insideFZ = false;
    break;
  }
  return insideFZ;
}

bool LaueOps::IsInsideFZ(const QuatD& quat, FZType fzType, AxisOrderingType order)
{
  const RodriguesDType rod = QuaternionDType(quat.getPositiveOrientation()).toRodrigues();
  return IsInsideFZ(rod, fzType, order);
}

// -----------------------------------------------------------------------------
AxisAngleDType LaueOps::calculateMisorientationInternal(const std::vector<QuatD>& quatsym, const QuatD& q1, const QuatD& q2) const
{
  AxisAngleDType axisAngleMin(0.0, 0.0, 0.0, std::numeric_limits<double>::max());
  const QuatD qr = q1 * (q2.conjugate());
  size_t numsym = quatsym.size();
  // Loop through all the symmetry operators and find the Axis Angle with the smallest angular part.
  for(size_t i = 0; i < numsym; i++)
  {
    QuatD qc = quatsym[i] * qr;

    if(qc.w() < -1)
    {
      qc.w() = -1.0;
    }
    else if(qc.w() > 1)
    {
      qc.w() = 1.0;
    }

    AxisAngleDType axisAngle = QuaternionDType(qc).toAxisAngle();
    if(axisAngle[3] > ebsdlib::constants::k_PiD)
    {
      axisAngle[3] = ebsdlib::constants::k_2PiD - axisAngle[3];
    }
    if(axisAngle[3] < axisAngleMin[3])
    {
      axisAngleMin = axisAngle;
    }
  }
  double denom = sqrt((axisAngleMin[0] * axisAngleMin[0] + axisAngleMin[1] * axisAngleMin[1] + axisAngleMin[2] * axisAngleMin[2]));

  if(denom == 0.0 || axisAngleMin[3] == 0.0)
  {
    axisAngleMin[0] = 0.0;
    axisAngleMin[1] = 0.0;
    axisAngleMin[2] = 1.0;
  }
  else
  {
    axisAngleMin[0] = axisAngleMin[0] / denom;
    axisAngleMin[1] = axisAngleMin[1] / denom;
    axisAngleMin[2] = axisAngleMin[2] / denom;
  }

  return axisAngleMin;
}

// -----------------------------------------------------------------------------
RodriguesDType LaueOps::_calcRodNearestOrigin(const RodriguesDType& inRod) const
{
  double denom = 0.0f, dist = 0.0f;
  double smallestdist = 100000000.0f;
  double rc1 = 0.0f, rc2 = 0.0f, rc3 = 0.0f;
  RodriguesDType outRod;
  // Turn into an actual 3 Comp Rodrigues Vector
  RodriguesDType rod = inRod;
  rod[0] *= rod[3];
  rod[1] *= rod[3];
  rod[2] *= rod[3];
  size_t numsym = getNumRodriguesSymOps();

  for(size_t i = 0; i < numsym; i++)
  {
    RodriguesDType currentRodSymmetry = getRodSymOp(i);
    // Convert Rodrigues 4 component into a 3 component
    std::array<double, 3> symRod = {currentRodSymmetry[0] * currentRodSymmetry[3], currentRodSymmetry[1] * currentRodSymmetry[3], currentRodSymmetry[2] * currentRodSymmetry[3]};

    denom = 1 - (rod[0] * symRod[0] + rod[1] * symRod[1] + rod[2] * symRod[2]);
    rc1 = (rod[0] + symRod[0] - (rod[1] * symRod[2] - rod[2] * symRod[1])) / denom;
    rc2 = (rod[1] + symRod[1] - (rod[2] * symRod[0] - rod[0] * symRod[2])) / denom;
    rc3 = (rod[2] + symRod[2] - (rod[0] * symRod[1] - rod[1] * symRod[0])) / denom;
    dist = rc1 * rc1 + rc2 * rc2 + rc3 * rc3;
    if(dist < smallestdist)
    {
      smallestdist = dist;
      outRod[0] = rc1;
      outRod[1] = rc2;
      outRod[2] = rc3;
    }
  }
  double mag = std::sqrt(outRod[0] * outRod[0] + outRod[1] * outRod[1] + outRod[2] * outRod[2]);
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
QuatD LaueOps::_calcNearestQuat(const std::vector<QuatD>& quatsym, const QuatD& q1, const QuatD& q2) const
{
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
  QuatD out = qmax;
  if(out.w() < 0)
  {
    out.negate();
  }
  return out;
}

QuatD LaueOps::ConvertToFZ(const std::vector<QuatD>& quatsym, const QuatD& qr, FZType fzType, AxisOrderingType order)
{
  // Ensure the Quaternion is Normalized and the Scalar Part is positive
  QuatD normalizedQuat = qr.getPositiveOrientation();
  RodriguesDType rod = QuaternionDType(normalizedQuat).toRodrigues();

  if(IsInsideFZ(rod, fzType, order))
  {
    return normalizedQuat;
  }

  size_t numsym = quatsym.size();
  for(size_t i = 0; i < numsym; i++)
  {
    QuatD qc = quatsym[i] * qr;
    normalizedQuat = qc.getPositiveOrientation();
    rod = QuaternionDType(normalizedQuat).toRodrigues();

    if(normalizedQuat.w() < 1.0E5 && IsInsideFZ(rod, fzType, order))
    {
      return normalizedQuat;
    }
  }
  // This should never happen so I guess returning a Qauaternion with all Infinity values is _a_ way to do it?
  // Maybe we should throw an exception instead? Or return a std::optional() if we were using C++17
  return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
}

int LaueOps::_calcMisoBin(double dim[3], double bins[3], double step[3], const HomochoricDType& ho) const
{
  int miso1bin = static_cast<int>((ho[0] + dim[0]) / step[0]);
  int miso2bin = static_cast<int>((ho[1] + dim[1]) / step[1]);
  int miso3bin = static_cast<int>((ho[2] + dim[2]) / step[2]);
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
int LaueOps::_calcODFBin(double dim[3], double bins[3], double step[3], const HomochoricDType& ho) const
{
  int g1euler1bin = static_cast<int>((ho[0] + dim[0]) / step[0]);
  int g1euler2bin = static_cast<int>((ho[1] + dim[1]) / step[1]);
  int g1euler3bin = static_cast<int>((ho[2] + dim[2]) / step[2]);
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
  int g1odfbin = static_cast<int>((g1euler3bin * bins[0] * bins[1]) + (g1euler2bin * bins[0]) + (g1euler1bin));
  return g1odfbin;
}

// -----------------------------------------------------------------------------
std::vector<LaueOps::Pointer> LaueOps::GetAllOrientationOps()
{
  std::vector<LaueOps::Pointer> m_OrientationOps;
  /*[0]*/ m_OrientationOps.push_back(HexagonalOps::New());

  /*[1]*/ m_OrientationOps.push_back(CubicOps::New());

  /*[2]*/ m_OrientationOps.push_back(HexagonalLowOps::New()); // Hex Low
  /*[3]*/ m_OrientationOps.push_back(CubicLowOps::New());     // Cubic Low
  /*[4]*/ m_OrientationOps.push_back(TriclinicOps::New());    // Triclinic
  /*[5]*/ m_OrientationOps.push_back(MonoclinicOps::New());   // Monoclinic

  /*[6]*/ m_OrientationOps.push_back(OrthoRhombicOps::New()); // OrthoRhombic

  /*[7]*/ m_OrientationOps.push_back(TetragonalLowOps::New()); // Tetragonal-low
  /*[8]*/ m_OrientationOps.push_back(TetragonalOps::New());    // Tetragonal-high

  /*[9]*/ m_OrientationOps.push_back(TrigonalLowOps::New()); // Trigonal-low
  /*[10]*/ m_OrientationOps.push_back(TrigonalOps::New());   // Trigonal-High

  /*[11]*/ m_OrientationOps.push_back(OrthoRhombicOps::New()); // Axis OrthorhombicOps

  return m_OrientationOps;
}

// -----------------------------------------------------------------------------
LaueOps::Pointer LaueOps::GetOrientationOpsFromSpaceGroupNumber(const size_t sgNumber)
{
  // There are only 230 Space Groups, so if the user asks for something outside of
  // that range, then return a null pointer. If they are asking for this kind of
  // value then there is something wrong in the calling code.
  if(sgNumber > 230)
  {
    return LaueOps::NullPointer();
  }
  std::array<size_t, 32> sgpg = {1, 2, 3, 6, 10, 16, 25, 47, 75, 81, 83, 89, 99, 111, 123, 143, 147, 149, 156, 162, 168, 174, 175, 177, 183, 187, 191, 195, 200, 207, 215, 221};
  std::array<size_t, 32> pgLaue = {1, 1, 2, 2, 2, 22, 22, 22, 4, 4, 4, 42, 42, 42, 42, 3, 3, 32, 32, 32, 6, 6, 6, 62, 62, 62, 62, 23, 23, 43, 43, 43};

  size_t pgNumber = sgpg.size() - 1;
  size_t i = 0;
  for(i = 0; i < sgpg.size(); i++)
  {
    if(sgpg[i] > sgNumber)
    {
      pgNumber = i;
      break;
    }
  }

  // std::cout << "Space Group: " << sgNumber << "   sgpg: " << i << "   sgpg[i]: " << sgpg[i] << "   pgNumber: " << pgNumber << "  pgLaue[pgNumber]: " << pgLaue[pgNumber] << std::endl;

  switch(pgLaue.at(pgNumber))
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
size_t LaueOps::getRandomSymmetryOperatorIndex(const int numSymOps) const
{

  using SizeTDistributionType = std::uniform_int_distribution<size_t>;

  constexpr SizeTDistributionType::result_type rangeMin = 0;
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
  return {"LaueOps"};
}

// -----------------------------------------------------------------------------
std::string LaueOps::ClassName()
{
  return {"LaueOps"};
}

//-----------------------------------------------------------------------------
ebsdlib::Rgb LaueOps::generateMisorientationColor(const QuatD& q, const QuatD& refFrame) const
{
  throw std::runtime_error("LaueOps::generateMisorientationColor is not implemented.");
}
