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
#include "EbsdLib/Utilities/CanvasUtilities.hpp"
#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/ComputeStereographicProjection.h"
#include "EbsdLib/Utilities/Fonts.hpp"

#include <canvas_ity.hpp>

#include <algorithm> // for std::max
#include <chrono>
#include <exception>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>

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

constexpr float k_OdfBinStepSize = 5.0f;

} // namespace

// -----------------------------------------------------------------------------
LaueOps::LaueOps() = default;

// -----------------------------------------------------------------------------
LaueOps::~LaueOps() = default;

// -----------------------------------------------------------------------------
std::array<float, 3> LaueOps::getOdfBinStepSize() const
{
  return {k_OdfBinStepSize, k_OdfBinStepSize, k_OdfBinStepSize};
}

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
ebsdlib::Rgb LaueOps::computeIPFColor(double* eulers, double* refDir, bool degToRad, const ebsdlib::IColorKey* key) const
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

  if(key != nullptr)
  {
    auto [r, g, b] = key->direction2Color(eta, chi, angleLimits);
    _rgb[0] = r;
    _rgb[1] = g;
    _rgb[2] = b;
    return ebsdlib::RgbColor::dRgb(static_cast<int32_t>(_rgb[0] * 255), static_cast<int32_t>(_rgb[1] * 255), static_cast<int32_t>(_rgb[2] * 255), 255);
  }

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
  constexpr double eps = 1.0e-8; // Match EMsoftOO tolerance for FZ boundary

  const std::array<double, 4> x = {rod[0], rod[1], rod[2], rod[3]}; // Make a copy of rod
  const std::array<double, 3> r = {x[0] * x[3], x[1] * x[3], x[2] * x[3]};
  // primary cube planes (only needed for octahedral case)
  if(fzType == FZType::Octahedral)
  {
    double max = std::max({std::fabs(r[0]), std::fabs(r[1]), std::fabs(r[2])});
    double diff = max - LPs::BP[4 - 1];
    c1 = (diff <= eps);
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
// Find the crystal-symmetry-equivalent orientation of inRod with the smallest
// rotation angle from identity (the FZ representative nearest the origin in
// Rodrigues space).
//
// Done in quaternion space to avoid the singularity at 180° rotations where
// tan(θ/2) = ∞. Rodrigues-space symmetry reduction fails for 180° inputs
// because the infinity in the 4th component propagates as NaN through
// `rod · symRod` when any axis component is zero (IEEE 754: ∞ · 0 = NaN).
//
// Minimizing rotation angle ≡ maximizing |w| of the unit quaternion, since
// |w| = cos(θ/2).
RodriguesDType LaueOps::_calcRodNearestOrigin(const RodriguesDType& inRod) const
{
  QuatD q = inRod.toQuaternion().getPositiveOrientation();
  QuatD qBest = q;
  double largestAbsW = std::fabs(q.w());

  size_t numsym = getNumSymOps();
  for(size_t i = 0; i < numsym; i++)
  {
    QuatD qCandidate = (getQuatSymOp(i) * q).getPositiveOrientation();
    double absW = std::fabs(qCandidate.w());
    if(absW > largestAbsW)
    {
      largestAbsW = absW;
      qBest = qCandidate;
    }
  }

  return qBest.toRodrigues();
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
  QuatD normalizedQuat = qr.normalize().getPositiveOrientation();
  RodriguesDType rod = normalizedQuat.toRodrigues();

  if(IsInsideFZ(rod, fzType, order))
  {
    return normalizedQuat;
  }

  size_t numsym = quatsym.size();
  for(size_t i = 0; i < numsym; i++)
  {
    QuatD qc = (quatsym[i] * qr).normalize();
    normalizedQuat = qc.getPositiveOrientation();
    rod = normalizedQuat.toRodrigues();

    if(normalizedQuat.w() < 1.0E5 && IsInsideFZ(rod, fzType, order))
    {
      return normalizedQuat;
    }
  }
  // This should never happen, so I guess returning a Quaternion with all Infinity values is _a_ way to do it?
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
std::vector<UInt8ArrayType::Pointer> LaueOps::generateInversePoleFigure(InversePoleFigureConfiguration_t& config) const
{
  std::vector<UInt8ArrayType::Pointer> ipfImages(3);

  // Determine labels
  std::string label0 = "IPF-0";
  std::string label1 = "IPF-1";
  std::string label2 = "IPF-2";
  if(config.labels.size() >= 1)
  {
    label0 = config.labels[0];
  }
  if(config.labels.size() >= 2)
  {
    label1 = config.labels[1];
  }
  if(config.labels.size() >= 3)
  {
    label2 = config.labels[2];
  }

  // Step 1: Compute IPF directions for each sample direction
  ebsdlib::FloatArrayType::Pointer dirs0 = InversePoleFigureUtilities::computeIPFDirections(*this, config.eulers, config.sampleDirections[0]);
  ebsdlib::FloatArrayType::Pointer dirs1 = InversePoleFigureUtilities::computeIPFDirections(*this, config.eulers, config.sampleDirections[1]);
  ebsdlib::FloatArrayType::Pointer dirs2 = InversePoleFigureUtilities::computeIPFDirections(*this, config.eulers, config.sampleDirections[2]);

  // Step 2: Compute intensity images for each (using stereographic SST mapping)
  ebsdlib::DoubleArrayType::Pointer intensity0 =
      InversePoleFigureUtilities::computeIPFIntensity(*this, dirs0.get(), config.imageWidth, config.imageHeight, config.lambertDim, config.normalizeMRD, true);
  ebsdlib::DoubleArrayType::Pointer intensity1 =
      InversePoleFigureUtilities::computeIPFIntensity(*this, dirs1.get(), config.imageWidth, config.imageHeight, config.lambertDim, config.normalizeMRD, true);
  ebsdlib::DoubleArrayType::Pointer intensity2 =
      InversePoleFigureUtilities::computeIPFIntensity(*this, dirs2.get(), config.imageWidth, config.imageHeight, config.lambertDim, config.normalizeMRD, true);

  // Step 3: Find global min/max across all 3 intensity images (only for pixels inside SST, value >= 0)
  double globalMax = std::numeric_limits<double>::lowest();
  double globalMin = std::numeric_limits<double>::max();

  std::array<ebsdlib::DoubleArrayType*, 3> intensities = {intensity0.get(), intensity1.get(), intensity2.get()};
  for(auto* intensityArr : intensities)
  {
    double* dPtr = intensityArr->getPointer(0);
    size_t count = intensityArr->getNumberOfTuples();
    for(size_t i = 0; i < count; ++i)
    {
      if(dPtr[i] >= 0.0) // Only consider pixels inside the SST
      {
        if(dPtr[i] > globalMax)
        {
          globalMax = dPtr[i];
        }
        if(dPtr[i] < globalMin)
        {
          globalMin = dPtr[i];
        }
      }
    }
  }

  // Handle case where no valid pixels were found
  if(globalMax < globalMin)
  {
    globalMin = 0.0;
    globalMax = 1.0;
  }

  // Step 4: Create RGBA color images
  std::vector<size_t> dims = {4};
  ebsdlib::UInt8ArrayType::Pointer image0 = ebsdlib::UInt8ArrayType::CreateArray(static_cast<size_t>(config.imageWidth * config.imageHeight), dims, label0, true);
  ebsdlib::UInt8ArrayType::Pointer image1 = ebsdlib::UInt8ArrayType::CreateArray(static_cast<size_t>(config.imageWidth * config.imageHeight), dims, label1, true);
  ebsdlib::UInt8ArrayType::Pointer image2 = ebsdlib::UInt8ArrayType::CreateArray(static_cast<size_t>(config.imageWidth * config.imageHeight), dims, label2, true);

  InversePoleFigureUtilities::createIPFColorImage(intensity0.get(), config.imageWidth, config.imageHeight, config.numColors, globalMin, globalMax, image0.get());
  InversePoleFigureUtilities::createIPFColorImage(intensity1.get(), config.imageWidth, config.imageHeight, config.numColors, globalMin, globalMax, image1.get());
  InversePoleFigureUtilities::createIPFColorImage(intensity2.get(), config.imageWidth, config.imageHeight, config.numColors, globalMin, globalMax, image2.get());

  ipfImages[0] = image0;
  ipfImages[1] = image1;
  ipfImages[2] = image2;

  return ipfImages;
}

//-----------------------------------------------------------------------------
ebsdlib::Rgb LaueOps::generateMisorientationColor(const QuatD& q, const QuatD& refFrame) const
{
  throw std::runtime_error("LaueOps::generateMisorientationColor is not implemented.");
}

// -----------------------------------------------------------------------------
bool LaueOps::mapPixelToSphereSST(int xPixel, int yPixel, int imageDim, std::array<float, 3>& sphereDir) const
{
  return false;
}

// -----------------------------------------------------------------------------
std::array<float, 2> LaueOps::adjustFigureOrigin(std::array<float, 2> figureOrigin, int legendWidth, int legendHeight, const std::vector<float>& margins, float fontPtSize,
                                                 bool generateEntirePlane) const
{
  return figureOrigin;
}

// -----------------------------------------------------------------------------
UInt8ArrayType::Pointer LaueOps::annotateIPFImage(UInt8ArrayType::Pointer triangleImage, int imageDim, int canvasDim, const std::string& title, bool generateEntirePlane, bool hasColorBar,
                                                  ebsdlib::HexConvention conv) const
{
  const float fontPtSize = static_cast<float>(canvasDim) / 24.0f;
  // When a color bar will be drawn, use a wider right margin to make room
  float rightMargin = hasColorBar ? static_cast<float>(canvasDim / 3.5f) : static_cast<float>(canvasDim / 7.0f);
  const std::vector<float> margins = {
      fontPtSize * 3,                      // Top
      rightMargin,                         // Right
      fontPtSize * 2,                      // Bottom
      static_cast<float>(canvasDim / 7.0f) // Left
  };

  int legendHeight = canvasDim - static_cast<int>(margins[0]) - static_cast<int>(margins[2]);
  int legendWidth = canvasDim - static_cast<int>(margins[1]) - static_cast<int>(margins[3]);

  if(legendHeight > legendWidth)
  {
    legendHeight = legendWidth;
  }
  else
  {
    legendWidth = legendHeight;
  }

  int halfWidth = legendWidth / 2;
  int halfHeight = legendHeight / 2;

  std::array<float, 2> figureOrigin = {margins[3], margins[0] * 1.33F};
  figureOrigin = adjustFigureOrigin(figureOrigin, legendWidth, legendHeight, margins, fontPtSize, generateEntirePlane);

  std::array<float, 2> figureCenter = {figureOrigin[0] + halfWidth, figureOrigin[1] + halfHeight};

  // Convert from ARGB to RGBA for canvas_ity
  ebsdlib::UInt8ArrayType::Pointer image = ebsdlib::ConvertColorOrder(triangleImage.get(), imageDim);
  // Mirror across X axis (image drawn with +Y pointing down)
  image = ebsdlib::MirrorImage(image.get(), imageDim);

  // Create canvas
  canvas_ity::canvas context(canvasDim, canvasDim);

  std::vector<unsigned char> latoBold = ebsdlib::fonts::GetLatoBold();
  std::vector<unsigned char> latoRegular = ebsdlib::fonts::GetLatoRegular();
  context.set_font(latoBold.data(), static_cast<int>(latoBold.size()), fontPtSize);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.text_baseline = canvas_ity::alphabetic;

  // Fill background with white
  context.move_to(0.0f, 0.0f);
  context.line_to(static_cast<float>(canvasDim), 0.0f);
  context.line_to(static_cast<float>(canvasDim), static_cast<float>(canvasDim));
  context.line_to(0.0f, static_cast<float>(canvasDim));
  context.line_to(0.0f, 0.0f);
  context.close_path();
  context.set_color(canvas_ity::fill_style, 1.0f, 1.0f, 1.0f, 1.0f);
  context.fill();

  // Draw the triangle image onto the canvas
  context.draw_image(image->getPointer(0), imageDim, imageDim, imageDim * image->getNumberOfComponents(), figureOrigin[0], figureOrigin[1], static_cast<float>(legendWidth),
                     static_cast<float>(legendHeight));

  // Draw title at the large font.
  context.set_font(latoBold.data(), static_cast<int>(latoBold.size()), fontPtSize * 1.5);
  ebsdlib::WriteText(context, title, {margins[0], static_cast<float>(fontPtSize * 1.5)}, fontPtSize * 1.5);

  // Hex/trig basis convention on a small sub-line just under the title (smaller
  // than the Miller-index annotation font) so it fits the cropped width and does
  // not collide with the triangle labels.
  std::string convLine;
  if(conv == ebsdlib::HexConvention::XParallelA)
  {
    convLine = "Convention: X||a (TSL)";
  }
  else if(conv == ebsdlib::HexConvention::XParallelAStar)
  {
    convLine = "Convention: X||a* (MTEX/Oxford)";
  }
  if(!convLine.empty())
  {
    const float subFontSize = fontPtSize * 0.7F;
    context.set_font(latoRegular.data(), static_cast<int>(latoRegular.size()), subFontSize);
    ebsdlib::WriteText(context, convLine, {margins[0], static_cast<float>(fontPtSize * 2.6F)}, static_cast<int>(subFontSize));
  }

  // Draw per-subclass annotations (Miller indices, SST boundary lines)
  context.set_font(latoRegular.data(), static_cast<int>(latoRegular.size()), fontPtSize);
  drawIPFAnnotations(context, canvasDim, fontPtSize, margins, figureOrigin, figureCenter, generateEntirePlane, conv);

  // Extract rendered pixels and remove alpha channel
  ebsdlib::UInt8ArrayType::Pointer rgbaCanvasImage = ebsdlib::UInt8ArrayType::CreateArray(canvasDim * canvasDim, {4ULL}, "Annotated IPF", true);
  context.get_image_data(rgbaCanvasImage->getPointer(0), canvasDim, canvasDim, canvasDim * 4, 0, 0);

  return ebsdlib::RemoveAlphaChannel(rgbaCanvasImage.get());
}

// -----------------------------------------------------------------------------
UInt8ArrayType::Pointer LaueOps::drawColorBar(UInt8ArrayType::Pointer image, int canvasDim, int numColors, double minValue, double maxValue, bool isMRD) const
{
  const float fontPtSize = static_cast<float>(canvasDim) / 24.0f;

  // Generate the color table
  std::vector<float> colors(numColors * 3, 0.0f);
  EbsdColorTable::GetColorTable(numColors, colors);

  // Create a canvas from the existing RGB image by first adding an alpha channel
  const size_t numPixels = static_cast<size_t>(canvasDim * canvasDim);
  ebsdlib::UInt8ArrayType::Pointer rgbaImage = ebsdlib::UInt8ArrayType::CreateArray(numPixels, {4ULL}, "ColorBarCanvas", true);
  uint8_t* srcPtr = image->getPointer(0);
  uint8_t* dstPtr = rgbaImage->getPointer(0);
  for(size_t i = 0; i < numPixels; i++)
  {
    dstPtr[i * 4 + 0] = srcPtr[i * 3 + 0];
    dstPtr[i * 4 + 1] = srcPtr[i * 3 + 1];
    dstPtr[i * 4 + 2] = srcPtr[i * 3 + 2];
    dstPtr[i * 4 + 3] = 255;
  }

  canvas_ity::canvas context(canvasDim, canvasDim);
  // Put the existing image onto the canvas
  context.draw_image(rgbaImage->getPointer(0), canvasDim, canvasDim, canvasDim * 4, 0.0f, 0.0f, static_cast<float>(canvasDim), static_cast<float>(canvasDim));

  // Color bar dimensions — positioned in the right margin area
  // Compute the figure right edge using the same layout as annotateIPFImage with hasColorBar=true
  float rightMargin = static_cast<float>(canvasDim / 3.5f);
  float leftMargin = static_cast<float>(canvasDim / 7.0f);
  float topMargin = fontPtSize * 3;
  float bottomMargin = fontPtSize * 2;
  int legendHeight = canvasDim - static_cast<int>(topMargin) - static_cast<int>(bottomMargin);
  int legendWidth = canvasDim - static_cast<int>(rightMargin) - static_cast<int>(leftMargin);
  if(legendHeight > legendWidth)
  {
    legendHeight = legendWidth;
  }
  float figureRightEdge = leftMargin + static_cast<float>(legendWidth);

  const float barLeft = figureRightEdge + fontPtSize * 2.5f;
  const float barTop = topMargin * 1.33f;
  const float barWidth = fontPtSize * 0.8f;
  const float barHeight = static_cast<float>(legendHeight) * 0.75f;

  // Draw color bar segments
  int colorSegments = numColors;
  float segmentHeight = barHeight / static_cast<float>(colorSegments);
  for(int i = 0; i < colorSegments; i++)
  {
    // Map from top (max) to bottom (min)
    int colorIdx = (colorSegments - 1 - i) * 3;
    float r = colors[colorIdx + 0];
    float g = colors[colorIdx + 1];
    float b = colors[colorIdx + 2];

    float segTop = barTop + static_cast<float>(i) * segmentHeight;
    context.begin_path();
    context.move_to(barLeft, segTop);
    context.line_to(barLeft + barWidth, segTop);
    context.line_to(barLeft + barWidth, segTop + segmentHeight);
    context.line_to(barLeft, segTop + segmentHeight);
    context.close_path();
    context.set_color(canvas_ity::fill_style, r, g, b, 1.0f);
    context.fill();
  }

  // Draw border around color bar
  context.begin_path();
  context.move_to(barLeft, barTop);
  context.line_to(barLeft + barWidth, barTop);
  context.line_to(barLeft + barWidth, barTop + barHeight);
  context.line_to(barLeft, barTop + barHeight);
  context.close_path();
  context.set_color(canvas_ity::stroke_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.set_line_width(1.0f);
  context.stroke();

  // Draw min/max labels
  std::vector<unsigned char> latoRegular = ebsdlib::fonts::GetLatoRegular();
  context.set_font(latoRegular.data(), static_cast<int>(latoRegular.size()), fontPtSize * 0.8f);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);

  // Format min/max values
  std::ostringstream maxStr;
  maxStr << std::fixed << std::setprecision(2) << maxValue;
  std::ostringstream minStr;
  minStr << std::fixed << std::setprecision(2) << minValue;

  float labelX = barLeft + barWidth + fontPtSize * 0.3f;
  ebsdlib::WriteText(context, maxStr.str(), {labelX, barTop + fontPtSize * 0.3f}, fontPtSize * 0.8f);
  ebsdlib::WriteText(context, minStr.str(), {labelX, barTop + barHeight}, fontPtSize * 0.8f);

  // Draw MRD or counts label
  std::string unitLabel = isMRD ? "MRD" : "Counts";
  std::vector<unsigned char> latoBold = ebsdlib::fonts::GetLatoBold();
  context.set_font(latoBold.data(), static_cast<int>(latoBold.size()), fontPtSize * 0.7f);
  ebsdlib::WriteText(context, unitLabel, {barLeft, barTop - fontPtSize * 0.5f}, fontPtSize * 0.7f);

  // Extract and remove alpha
  ebsdlib::UInt8ArrayType::Pointer outRgba = ebsdlib::UInt8ArrayType::CreateArray(numPixels, {4ULL}, "ColorBarOutput", true);
  context.get_image_data(outRgba->getPointer(0), canvasDim, canvasDim, canvasDim * 4, 0, 0);

  return ebsdlib::RemoveAlphaChannel(outRgba.get());
}

// -----------------------------------------------------------------------------
std::vector<UInt8ArrayType::Pointer> LaueOps::generateAnnotatedIPFDensity(InversePoleFigureConfiguration_t& config, std::pair<double, double>* outMinMax) const
{
  // Validate square images
  if(config.imageWidth != config.imageHeight)
  {
    throw std::runtime_error("generateAnnotatedIPFDensity requires square images (imageWidth == imageHeight).");
  }

  const int imageDim = config.imageWidth;
  const int canvasDim = static_cast<int>(static_cast<float>(imageDim) * 1.5f);

  // Determine labels
  std::string label0 = "IPF-0";
  std::string label1 = "IPF-1";
  std::string label2 = "IPF-2";
  if(config.labels.size() >= 1)
  {
    label0 = config.labels[0];
  }
  if(config.labels.size() >= 2)
  {
    label1 = config.labels[1];
  }
  if(config.labels.size() >= 3)
  {
    label2 = config.labels[2];
  }

  // Step 1: Compute IPF directions for each sample direction
  ebsdlib::FloatArrayType::Pointer dirs0 = InversePoleFigureUtilities::computeIPFDirections(*this, config.eulers, config.sampleDirections[0]);
  ebsdlib::FloatArrayType::Pointer dirs1 = InversePoleFigureUtilities::computeIPFDirections(*this, config.eulers, config.sampleDirections[1]);
  ebsdlib::FloatArrayType::Pointer dirs2 = InversePoleFigureUtilities::computeIPFDirections(*this, config.eulers, config.sampleDirections[2]);

  // Step 2: Compute intensity images (using stereographic SST mapping)
  ebsdlib::DoubleArrayType::Pointer intensity0 = InversePoleFigureUtilities::computeIPFIntensity(*this, dirs0.get(), imageDim, imageDim, config.lambertDim, config.normalizeMRD, true);
  ebsdlib::DoubleArrayType::Pointer intensity1 = InversePoleFigureUtilities::computeIPFIntensity(*this, dirs1.get(), imageDim, imageDim, config.lambertDim, config.normalizeMRD, true);
  ebsdlib::DoubleArrayType::Pointer intensity2 = InversePoleFigureUtilities::computeIPFIntensity(*this, dirs2.get(), imageDim, imageDim, config.lambertDim, config.normalizeMRD, true);

  // Step 3: Find global min/max
  double globalMax = std::numeric_limits<double>::lowest();
  double globalMin = std::numeric_limits<double>::max();

  std::array<ebsdlib::DoubleArrayType*, 3> intensities = {intensity0.get(), intensity1.get(), intensity2.get()};
  for(auto* intensityArr : intensities)
  {
    double* dPtr = intensityArr->getPointer(0);
    size_t count = intensityArr->getNumberOfTuples();
    for(size_t i = 0; i < count; ++i)
    {
      if(dPtr[i] >= 0.0)
      {
        if(dPtr[i] > globalMax)
        {
          globalMax = dPtr[i];
        }
        if(dPtr[i] < globalMin)
        {
          globalMin = dPtr[i];
        }
      }
    }
  }

  if(globalMax < globalMin)
  {
    globalMin = 0.0;
    globalMax = 1.0;
  }

  if(outMinMax != nullptr)
  {
    *outMinMax = {globalMin, globalMax};
  }

  // Step 4: Create RGBA color images
  std::vector<size_t> dims = {4};
  ebsdlib::UInt8ArrayType::Pointer image0 = ebsdlib::UInt8ArrayType::CreateArray(static_cast<size_t>(imageDim * imageDim), dims, label0, true);
  ebsdlib::UInt8ArrayType::Pointer image1 = ebsdlib::UInt8ArrayType::CreateArray(static_cast<size_t>(imageDim * imageDim), dims, label1, true);
  ebsdlib::UInt8ArrayType::Pointer image2 = ebsdlib::UInt8ArrayType::CreateArray(static_cast<size_t>(imageDim * imageDim), dims, label2, true);

  InversePoleFigureUtilities::createIPFColorImage(intensity0.get(), imageDim, imageDim, config.numColors, globalMin, globalMax, image0.get());
  InversePoleFigureUtilities::createIPFColorImage(intensity1.get(), imageDim, imageDim, config.numColors, globalMin, globalMax, image1.get());
  InversePoleFigureUtilities::createIPFColorImage(intensity2.get(), imageDim, imageDim, config.numColors, globalMin, globalMax, image2.get());

  // Step 5: Build title strings
  std::string titlePrefix = config.phaseName.empty() ? "" : config.phaseName + " - ";

  // Step 6: Annotate each image. Forward config.hexConvention so the
  // Miller-index labels drawn around each SST honor the caller's choice
  // (PR 2k); without this, hex/trig IPF density images silently render
  // labels under the default convention regardless of caller intent.
  UInt8ArrayType::Pointer annotated0 = annotateIPFImage(image0, imageDim, canvasDim, titlePrefix + label0, false, true, config.hexConvention);
  UInt8ArrayType::Pointer annotated1 = annotateIPFImage(image1, imageDim, canvasDim, titlePrefix + label1, false, true, config.hexConvention);
  UInt8ArrayType::Pointer annotated2 = annotateIPFImage(image2, imageDim, canvasDim, titlePrefix + label2, false, true, config.hexConvention);

  // Step 7: Add color bars
  annotated0 = drawColorBar(annotated0, canvasDim, config.numColors, globalMin, globalMax, config.normalizeMRD);
  annotated1 = drawColorBar(annotated1, canvasDim, config.numColors, globalMin, globalMax, config.normalizeMRD);
  annotated2 = drawColorBar(annotated2, canvasDim, config.numColors, globalMin, globalMax, config.normalizeMRD);

  return {annotated0, annotated1, annotated2};
}
