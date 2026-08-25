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

#include "RandomAngleDistribution.h"

#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Math/EbsdLibMath.h"

#include <cmath>
#include <stdexcept>

namespace
{
// -----------------------------------------------------------------------------
// Direct ports of the helper functions in MTEX 6.1.0's
// geometry/@symmetry/calcAngleDistribution.m (lines 231-255).

// the area of the spherical triangle; alpha, beta, gamma are angles between vertices
double C(double alpha, double beta, double gamma)
{
  return std::acos((std::cos(gamma) - std::cos(alpha) * std::cos(beta)) / (std::sin(alpha) * std::sin(beta)));
}

// the area of a spherical cap
double S1(double rho)
{
  return 2.0 * ebsdlib::constants::k_PiD * (1.0 - std::cos(rho));
}

// area of the intersection of two spherical caps; rho1, rho2 are radii of the
// caps, xi is the distance between the centers of the caps
double S2(double rho1, double rho2, double xi)
{
  return 2.0 * (ebsdlib::constants::k_PiD - C(rho1, rho2, xi) - std::cos(rho1) * C(xi, rho1, rho2) - std::cos(rho2) * C(rho2, xi, rho1));
}

// Cn branch (calcAngleDistribution.m lines 58-63): nfold is the crystal's
// rotational symmetry order about its principal axis.
double ChiCn(double rmag, uint32_t nfold)
{
  const double xhn = std::tan(ebsdlib::constants::k_PiD / 2.0 / static_cast<double>(nfold));
  double xchi = 1.0;
  if(rmag > xhn)
  {
    xchi = xhn / rmag;
  }
  return xchi;
}

// Dnh branch (calcAngleDistribution.m lines 64-83), built on top of the Cn
// first region.
double ChiDnh(double rmag, uint32_t nfold)
{
  const double xhn = std::tan(ebsdlib::constants::k_PiD / 2.0 / static_cast<double>(nfold));
  double xchi = ChiCn(rmag, nfold);

  if(rmag > 1.0)
  {
    xchi += static_cast<double>(nfold) * (1.0 / rmag - 1.0);
  }

  const double xedge = std::sqrt(1.0 + xhn * xhn);
  if(rmag > xedge)
  {
    const double alpha1 = std::acos(xhn / rmag);
    const double alpha2 = std::acos(1.0 / rmag);
    const double xs21 = S2(alpha1, alpha2, ebsdlib::constants::k_PiD / 2.0);
    const double xs22 = S2(alpha2, alpha2, ebsdlib::constants::k_PiD / static_cast<double>(nfold));
    xchi += static_cast<double>(nfold) * xs21 / ebsdlib::constants::k_PiD + static_cast<double>(nfold) * xs22 / (2.0 * ebsdlib::constants::k_PiD);
  }

  return xchi;
}

// m-3 branch (calcAngleDistribution.m lines 85-96).
double ChiM3(double rmag)
{
  double xchi = 1.0;

  // first region
  const double xh3 = std::sqrt(3.0) / 3.0;
  if(rmag > xh3)
  {
    xchi = 4.0 * xh3 / rmag - 3.0;
  }

  // second region
  const double xedge = std::sqrt(2.0) / 2.0;
  if(rmag > xedge)
  {
    const double alpha = std::acos(xh3 / rmag);
    xchi += 3.0 * S2(alpha, alpha, std::acos(1.0 / 3.0)) / ebsdlib::constants::k_PiD;
  }

  return xchi;
}

// m-3m branch (calcAngleDistribution.m lines 98-117).
double ChiM3m(double rmag)
{
  double xchi = 1.0;

  // first region -> four fold axis active
  const double xh4 = std::sqrt(2.0) - 1.0;
  if(rmag > xh4)
  {
    xchi = 3.0 * xh4 / rmag - 2.0;
  }

  // second region -> three fold axis active
  const double xh3 = std::sqrt(3.0) / 3.0;
  if(rmag > xh3)
  {
    xchi += 4.0 * (xh3 / rmag - 1.0);
  }

  // third region
  const double xedge = 2.0 - std::sqrt(2.0);
  if(rmag > xedge)
  {
    const double alpha1 = std::acos(xh4 / rmag);
    const double alpha2 = std::acos(xh3 / rmag);
    const double s12 = S2(alpha1, alpha1, ebsdlib::constants::k_PiD / 2.0);
    const double s24 = S2(alpha1, alpha2, std::acos(xh3));
    xchi += 3.0 * s12 / ebsdlib::constants::k_PiD + 6.0 * s24 / ebsdlib::constants::k_PiD;
  }

  return xchi;
}
} // namespace

namespace ebsdlib
{
namespace random_angle_distribution
{
double MaxMisorientationAngle(uint32_t crystalStructure)
{
  switch(crystalStructure)
  {
  case CrystalStructure::Hexagonal_High:
    return 1.637833825000;
  case CrystalStructure::Cubic_High:
    return 1.096056815241;
  case CrystalStructure::Hexagonal_Low:
    return 3.141592653590;
  case CrystalStructure::Cubic_Low:
    return 1.570796326795;
  case CrystalStructure::Triclinic:
    return 3.141592653590;
  case CrystalStructure::Monoclinic:
    return 3.141592653590;
  case CrystalStructure::OrthoRhombic:
    return 2.094395102393;
  case CrystalStructure::Tetragonal_Low:
    return 3.141592653590;
  case CrystalStructure::Tetragonal_High:
    return 1.717771517458;
  case CrystalStructure::Trigonal_Low:
    return 3.141592653590;
  case CrystalStructure::Trigonal_High:
    return 1.823476581937;
  default:
    throw std::invalid_argument("random_angle_distribution::MaxMisorientationAngle: unknown crystal structure");
  }
}

std::vector<double> Compute(uint32_t crystalStructure, const std::vector<double>& omega)
{
  const double maxAngle = MaxMisorientationAngle(crystalStructure);

  std::vector<double> ad(omega.size(), 0.0);

  for(size_t i = 0; i < omega.size(); i++)
  {
    if(omega[i] > maxAngle)
    {
      ad[i] = 0.0;
      continue;
    }

    // omega/2 is in [0, maxAngle/2] with maxAngle <= pi, so mathematically
    // rmag >= 0; std::abs guards the omega==maxAngle==pi boundary, where
    // omega/2 sits right at tan()'s pole at pi/2 and floating-point rounding
    // can push it a hair past the pole, flipping tan()'s sign.
    const double rmag = std::abs(std::tan(omega[i] / 2.0));
    double xchi = 1.0;

    switch(crystalStructure)
    {
    case CrystalStructure::Hexagonal_Low: // 6/m
      xchi = ChiCn(rmag, 6);
      break;
    case CrystalStructure::Tetragonal_Low: // 4/m
      xchi = ChiCn(rmag, 4);
      break;
    case CrystalStructure::Trigonal_Low: // -3
      xchi = ChiCn(rmag, 3);
      break;
    case CrystalStructure::Monoclinic: // 2/m
      xchi = ChiCn(rmag, 2);
      break;
    case CrystalStructure::Hexagonal_High: // 6/mmm
      xchi = ChiDnh(rmag, 6);
      break;
    case CrystalStructure::Tetragonal_High: // 4/mmm
      xchi = ChiDnh(rmag, 4);
      break;
    case CrystalStructure::Trigonal_High: // -3m
      xchi = ChiDnh(rmag, 3);
      break;
    case CrystalStructure::OrthoRhombic: // mmm
      xchi = ChiDnh(rmag, 2);
      break;
    case CrystalStructure::Cubic_Low: // m-3
      xchi = ChiM3(rmag);
      break;
    case CrystalStructure::Cubic_High: // m-3m
      xchi = ChiM3m(rmag);
      break;
    case CrystalStructure::Triclinic: // -1
      xchi = 1.0;
      break;
    default:
      throw std::invalid_argument("random_angle_distribution::Compute: unknown crystal structure");
    }

    // MTEX's `2 * numSym(cs)` prefactor is dropped here: it is a constant
    // multiplier that cancels out in the unit-mean normalization below.
    ad[i] = xchi * std::sin(omega[i] / 2.0) * std::sin(omega[i] / 2.0);
  }

  double sum = 0.0;
  size_t count = 0;
  for(size_t i = 0; i < omega.size(); i++)
  {
    if(omega[i] <= maxAngle)
    {
      sum += ad[i];
      count++;
    }
  }
  const double mean = (count > 0) ? (sum / static_cast<double>(count)) : 0.0;

  for(size_t i = 0; i < ad.size(); i++)
  {
    if(omega[i] > maxAngle || mean == 0.0)
    {
      ad[i] = 0.0;
      continue;
    }
    ad[i] /= mean;
    if(ad[i] < 0.0)
    {
      ad[i] = 0.0;
    }
  }

  return ad;
}
} // namespace random_angle_distribution
} // namespace ebsdlib
