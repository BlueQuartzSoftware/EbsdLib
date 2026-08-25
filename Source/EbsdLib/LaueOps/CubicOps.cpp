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

#include "CubicOps.h"

// Include this FIRST because there is a needed define for some compiles
// to expose some of the constants needed below
#include "EbsdLib/Core/EbsdMacros.h"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Orientation/Euler.hpp"
#include "EbsdLib/Orientation/OrientationFwd.hpp"
#include "EbsdLib/Orientation/OrientationMatrix.hpp"
#include "EbsdLib/Orientation/Quaternion.hpp"
#include "EbsdLib/Orientation/Rodrigues.hpp"
#include "EbsdLib/Utilities/CanvasUtilities.hpp"
#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/ComputeStereographicProjection.h"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"
#include "EbsdLib/Utilities/Fonts.hpp"
#include "EbsdLib/Utilities/FundamentalSectorGeometry.hpp"
#include "EbsdLib/Utilities/GriddedColorKey.hpp"
#include "EbsdLib/Utilities/NolzeHielscherColorKey.hpp"
#include "EbsdLib/Utilities/PUCMColorKey.hpp"
#include "EbsdLib/Utilities/TSLColorKey.hpp"

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/task_group.h>
#endif
using namespace ebsdlib;

namespace
{
// Per-class color-key singletons. Each LaueOps subclass uses its own point group
// for PUCM and its own FundamentalSectorGeometry for Nolze-Hielscher; CubicOps
// is 432 / cubicHigh.
ebsdlib::IColorKey::Pointer keyForKind(ebsdlib::ColorKeyKind kind)
{
  static const auto k_TSL = std::make_shared<ebsdlib::TSLColorKey>();
  static const auto k_PUCM = std::make_shared<ebsdlib::PUCMColorKey>("432");
  static const auto k_NH = std::make_shared<ebsdlib::NolzeHielscherColorKey>(ebsdlib::FundamentalSectorGeometry::cubicHigh());
  switch(kind)
  {
  case ebsdlib::ColorKeyKind::PUCM:
    return k_PUCM;
  case ebsdlib::ColorKeyKind::NolzeHielscher:
    return k_NH;
  case ebsdlib::ColorKeyKind::TSL:
    break;
  }
  return k_TSL;
}
} // namespace

namespace CubicHigh
{

constexpr std::array<size_t, 3> k_OdfNumBins = {18, 18, 18}; // Represents a 5Deg bin in homochoric space
static const std::array<double, 3> k_OdfDimInitValue = {std::pow((0.75 * (ebsdlib::constants::k_PiOver4D - std::sin(ebsdlib::constants::k_PiOver4D))), (1.0 / 3.0)),
                                                        std::pow((0.75 * (ebsdlib::constants::k_PiOver4D - std::sin(ebsdlib::constants::k_PiOver4D))), (1.0 / 3.0)),
                                                        std::pow((0.75 * (ebsdlib::constants::k_PiOver4D - std::sin(ebsdlib::constants::k_PiOver4D))), (1.0 / 3.0))};

static const std::array<double, 3> k_OdfDimStepValue = {k_OdfDimInitValue[0] / static_cast<double>(k_OdfNumBins[0] / 2), k_OdfDimInitValue[1] / static_cast<double>(k_OdfNumBins[1] / 2),
                                                        k_OdfDimInitValue[2] / static_cast<double>(k_OdfNumBins[2] / 2)};

constexpr int k_SymSize0 = 6;
constexpr int k_SymSize1 = 12;
constexpr int k_SymSize2 = 8;

constexpr size_t k_OdfSize = 5832;
constexpr size_t k_MdfSize = 5832;
constexpr size_t k_SymOpsCount = 24;
constexpr int k_NumMdfBins = 13;

static const double SlipDirections[12][3] = {{0.0, 1.0, -1.0}, {1.0, 0.0, -1.0}, {1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {1.0, 0.0, 1.0}, {0.0, 1.0, 1.0},
                                             {1.0, 1.0, 0.0},  {0.0, 1.0, 1.0},  {1.0, 0.0, -1.0}, {1.0, 1.0, 0.0},  {1.0, 0.0, 1.0}, {0.0, 1.0, -1.0}};

static const double SlipPlanes[12][3] = {{1.0, 1.0, 1.0},  {1.0, 1.0, 1.0},  {1.0, 1.0, 1.0},  {1.0, 1.0, -1.0}, {1.0, 1.0, -1.0}, {1.0, 1.0, -1.0},
                                         {1.0, -1.0, 1.0}, {1.0, -1.0, 1.0}, {1.0, -1.0, 1.0}, {-1.0, 1.0, 1.0}, {-1.0, 1.0, 1.0}, {-1.0, 1.0, 1.0}};

static const double sqrtHalf = std::sqrt(0.5);
static const double sqrtOneThird = std::sqrt(0.3333333333333333333);
static const double sqrtThree = std::sqrt(3.0);

// Rotation Point Group: 432
// Operator ordering matches EMsoftOO: identity, 90deg, 270deg, 120deg, 180deg-axis, 180deg-diagonal
/* clang-format off */
static const std::vector<QuatD> k_QuatSym ={
    QuatD(0.0, 0.0, 0.0, 1.0),
    QuatD(sqrtHalf, 0.0, 0.0, sqrtHalf),
    QuatD(0.0, sqrtHalf, 0.0, sqrtHalf),
    QuatD(0.0, 0.0, sqrtHalf, sqrtHalf),
    QuatD(-sqrtHalf, 0.0, 0.0, sqrtHalf),
    QuatD(0.0, -sqrtHalf, 0.0, sqrtHalf),
    QuatD(0.0, 0.0, -sqrtHalf, sqrtHalf),
    QuatD(0.5, 0.5, 0.5, 0.5),
    QuatD(-0.5, -0.5, -0.5, 0.5),
    QuatD(0.5, -0.5, 0.5, 0.5),
    QuatD(-0.5, 0.5, -0.5, 0.5),
    QuatD(-0.5, 0.5, 0.5, 0.5),
    QuatD(0.5, -0.5, -0.5, 0.5),
    QuatD(-0.5, -0.5, 0.5, 0.5),
    QuatD(0.5, 0.5, -0.5, 0.5),
    QuatD(1.0, 0.0, 0.0, 0.0),
    QuatD(0.0, 1.0, 0.0, 0.0),
    QuatD(0.0, 0.0, 1.0, 0.0),
    QuatD(sqrtHalf, sqrtHalf, 0.0, 0.0),
    QuatD(-sqrtHalf, sqrtHalf, 0.0, 0.0),
    QuatD(0.0, sqrtHalf, sqrtHalf, 0.0),
    QuatD(0.0, -sqrtHalf, sqrtHalf, 0.0),
    QuatD(sqrtHalf, 0.0, sqrtHalf, 0.0),
    QuatD(-sqrtHalf, 0.0, sqrtHalf, 0.0),
};

static const std::vector<RodriguesDType> k_RodSym = {
    {0.0, 0.0, 1.0, 0.0},
    {1.0, 0.0, 0.0, 1.0},
    {0.0, 1.0, 0.0, 1.0},
    {0.0, 0.0, 1.0, 1.0},
    {-1.0, 0.0, 0.0, 1.0},
    {0.0, -1.0, 0.0, 1.0},
    {0.0, 0.0, -1.0, 1.0},
    {sqrtOneThird, sqrtOneThird, sqrtOneThird, sqrtThree},
    {-sqrtOneThird, -sqrtOneThird, -sqrtOneThird, sqrtThree},
    {sqrtOneThird, -sqrtOneThird, sqrtOneThird, sqrtThree},
    {-sqrtOneThird, sqrtOneThird, -sqrtOneThird, sqrtThree},
    {-sqrtOneThird, sqrtOneThird, sqrtOneThird, sqrtThree},
    {sqrtOneThird, -sqrtOneThird, -sqrtOneThird, sqrtThree},
    {-sqrtOneThird, -sqrtOneThird, sqrtOneThird, sqrtThree},
    {sqrtOneThird, sqrtOneThird, -sqrtOneThird, sqrtThree},
    {1.0, 0.0, 0.0, 10000000000000.0},
    {0.0, 1.0, 0.0, 10000000000000.0},
    {0.0, 0.0, 1.0, 10000000000000.0},
    {sqrtHalf, sqrtHalf, 0.0, 10000000000000.0},
    {-sqrtHalf, sqrtHalf, 0.0, 10000000000000.0},
    {0.0, sqrtHalf, sqrtHalf, 10000000000000.0},
    {0.0, -sqrtHalf, sqrtHalf, 10000000000000.0},
    {sqrtHalf, 0.0, sqrtHalf, 10000000000000.0},
    {-sqrtHalf, 0.0, sqrtHalf, 10000000000000.0},
};

static const std::vector<Matrix3X3D> k_MatSym = {
    {1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0},

    {1.0, 0.0, 0.0,
    0.0, 0.0, -1.0,
    0.0, 1.0, 0.0},

    {0.0, 0.0, 1.0,
    0.0, 1.0, 0.0,
    -1.0, 0.0, 0.0},

    {0.0, -1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, 1.0},

    {1.0, -0.0, 0.0,
    0.0, 0.0, 1.0,
    -0.0, -1.0, 0.0},

    {0.0, -0.0, -1.0,
    0.0, 1.0, -0.0,
    1.0, 0.0, 0.0},

    {0.0, 1.0, 0.0,
    -1.0, 0.0, -0.0,
    -0.0, 0.0, 1.0},

    {0.0, 0.0, 1.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0},

    {0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
    1.0, 0.0, 0.0},

    {0.0, -1.0, 0.0,
    0.0, 0.0, -1.0,
    1.0, 0.0, 0.0},

    {0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0},

    {0.0, -1.0, 0.0,
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0},

    {0.0, 0.0, -1.0,
    -1.0, 0.0, 0.0,
    0.0, 1.0, 0.0},

    {0.0, 0.0, -1.0,
    1.0, 0.0, 0.0,
    0.0, -1.0, 0.0},

    {0.0, 1.0, 0.0,
    0.0, 0.0, -1.0,
    -1.0, 0.0, 0.0},

    {1.0, 0.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, 0.0, -1.0},

    {-1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, -1.0},

    {-1.0, 0.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, 0.0, 1.0},

    {0.0, 1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, -1.0},

    {0.0, -1.0, 0.0,
    -1.0, 0.0, 0.0,
    -0.0, 0.0, -1.0},

    {-1.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0, 1.0, 0.0},

    {-1.0, -0.0, 0.0,
    0.0, 0.0, -1.0,
    0.0, -1.0, 0.0},

    {0.0, 0.0, 1.0,
    0.0, -1.0, 0.0,
    1.0, 0.0, 0.0},

    {0.0, -0.0, -1.0,
    0.0, -1.0, 0.0,
    -1.0, 0.0, 0.0},

};
/* clang-format on */

constexpr double k_EtaMin = 0.0;
constexpr double k_EtaMax = 45.0;

} // namespace CubicHigh

// -----------------------------------------------------------------------------
CubicOps::CubicOps() = default;

// -----------------------------------------------------------------------------
CubicOps::~CubicOps() = default;

// -----------------------------------------------------------------------------
bool CubicOps::getHasInversion() const
{
  return true;
}

// -----------------------------------------------------------------------------
size_t CubicOps::getODFSize() const
{
  return CubicHigh::k_OdfSize;
}

// -----------------------------------------------------------------------------
std::array<int32_t, 3> CubicOps::getNumSymmetry() const
{
  return {CubicHigh::k_SymSize0, CubicHigh::k_SymSize1, CubicHigh::k_SymSize2};
}

// -----------------------------------------------------------------------------
size_t CubicOps::getMDFSize() const
{
  return CubicHigh::k_MdfSize;
}

// -----------------------------------------------------------------------------
int CubicOps::getMdfPlotBins() const
{
  return CubicHigh::k_NumMdfBins;
}

// -----------------------------------------------------------------------------
size_t CubicOps::getNumSymOps() const
{
  return CubicHigh::k_SymOpsCount;
}

// -----------------------------------------------------------------------------
std::array<size_t, 3> CubicOps::getOdfNumBins() const
{
  return CubicHigh::k_OdfNumBins;
}

// -----------------------------------------------------------------------------
std::string CubicOps::getSymmetryName() const
{
  return "Cubic m-3m (Oh)"; /* Group 432 */
}

// -----------------------------------------------------------------------------
std::string CubicOps::getRotationPointGroup() const
{
  return "432";
}

// -----------------------------------------------------------------------------
int CubicOps::getPointGroup() const
{
  return 32;
}

// -----------------------------------------------------------------------------
bool CubicOps::isInsideFZ(const QuatD& quat) const
{
  return IsInsideFZ(quat, getFZType(), getAxisOrderingType());
}

// -----------------------------------------------------------------------------
bool CubicOps::isInsideFZ(const RodriguesDType& rod) const
{
  return IsInsideFZ(rod, getFZType(), getAxisOrderingType());
}

// -----------------------------------------------------------------------------
AxisAngleDType CubicOps::calculateMisorientation(const QuatD& q1, const QuatD& q2) const
{
  return calculateMisorientationInternal(CubicHigh::k_QuatSym, q1, q2);
}

// -----------------------------------------------------------------------------
AxisAngleDType CubicOps::calculateMisorientationInternal(const std::vector<QuatD>& quatsym, const QuatD& q1, const QuatD& q2) const
{
  double wmin = 9999999.0f; //,na,nb,nc;
  QuatD qco;
  int type = 1;
  double sin_wmin_over_2 = 0.0;

  QuatD qc = q1 * (q2.conjugate());
  qc.elementWiseAbs();

  // if qc.x() is smallest
  if(qc.x() <= qc.y() && qc.x() <= qc.z() && qc.x() <= qc.w())
  {
    qco.x() = qc.x();
    // if qc.y() is next smallest
    if(qc.y() <= qc.z() && qc.y() <= qc.w())
    {
      qco.y() = qc.y();
      if(qc.z() <= qc.w())
      {
        qco.z() = qc.z(), qco.w() = qc.w();
      }
      else
      {
        qco.z() = qc.w(), qco.w() = qc.z();
      }
    }
    // if qc.z() is next smallest
    else if(qc.z() <= qc.y() && qc.z() <= qc.w())
    {
      qco.y() = qc.z();
      if(qc.y() <= qc.w())
      {
        qco.z() = qc.y(), qco.w() = qc.w();
      }
      else
      {
        qco.z() = qc.w(), qco.w() = qc.y();
      }
    }
    // if qc.w() is next smallest
    else
    {
      qco.y() = qc.w();
      if(qc.y() <= qc.z())
      {
        qco.z() = qc.y(), qco.w() = qc.z();
      }
      else
      {
        qco.z() = qc.z(), qco.w() = qc.y();
      }
    }
  }
  // if qc.y() is smallest
  else if(qc.y() <= qc.x() && qc.y() <= qc.z() && qc.y() <= qc.w())
  {
    qco.x() = qc.y();
    // if qc.x() is next smallest
    if(qc.x() <= qc.z() && qc.x() <= qc.w())
    {
      qco.y() = qc.x();
      if(qc.z() <= qc.w())
      {
        qco.z() = qc.z(), qco.w() = qc.w();
      }
      else
      {
        qco.z() = qc.w(), qco.w() = qc.z();
      }
    }
    // if qc.z() is next smallest
    else if(qc.z() <= qc.x() && qc.z() <= qc.w())
    {
      qco.y() = qc.z();
      if(qc.x() <= qc.w())
      {
        qco.z() = qc.x(), qco.w() = qc.w();
      }
      else
      {
        qco.z() = qc.w(), qco.w() = qc.x();
      }
    }
    // if qc.w() is next smallest
    else
    {
      qco.y() = qc.w();
      if(qc.x() <= qc.z())
      {
        qco.z() = qc.x(), qco.w() = qc.z();
      }
      else
      {
        qco.z() = qc.z(), qco.w() = qc.x();
      }
    }
  }
  // if qc.z() is smallest
  else if(qc.z() <= qc.x() && qc.z() <= qc.y() && qc.z() <= qc.w())
  {
    qco.x() = qc.z();
    // if qc.x() is next smallest
    if(qc.x() <= qc.y() && qc.x() <= qc.w())
    {
      qco.y() = qc.x();
      if(qc.y() <= qc.w())
      {
        qco.z() = qc.y(), qco.w() = qc.w();
      }
      else
      {
        qco.z() = qc.w(), qco.w() = qc.y();
      }
    }
    // if qc.y() is next smallest
    else if(qc.y() <= qc.x() && qc.y() <= qc.w())
    {
      qco.y() = qc.y();
      if(qc.x() <= qc.w())
      {
        qco.z() = qc.x(), qco.w() = qc.w();
      }
      else
      {
        qco.z() = qc.w(), qco.w() = qc.x();
      }
    }
    // if qc.w() is next smallest
    else
    {
      qco.y() = qc.w();
      if(qc.x() <= qc.y())
      {
        qco.z() = qc.x(), qco.w() = qc.y();
      }
      else
      {
        qco.z() = qc.y(), qco.w() = qc.x();
      }
    }
  }
  // if qc.w() is smallest
  else
  {
    qco.x() = qc.w();
    // if qc.x() is next smallest
    if(qc.x() <= qc.y() && qc.x() <= qc.z())
    {
      qco.y() = qc.x();
      if(qc.y() <= qc.z())
      {
        qco.z() = qc.y(), qco.w() = qc.z();
      }
      else
      {
        qco.z() = qc.z(), qco.w() = qc.y();
      }
    }
    // if qc.y() is next smallest
    else if(qc.y() <= qc.x() && qc.y() <= qc.z())
    {
      qco.y() = qc.y();
      if(qc.x() <= qc.z())
      {
        qco.z() = qc.x(), qco.w() = qc.z();
      }
      else
      {
        qco.z() = qc.z(), qco.w() = qc.x();
      }
    }
    // if qc.z() is next smallest
    else
    {
      qco.y() = qc.z();
      if(qc.x() <= qc.y())
      {
        qco.z() = qc.x(), qco.w() = qc.y();
      }
      else
      {
        qco.z() = qc.y(), qco.w() = qc.x();
      }
    }
  }
  // Three candidate symmetry-op reductions of qco. For each "type" (sym op class),
  // compute BOTH the candidate cos(half-angle) AND the vector components (vx,vy,vz)
  // of the reduced quaternion explicitly. Choosing the largest cos(half-angle)
  // picks the sym op that minimizes the misorientation angle.
  //
  // The angle is then 2 * atan2(|v|, w) using the EXPLICIT v from the reduced
  // quaternion -- rather than sqrt(1 - w*w) -- which preserves precision through
  // symmetry-op cancellations. When qco encodes a sym op exactly, the v components
  // collapse to subtractions of identical floats (e.g., (qco.z - qco.w) when
  // qco.z == qco.w), which yield exactly 0 in IEEE 754. The sqrt(1 - w*w) form
  // loses this precision because w is computed via sums/divisions that do not
  // preserve the ULP structure of the inputs -- a real concern when AvgQuats
  // is stored as float32 and promoted to double inside the calculation.

  // Type 1: identity (no sym op applied) -- reduced quaternion is qco itself
  double wCand = qco.w();
  double vx = qco.x();
  double vy = qco.y();
  double vz = qco.z();
  type = 1;

  // Type 2: 90 deg about z; sym op (0, 0, 1, 1) / sqrt(2)
  {
    const double w2 = (qco.z() + qco.w()) / ebsdlib::constants::k_Sqrt2D;
    if(w2 > wCand)
    {
      wCand = w2;
      vx = (qco.x() - qco.y()) / ebsdlib::constants::k_Sqrt2D;
      vy = (qco.x() + qco.y()) / ebsdlib::constants::k_Sqrt2D;
      vz = (qco.z() - qco.w()) / ebsdlib::constants::k_Sqrt2D;
      type = 2;
    }
  }

  // Type 3: 120 deg about [1, 1, 1]; sym op (1, 1, 1, 1) / 2
  {
    const double w3 = (qco.x() + qco.y() + qco.z() + qco.w()) / 2.0;
    if(w3 > wCand)
    {
      wCand = w3;
      vx = (qco.x() - qco.y() + qco.z() - qco.w()) / 2.0;
      vy = (qco.x() + qco.y() - qco.z() - qco.w()) / 2.0;
      vz = (-qco.x() + qco.y() + qco.z() - qco.w()) / 2.0;
      type = 3;
    }
  }

  // Stable angle: 2 * atan2(|v|, w). Clamp w only to defend against ULP excess
  // above 1.0 from float roundoff (this does NOT affect the precision-recovery
  // path -- precision is recovered by computing |v| from the explicit v terms).
  sin_wmin_over_2 = std::sqrt(vx * vx + vy * vy + vz * vz);
  const double clampedW = std::clamp(wCand, -1.0, 1.0);
  wmin = 2.0 * std::atan2(sin_wmin_over_2, clampedW);

  // Axis = v / |v|. When |v| == 0 the reduced quaternion is identity (angle 0,
  // axis undefined); use [0, 0, 1] by convention.
  double n1 = 0.0;
  double n2 = 0.0;
  double n3 = 0.0;
  if(sin_wmin_over_2 > 0.0)
  {
    n1 = vx / sin_wmin_over_2;
    n2 = vy / sin_wmin_over_2;
    n3 = vz / sin_wmin_over_2;
  }
  else
  {
    n3 = 1.0;
  }

  AxisAngleDType axisAngle(n1, n2, n3, wmin);
  return axisAngle;
}

QuatD CubicOps::getQuatSymOp(size_t i) const
{
  return CubicHigh::k_QuatSym[i];
}

size_t CubicOps::getNumRodriguesSymOps() const
{
  return CubicHigh::k_RodSym.size();
}

RodriguesDType CubicOps::getRodSymOp(size_t i) const
{
  return CubicHigh::k_RodSym[i];
}

Matrix3X3D CubicOps::getMatSymOpD(size_t i) const
{
  return CubicHigh::k_MatSym[i];
}

Matrix3X3F CubicOps::getMatSymOpF(size_t i) const
{
  return {static_cast<float>(CubicHigh::k_MatSym[i](0, 0)), static_cast<float>(CubicHigh::k_MatSym[i](0, 1)), static_cast<float>(CubicHigh::k_MatSym[i](0, 2)),
          static_cast<float>(CubicHigh::k_MatSym[i](1, 0)), static_cast<float>(CubicHigh::k_MatSym[i](1, 1)), static_cast<float>(CubicHigh::k_MatSym[i](1, 2)),
          static_cast<float>(CubicHigh::k_MatSym[i](2, 0)), static_cast<float>(CubicHigh::k_MatSym[i](2, 1)), static_cast<float>(CubicHigh::k_MatSym[i](2, 2))};
}

// -----------------------------------------------------------------------------
RodriguesDType CubicOps::getODFFZRod(const RodriguesDType& rod) const
{
  return _calcRodNearestOrigin(rod);
}

// -----------------------------------------------------------------------------
RodriguesDType CubicOps::getMDFFZRod(const RodriguesDType& inRod) const
{
  double w, n1, n2, n3;
  double FZw, FZn1, FZn2, FZn3;

  RodriguesDType rod = _calcRodNearestOrigin(inRod);
  AxisAngleDType ax = rod.toAxisAngle();

  n1 = ax[0];
  n2 = ax[1], n3 = ax[2], w = ax[3];

  FZw = w;
  n1 = std::fabs(n1);
  n2 = std::fabs(n2);
  n3 = std::fabs(n3);
  if(n1 > n2)
  {
    if(n1 > n3)
    {
      FZn1 = n1;
      if(n2 > n3)
      {
        FZn2 = n2, FZn3 = n3;
      }
      else
      {
        FZn2 = n3, FZn3 = n2;
      }
    }
    else
    {
      FZn1 = n3, FZn2 = n1, FZn3 = n2;
    }
  }
  else
  {
    if(n2 > n3)
    {
      FZn1 = n2;
      if(n1 > n3)
      {
        FZn2 = n1, FZn3 = n3;
      }
      else
      {
        FZn2 = n3, FZn3 = n1;
      }
    }
    else
    {
      FZn1 = n3, FZn2 = n2, FZn3 = n1;
    }
  }

  return AxisAngleDType(FZn1, FZn2, FZn3, FZw).toRodrigues();
}

QuatD CubicOps::getNearestQuat(const QuatD& q1, const QuatD& q2) const
{
  return _calcNearestQuat(CubicHigh::k_QuatSym, q1, q2);
}

QuatF CubicOps::getNearestQuat(const QuatF& q1f, const QuatF& q2f) const
{
  return _calcNearestQuat(CubicHigh::k_QuatSym, q1f.to<double>(), q2f.to<double>()).to<float>();
}

QuatD CubicOps::getFZQuat(const QuatD& qr) const
{
  LaueOps::FZType fzType = laue_ops::FZtarray[getPointGroup() - 1];
  LaueOps::AxisOrderingType orderingType = laue_ops::FZoarray[getPointGroup() - 1];
  return ConvertToFZ(CubicHigh::k_QuatSym, qr, fzType, orderingType);
}

// -----------------------------------------------------------------------------
int CubicOps::getMisoBin(const RodriguesDType& rod) const
{
  double dim[3];
  double bins[3];
  double step[3];

  HomochoricDType ho = rod.toHomochoric();

  dim[0] = CubicHigh::k_OdfDimInitValue[0];
  dim[1] = CubicHigh::k_OdfDimInitValue[1];
  dim[2] = CubicHigh::k_OdfDimInitValue[2];
  step[0] = CubicHigh::k_OdfDimStepValue[0];
  step[1] = CubicHigh::k_OdfDimStepValue[1];
  step[2] = CubicHigh::k_OdfDimStepValue[2];
  bins[0] = static_cast<double>(CubicHigh::k_OdfNumBins[0]);
  bins[1] = static_cast<double>(CubicHigh::k_OdfNumBins[1]);
  bins[2] = static_cast<double>(CubicHigh::k_OdfNumBins[2]);

  return _calcMisoBin(dim, bins, step, ho);
}

// -----------------------------------------------------------------------------
EulerDType CubicOps::determineEulerAngles(double random[3], int choose) const
{
  double init[3];
  double step[3];
  int32_t phi[3];
  double h1, h2, h3;

  init[0] = CubicHigh::k_OdfDimInitValue[0];
  init[1] = CubicHigh::k_OdfDimInitValue[1];
  init[2] = CubicHigh::k_OdfDimInitValue[2];
  step[0] = CubicHigh::k_OdfDimStepValue[0];
  step[1] = CubicHigh::k_OdfDimStepValue[1];
  step[2] = CubicHigh::k_OdfDimStepValue[2];
  phi[0] = static_cast<int32_t>(choose % CubicHigh::k_OdfNumBins[0]);
  phi[1] = static_cast<int32_t>((choose / CubicHigh::k_OdfNumBins[0]) % CubicHigh::k_OdfNumBins[1]);
  phi[2] = static_cast<int32_t>(choose / (CubicHigh::k_OdfNumBins[0] * CubicHigh::k_OdfNumBins[1]));

  _calcDetermineHomochoricValues(random, init, step, phi, h1, h2, h3);

  RodriguesDType ro = HomochoricDType(h1, h2, h3).toRodrigues();
  ro = getODFFZRod(ro);
  EulerDType eu = ro.toEuler();
  return eu;
}

// -----------------------------------------------------------------------------
EulerDType CubicOps::randomizeEulerAngles(const EulerDType& synea) const
{
  size_t symOp = getRandomSymmetryOperatorIndex(CubicHigh::k_SymOpsCount);
  QuatD quat = synea.toQuaternion();
  QuatD qc = CubicHigh::k_QuatSym[symOp] * quat;
  return QuaternionDType(qc).toEuler();
}

// -----------------------------------------------------------------------------
RodriguesDType CubicOps::determineRodriguesVector(double random[3], int choose) const
{
  double init[3];
  double step[3];
  int32_t phi[3];
  double h1, h2, h3;

  init[0] = CubicHigh::k_OdfDimInitValue[0];
  init[1] = CubicHigh::k_OdfDimInitValue[1];
  init[2] = CubicHigh::k_OdfDimInitValue[2];
  step[0] = CubicHigh::k_OdfDimStepValue[0];
  step[1] = CubicHigh::k_OdfDimStepValue[1];
  step[2] = CubicHigh::k_OdfDimStepValue[2];
  phi[0] = static_cast<int32_t>(choose % CubicHigh::k_OdfNumBins[0]);
  phi[1] = static_cast<int32_t>((choose / CubicHigh::k_OdfNumBins[0]) % CubicHigh::k_OdfNumBins[1]);
  phi[2] = static_cast<int32_t>(choose / (CubicHigh::k_OdfNumBins[0] * CubicHigh::k_OdfNumBins[1]));

  _calcDetermineHomochoricValues(random, init, step, phi, h1, h2, h3);
  RodriguesDType ro = HomochoricDType(h1, h2, h3).toRodrigues();
  ro = getMDFFZRod(ro);
  return ro;
}

// -----------------------------------------------------------------------------
int CubicOps::getOdfBin(const RodriguesDType& rod) const
{
  double dim[3];
  double bins[3];
  double step[3];

  HomochoricDType ho = rod.toHomochoric();

  dim[0] = CubicHigh::k_OdfDimInitValue[0];
  dim[1] = CubicHigh::k_OdfDimInitValue[1];
  dim[2] = CubicHigh::k_OdfDimInitValue[2];
  step[0] = CubicHigh::k_OdfDimStepValue[0];
  step[1] = CubicHigh::k_OdfDimStepValue[1];
  step[2] = CubicHigh::k_OdfDimStepValue[2];
  bins[0] = static_cast<double>(CubicHigh::k_OdfNumBins[0]);
  bins[1] = static_cast<double>(CubicHigh::k_OdfNumBins[1]);
  bins[2] = static_cast<double>(CubicHigh::k_OdfNumBins[2]);

  return _calcODFBin(dim, bins, step, ho);
}

void CubicOps::getSchmidFactorAndSS(double load[3], double& schmidfactor, double angleComps[2], int& slipsys) const
{
  schmidfactor = 0.0;
  double theta1, theta2, theta3, theta4;
  double lambda1, lambda2, lambda3, lambda4, lambda5, lambda6;
  double schmid1, schmid2, schmid3, schmid4, schmid5, schmid6, schmid7, schmid8, schmid9, schmid10, schmid11, schmid12;

  double loadx = load[0];
  double loady = load[1];
  double loadz = load[2];

  double mag = loadx * loadx + loady * loady + loadz * loadz;
  mag = std::sqrt(mag);
  theta1 = (loadx + loady + loadz) / (mag * ebsdlib::constants::k_Sqrt3D);
  theta1 = std::fabs(theta1);
  theta2 = (loadx + loady - loadz) / (mag * ebsdlib::constants::k_Sqrt3D);
  theta2 = std::fabs(theta2);
  theta3 = (loadx - loady + loadz) / (mag * ebsdlib::constants::k_Sqrt3D);
  theta3 = std::fabs(theta3);
  theta4 = (-loadx + loady + loadz) / (mag * ebsdlib::constants::k_Sqrt3D);
  theta4 = std::fabs(theta4);
  lambda1 = (loadx + loady) / (mag * ebsdlib::constants::k_Sqrt2D);
  lambda1 = std::fabs(lambda1);
  lambda2 = (loadx + loadz) / (mag * ebsdlib::constants::k_Sqrt2D);
  lambda2 = std::fabs(lambda2);
  lambda3 = (loadx - loady) / (mag * ebsdlib::constants::k_Sqrt2D);
  lambda3 = std::fabs(lambda3);
  lambda4 = (loadx - loadz) / (mag * ebsdlib::constants::k_Sqrt2D);
  lambda4 = std::fabs(lambda4);
  lambda5 = (loady + loadz) / (mag * ebsdlib::constants::k_Sqrt2D);
  lambda5 = std::fabs(lambda5);
  lambda6 = (loady - loadz) / (mag * ebsdlib::constants::k_Sqrt2D);
  lambda6 = std::fabs(lambda6);
  schmid1 = theta1 * lambda6;
  schmid2 = theta1 * lambda4;
  schmid3 = theta1 * lambda3;
  schmid4 = theta2 * lambda3;
  schmid5 = theta2 * lambda2;
  schmid6 = theta2 * lambda5;
  schmid7 = theta3 * lambda1;
  schmid8 = theta3 * lambda5;
  schmid9 = theta3 * lambda4;
  schmid10 = theta4 * lambda1;
  schmid11 = theta4 * lambda2;
  schmid12 = theta4 * lambda6;
  schmidfactor = schmid1;
  slipsys = 0;
  angleComps[0] = theta1;
  angleComps[1] = lambda6;

  if(schmid2 > schmidfactor)
  {
    schmidfactor = schmid2;
    slipsys = 1;
    angleComps[0] = theta1;
    angleComps[1] = lambda4;
  }
  if(schmid3 > schmidfactor)
  {
    schmidfactor = schmid3;
    slipsys = 2;
    angleComps[0] = theta1;
    angleComps[1] = lambda3;
  }
  if(schmid4 > schmidfactor)
  {
    schmidfactor = schmid4;
    slipsys = 3;
    angleComps[0] = theta2;
    angleComps[1] = lambda3;
  }
  if(schmid5 > schmidfactor)
  {
    schmidfactor = schmid5;
    slipsys = 4;
    angleComps[0] = theta2;
    angleComps[1] = lambda2;
  }
  if(schmid6 > schmidfactor)
  {
    schmidfactor = schmid6;
    slipsys = 5;
    angleComps[0] = theta2;
    angleComps[1] = lambda5;
  }
  if(schmid7 > schmidfactor)
  {
    schmidfactor = schmid7;
    slipsys = 6;
    angleComps[0] = theta3;
    angleComps[1] = lambda1;
  }
  if(schmid8 > schmidfactor)
  {
    schmidfactor = schmid8;
    slipsys = 7;
    angleComps[0] = theta3;
    angleComps[1] = lambda5;
  }
  if(schmid9 > schmidfactor)
  {
    schmidfactor = schmid9;
    slipsys = 8;
    angleComps[0] = theta3;
    angleComps[1] = lambda4;
  }
  if(schmid10 > schmidfactor)
  {
    schmidfactor = schmid10;
    slipsys = 9;
    angleComps[0] = theta4;
    angleComps[1] = lambda1;
  }
  if(schmid11 > schmidfactor)
  {
    schmidfactor = schmid11;
    slipsys = 10;
    angleComps[0] = theta4;
    angleComps[1] = lambda2;
  }
  if(schmid12 > schmidfactor)
  {
    schmidfactor = schmid12;
    slipsys = 11;
    angleComps[0] = theta4;
    angleComps[1] = lambda6;
  }
}

void CubicOps::getSchmidFactorAndSS(double load[3], double plane[3], double direction[3], double& schmidfactor, double angleComps[2], int& slipsys) const
{
  schmidfactor = 0;
  slipsys = 0;
  angleComps[0] = 0;
  angleComps[1] = 0;

  // compute mags
  double loadMag = std::sqrt(load[0] * load[0] + load[1] * load[1] + load[2] * load[2]);
  double planeMag = std::sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
  double directionMag = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
  planeMag *= loadMag;
  directionMag *= loadMag;

  // loop over symmetry operators finding highest schmid factor
  for(int i = 0; i < CubicHigh::k_SymOpsCount; i++)
  {
    // compute slip system
    double slipPlane[3] = {0};
    slipPlane[2] = CubicHigh::k_MatSym[i](2, 0) * plane[0] + CubicHigh::k_MatSym[i](2, 1) * plane[1] + CubicHigh::k_MatSym[i](2, 2) * plane[2];

    // dont consider negative z planes (to avoid duplicates)
    if(slipPlane[2] >= 0)
    {
      slipPlane[0] = CubicHigh::k_MatSym[i](0, 0) * plane[0] + CubicHigh::k_MatSym[i](0, 1) * plane[1] + CubicHigh::k_MatSym[i](0, 2) * plane[2];
      slipPlane[1] = CubicHigh::k_MatSym[i](1, 0) * plane[0] + CubicHigh::k_MatSym[i](1, 1) * plane[1] + CubicHigh::k_MatSym[i](1, 2) * plane[2];

      double slipDirection[3] = {0};
      slipDirection[0] = CubicHigh::k_MatSym[i](0, 0) * direction[0] + CubicHigh::k_MatSym[i](0, 1) * direction[1] + CubicHigh::k_MatSym[i](0, 2) * direction[2];
      slipDirection[1] = CubicHigh::k_MatSym[i](1, 0) * direction[0] + CubicHigh::k_MatSym[i](1, 1) * direction[1] + CubicHigh::k_MatSym[i](1, 2) * direction[2];
      slipDirection[2] = CubicHigh::k_MatSym[i](2, 0) * direction[0] + CubicHigh::k_MatSym[i](2, 1) * direction[1] + CubicHigh::k_MatSym[i](2, 2) * direction[2];

      double cosPhi = std::fabs(load[0] * slipPlane[0] + load[1] * slipPlane[1] + load[2] * slipPlane[2]) / planeMag;
      double cosLambda = std::fabs(load[0] * slipDirection[0] + load[1] * slipDirection[1] + load[2] * slipDirection[2]) / directionMag;

      double schmid = cosPhi * cosLambda;
      if(schmid > schmidfactor)
      {
        schmidfactor = schmid;
        slipsys = i;
        angleComps[0] = std::acos(cosPhi);
        angleComps[1] = std::acos(cosLambda);
      }
    }
  }
}

double CubicOps::getmPrime(const QuatD& q1, const QuatD& q2, double LDPtr[3]) const
{
  Matrix3X1D hkl1;
  Matrix3X1D uvw1;
  Matrix3X1D hkl2;
  Matrix3X1D uvw2;
  Matrix3X1D slipDirection;
  Matrix3X1D slipPlane;
  Matrix3X1D LD(LDPtr);

  double schmidFactor1 = 0, schmidFactor2 = 0, maxSchmidFactor = 0;
  double directionComponent1 = 0, planeComponent1 = 0;
  double directionComponent2 = 0, planeComponent2 = 0;
  double planeMisalignment = 0, directionMisalignment = 0;
  int ss1 = 0, ss2 = 0;

  Matrix3X3D g1 = QuaternionDType(q1).toOrientationMatrix().toGMatrix().transpose();
  Matrix3X3D g2 = QuaternionDType(q2).toOrientationMatrix().toGMatrix().transpose();

  for(int i = 0; i < 12; i++)
  {
    slipDirection[0] = CubicHigh::SlipDirections[i][0];
    slipDirection[1] = CubicHigh::SlipDirections[i][1];
    slipDirection[2] = CubicHigh::SlipDirections[i][2];
    slipPlane[0] = CubicHigh::SlipPlanes[i][0];
    slipPlane[1] = CubicHigh::SlipPlanes[i][1];
    slipPlane[2] = CubicHigh::SlipPlanes[i][2];
    hkl1 = g1 * slipPlane;
    uvw1 = g1 * slipDirection;
    hkl1 = hkl1.normalize();
    uvw1 = uvw1.normalize();
    directionComponent1 = std::fabs(LD.cosTheta(uvw1));
    planeComponent1 = std::fabs(LD.cosTheta(hkl1));
    schmidFactor1 = directionComponent1 * planeComponent1;
    if(schmidFactor1 > maxSchmidFactor)
    {
      maxSchmidFactor = schmidFactor1;
      ss1 = i;
    }
  }

  slipDirection[0] = CubicHigh::SlipDirections[ss1][0];
  slipDirection[1] = CubicHigh::SlipDirections[ss1][1];
  slipDirection[2] = CubicHigh::SlipDirections[ss1][2];
  slipPlane[0] = CubicHigh::SlipPlanes[ss1][0];
  slipPlane[1] = CubicHigh::SlipPlanes[ss1][1];
  slipPlane[2] = CubicHigh::SlipPlanes[ss1][2];

  hkl1 = g1 * slipPlane;
  uvw1 = g1 * slipDirection;
  hkl1 = hkl1.normalize();
  uvw1 = uvw1.normalize();

  maxSchmidFactor = 0;
  for(int j = 0; j < 12; j++)
  {
    slipDirection[0] = CubicHigh::SlipDirections[j][0];
    slipDirection[1] = CubicHigh::SlipDirections[j][1];
    slipDirection[2] = CubicHigh::SlipDirections[j][2];
    slipPlane[0] = CubicHigh::SlipPlanes[j][0];
    slipPlane[1] = CubicHigh::SlipPlanes[j][1];
    slipPlane[2] = CubicHigh::SlipPlanes[j][2];

    hkl2 = g2 * slipPlane;
    uvw2 = g2 * slipDirection;
    hkl2 = hkl2.normalize();
    uvw2 = uvw2.normalize();

    directionComponent2 = std::fabs(LD.cosTheta(uvw2));
    planeComponent2 = std::fabs(LD.cosTheta(hkl2));
    schmidFactor2 = directionComponent2 * planeComponent2;
    if(schmidFactor2 > maxSchmidFactor)
    {
      maxSchmidFactor = schmidFactor2;
      ss2 = j;
    }
  }
  slipDirection[0] = CubicHigh::SlipDirections[ss2][0];
  slipDirection[1] = CubicHigh::SlipDirections[ss2][1];
  slipDirection[2] = CubicHigh::SlipDirections[ss2][2];
  slipPlane[0] = CubicHigh::SlipPlanes[ss2][0];
  slipPlane[1] = CubicHigh::SlipPlanes[ss2][1];
  slipPlane[2] = CubicHigh::SlipPlanes[ss2][2];

  hkl2 = g2 * slipPlane;
  uvw2 = g2 * slipDirection;
  hkl2 = hkl2.normalize();
  uvw2 = uvw2.normalize();

  planeMisalignment = std::fabs(hkl1.cosTheta(hkl2));
  directionMisalignment = std::fabs(uvw1.cosTheta(uvw2));
  return planeMisalignment * directionMisalignment;
}

double CubicOps::getF1(const QuatD& q1, const QuatD& q2, double LDPtr[3], bool maxSF) const
{
  Matrix3X1D hkl1;
  Matrix3X1D uvw1;
  Matrix3X1D hkl2;
  Matrix3X1D uvw2;
  Matrix3X1D slipDirection;
  Matrix3X1D slipPlane;
  Matrix3X1D LD(LDPtr);

  double directionMisalignment = 0, totalDirectionMisalignment = 0;
  double schmidFactor1 = 0, maxSchmidFactor = 0;
  double directionComponent1 = 0, planeComponent1 = 0;
  // double directionComponent2 = 0, planeComponent2 = 0;
  double maxF1 = 0.0;
  double F1 = 0.0;

  Matrix3X3D g1 = QuaternionDType(q1).toOrientationMatrix().toGMatrix().transpose();
  Matrix3X3D g2 = QuaternionDType(q2).toOrientationMatrix().toGMatrix().transpose();
  LD = LD.normalize();

  if(maxSF)
  {
    maxSchmidFactor = 0;
  }
  for(int i = 0; i < 12; i++)
  {
    slipDirection[0] = CubicHigh::SlipDirections[i][0];
    slipDirection[1] = CubicHigh::SlipDirections[i][1];
    slipDirection[2] = CubicHigh::SlipDirections[i][2];
    slipPlane[0] = CubicHigh::SlipPlanes[i][0];
    slipPlane[1] = CubicHigh::SlipPlanes[i][1];
    slipPlane[2] = CubicHigh::SlipPlanes[i][2];
    hkl1 = g1 * slipPlane;
    uvw1 = g1 * slipDirection;
    hkl1 = hkl1.normalize();
    uvw1 = uvw1.normalize();
    directionComponent1 = std::fabs(LD.cosTheta(uvw1));
    planeComponent1 = std::fabs(LD.cosTheta(hkl1));
    schmidFactor1 = directionComponent1 * planeComponent1;
    if(schmidFactor1 > maxSchmidFactor || !maxSF)
    {
      totalDirectionMisalignment = 0;
      if(maxSF)
      {
        maxSchmidFactor = schmidFactor1;
      }
      for(int j = 0; j < 12; j++)
      {
        slipDirection[0] = CubicHigh::SlipDirections[j][0];
        slipDirection[1] = CubicHigh::SlipDirections[j][1];
        slipDirection[2] = CubicHigh::SlipDirections[j][2];
        slipPlane[0] = CubicHigh::SlipPlanes[j][0];
        slipPlane[1] = CubicHigh::SlipPlanes[j][1];
        slipPlane[2] = CubicHigh::SlipPlanes[j][2];
        hkl2 = g2 * slipPlane;
        uvw2 = g2 * slipDirection;
        hkl2 = hkl2.normalize();
        uvw2 = uvw2.normalize();

        directionMisalignment = std::fabs(uvw1.cosTheta(uvw2));
        totalDirectionMisalignment = totalDirectionMisalignment + directionMisalignment;
      }
      F1 = schmidFactor1 * directionComponent1 * totalDirectionMisalignment;
      if(!maxSF)
      {
        if(F1 < maxF1)
        {
          F1 = maxF1;
        }
        else
        {
          maxF1 = F1;
        }
      }
    }
  }
  return F1;
}

double CubicOps::getF1spt(const QuatD& q1, const QuatD& q2, double LDPtr[3], bool maxSF) const
{
  Matrix3X1D hkl1;
  Matrix3X1D uvw1;
  Matrix3X1D hkl2;
  Matrix3X1D uvw2;
  Matrix3X1D slipDirection;
  Matrix3X1D slipPlane;
  Matrix3X1D LD(LDPtr);

  double directionMisalignment = 0, totalDirectionMisalignment = 0;
  double planeMisalignment = 0, totalPlaneMisalignment = 0;
  double schmidFactor1 = 0, maxSchmidFactor = 0;
  double directionComponent1 = 0, planeComponent1 = 0;
  double maxF1spt = 0.0;
  double F1spt = 0.0f;

  Matrix3X3D g1 = QuaternionDType(q1).toOrientationMatrix().toGMatrix().transpose();
  Matrix3X3D g2 = QuaternionDType(q2).toOrientationMatrix().toGMatrix().transpose();

  LD = LD.normalize();

  if(maxSF)
  {
    maxSchmidFactor = 0;
  }
  for(int i = 0; i < 12; i++)
  {
    slipDirection[0] = CubicHigh::SlipDirections[i][0];
    slipDirection[1] = CubicHigh::SlipDirections[i][1];
    slipDirection[2] = CubicHigh::SlipDirections[i][2];
    slipPlane[0] = CubicHigh::SlipPlanes[i][0];
    slipPlane[1] = CubicHigh::SlipPlanes[i][1];
    slipPlane[2] = CubicHigh::SlipPlanes[i][2];
    hkl1 = g1 * slipPlane;
    uvw1 = g1 * slipDirection;

    hkl1 = hkl1.normalize();
    uvw1 = uvw1.normalize();

    directionComponent1 = std::fabs(LD.cosTheta(uvw1));
    planeComponent1 = std::fabs(LD.cosTheta(hkl1));
    schmidFactor1 = directionComponent1 * planeComponent1;
    if(schmidFactor1 > maxSchmidFactor || !maxSF)
    {
      totalDirectionMisalignment = 0;
      totalPlaneMisalignment = 0;
      if(maxSF)
      {
        maxSchmidFactor = schmidFactor1;
      }
      for(int j = 0; j < 12; j++)
      {
        slipDirection[0] = CubicHigh::SlipDirections[j][0];
        slipDirection[1] = CubicHigh::SlipDirections[j][1];
        slipDirection[2] = CubicHigh::SlipDirections[j][2];
        slipPlane[0] = CubicHigh::SlipPlanes[j][0];
        slipPlane[1] = CubicHigh::SlipPlanes[j][1];
        slipPlane[2] = CubicHigh::SlipPlanes[j][2];

        hkl2 = g2 * slipPlane;
        uvw2 = g2 * slipDirection;
        hkl2 = hkl2.normalize();
        uvw2 = uvw2.normalize();

        directionMisalignment = std::fabs(uvw1.cosTheta(uvw2));
        planeMisalignment = std::fabs(hkl1.cosTheta(hkl2));
        totalDirectionMisalignment = totalDirectionMisalignment + directionMisalignment;
        totalPlaneMisalignment = totalPlaneMisalignment + planeMisalignment;
      }
      F1spt = schmidFactor1 * directionComponent1 * totalDirectionMisalignment * totalPlaneMisalignment;
      if(!maxSF)
      {
        if(F1spt < maxF1spt)
        {
          F1spt = maxF1spt;
        }
        else
        {
          maxF1spt = F1spt;
        }
      }
    }
  }
  return F1spt;
}

double CubicOps::getF7(const QuatD& q1, const QuatD& q2, double LDPtr[3], bool maxSF) const
{
  Matrix3X1D hkl1;
  Matrix3X1D uvw1;
  Matrix3X1D hkl2;
  Matrix3X1D uvw2;
  Matrix3X1D slipDirection;
  Matrix3X1D slipPlane;
  Matrix3X1D LD(LDPtr);

  double directionMisalignment = 0, totalDirectionMisalignment = 0;
  double schmidFactor1 = 0.0, maxSchmidFactor = 0.0;
  double directionComponent1 = 0.0;
  double planeComponent1 = 0.0;

  // double directionComponent2 = 0, planeComponent2 = 0;
  double maxF7 = 0.0;
  double F7 = 0.0f;

  Matrix3X3D g1 = QuaternionDType(q1).toOrientationMatrix().toGMatrix().transpose();
  Matrix3X3D g2 = QuaternionDType(q2).toOrientationMatrix().toGMatrix().transpose();

  LD = LD.normalize();

  for(int i = 0; i < 12; i++)
  {
    slipDirection[0] = CubicHigh::SlipDirections[i][0];
    slipDirection[1] = CubicHigh::SlipDirections[i][1];
    slipDirection[2] = CubicHigh::SlipDirections[i][2];
    slipPlane[0] = CubicHigh::SlipPlanes[i][0];
    slipPlane[1] = CubicHigh::SlipPlanes[i][1];
    slipPlane[2] = CubicHigh::SlipPlanes[i][2];
    hkl1 = g1 * slipPlane;
    uvw1 = g1 * slipDirection;
    hkl1 = hkl1.normalize();
    uvw1 = uvw1.normalize();
    directionComponent1 = std::fabs(LD.cosTheta(uvw1));
    planeComponent1 = std::fabs(LD.cosTheta(hkl1));
    schmidFactor1 = directionComponent1 * planeComponent1;
    if(schmidFactor1 > maxSchmidFactor || !maxSF)
    {
      totalDirectionMisalignment = 0;
      if(maxSF)
      {
        maxSchmidFactor = schmidFactor1;
      }
      for(int j = 0; j < 12; j++)
      {
        slipDirection[0] = CubicHigh::SlipDirections[j][0];
        slipDirection[1] = CubicHigh::SlipDirections[j][1];
        slipDirection[2] = CubicHigh::SlipDirections[j][2];
        slipPlane[0] = CubicHigh::SlipPlanes[j][0];
        slipPlane[1] = CubicHigh::SlipPlanes[j][1];
        slipPlane[2] = CubicHigh::SlipPlanes[j][2];
        hkl2 = g2 * slipPlane;
        uvw2 = g2 * slipDirection;
        hkl2 = hkl2.normalize();
        uvw2 = uvw2.normalize();

        directionMisalignment = std::fabs(uvw1.cosTheta(uvw2));
        totalDirectionMisalignment = totalDirectionMisalignment + directionMisalignment;
      }
      F7 = directionComponent1 * directionComponent1 * totalDirectionMisalignment;
      if(!maxSF)
      {
        if(F7 < maxF7)
        {
          F7 = maxF7;
        }
        else
        {
          maxF7 = F7;
        }
      }
    }
  }
  return F7;
}

// -----------------------------------------------------------------------------
namespace CubicHigh
{
class GenerateSphereCoordsImpl
{
  ebsdlib::FloatArrayType* m_Eulers;
  ebsdlib::FloatArrayType* m_xyz001;
  ebsdlib::FloatArrayType* m_xyz011;
  ebsdlib::FloatArrayType* m_xyz111;

public:
  GenerateSphereCoordsImpl(ebsdlib::FloatArrayType* eulers, ebsdlib::FloatArrayType* xyz001, ebsdlib::FloatArrayType* xyz011, ebsdlib::FloatArrayType* xyz111)
  : m_Eulers(eulers)
  , m_xyz001(xyz001)
  , m_xyz011(xyz011)
  , m_xyz111(xyz111)
  {
  }
  virtual ~GenerateSphereCoordsImpl() = default;

  void generate(size_t start, size_t end) const
  {
    Matrix3X1D direction(0.0, 0.0, 0.0);

    for(size_t i = start; i < end; ++i)
    {

      EulerDType euler(m_Eulers->getValue(i * 3), m_Eulers->getValue(i * 3 + 1), m_Eulers->getValue(i * 3 + 2));
      ebsdlib::Matrix3X3D gTranspose = euler.toOrientationMatrix().toGMatrix().transpose();

      // -----------------------------------------------------------------------------
      // 001 Family
      direction[0] = 1.0;
      direction[1] = 0.0;
      direction[2] = 0.0;
      (gTranspose * direction).copyInto<float>(m_xyz001->getPointer(i * 18));
      std::transform(m_xyz001->getPointer(i * 18), m_xyz001->getPointer(i * 18 + 3),
                     m_xyz001->getPointer(i * 18 + 3),           // write to the next triplet in memory
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0
      direction[0] = 0.0;
      direction[1] = 1.0;
      direction[2] = 0.0;
      (gTranspose * direction).copyInto<float>(m_xyz001->getPointer(i * 18 + 6));
      std::transform(m_xyz001->getPointer(i * 18 + 6), m_xyz001->getPointer(i * 18 + 9),
                     m_xyz001->getPointer(i * 18 + 9),           // write to the next triplet in memory
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0
      direction[0] = 0.0;
      direction[1] = 0.0;
      direction[2] = 1.0;
      (gTranspose * direction).copyInto<float>(m_xyz001->getPointer(i * 18 + 12));
      std::transform(m_xyz001->getPointer(i * 18 + 12), m_xyz001->getPointer(i * 18 + 15),
                     m_xyz001->getPointer(i * 18 + 15),          // write to the next triplet in memory
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0

      // -----------------------------------------------------------------------------
      // 011 Family
      direction[0] = ebsdlib::constants::k_1OverRoot2D;
      direction[1] = ebsdlib::constants::k_1OverRoot2D;
      direction[2] = 0.0;
      (gTranspose * direction).copyInto<float>(m_xyz011->getPointer(i * 36));
      std::transform(m_xyz011->getPointer(i * 36), m_xyz011->getPointer(i * 36 + 3),
                     m_xyz011->getPointer(i * 36 + 3),           // write to the next triplet in memory
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0
      direction[0] = ebsdlib::constants::k_1OverRoot2D;
      direction[1] = 0.0;
      direction[2] = ebsdlib::constants::k_1OverRoot2D;
      (gTranspose * direction).copyInto<float>(m_xyz011->getPointer(i * 36 + 6));
      std::transform(m_xyz011->getPointer(i * 36 + 6), m_xyz011->getPointer(i * 36 + 9),
                     m_xyz011->getPointer(i * 36 + 9),           // write to the next triplet in memory
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0
      direction[0] = 0.0;
      direction[1] = ebsdlib::constants::k_1OverRoot2D;
      direction[2] = ebsdlib::constants::k_1OverRoot2D;
      (gTranspose * direction).copyInto<float>(m_xyz011->getPointer(i * 36 + 12));
      std::transform(m_xyz011->getPointer(i * 36 + 12), m_xyz011->getPointer(i * 36 + 15),
                     m_xyz011->getPointer(i * 36 + 15),          // write to the next triplet in memory
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0
      direction[0] = -ebsdlib::constants::k_1OverRoot2D;
      direction[1] = ebsdlib::constants::k_1OverRoot2D;
      direction[2] = 0.0;
      (gTranspose * direction).copyInto<float>(m_xyz011->getPointer(i * 36 + 18));
      std::transform(m_xyz011->getPointer(i * 36 + 18), m_xyz011->getPointer(i * 36 + 21),
                     m_xyz011->getPointer(i * 36 + 21),          // write to the next triplet in memory
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0
      direction[0] = -ebsdlib::constants::k_1OverRoot2D;
      direction[1] = 0.0;
      direction[2] = ebsdlib::constants::k_1OverRoot2D;
      (gTranspose * direction).copyInto<float>(m_xyz011->getPointer(i * 36 + 24));
      std::transform(m_xyz011->getPointer(i * 36 + 24), m_xyz011->getPointer(i * 36 + 27),
                     m_xyz011->getPointer(i * 36 + 27),          // write to the next triplet in memory
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0
      direction[0] = 0.0;
      direction[1] = -ebsdlib::constants::k_1OverRoot2D;
      direction[2] = ebsdlib::constants::k_1OverRoot2D;
      (gTranspose * direction).copyInto<float>(m_xyz011->getPointer(i * 36 + 30));
      std::transform(m_xyz011->getPointer(i * 36 + 30), m_xyz011->getPointer(i * 36 + 33),
                     m_xyz011->getPointer(i * 36 + 33),          // write to the next triplet in memory
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0

      // -----------------------------------------------------------------------------
      // 111 Family
      direction[0] = ebsdlib::constants::k_1OverRoot3D;
      direction[1] = ebsdlib::constants::k_1OverRoot3D;
      direction[2] = ebsdlib::constants::k_1OverRoot3D;
      (gTranspose * direction).copyInto<float>(m_xyz111->getPointer(i * 24));
      std::transform(m_xyz111->getPointer(i * 24), m_xyz111->getPointer(i * 24 + 3),
                     m_xyz111->getPointer(i * 24 + 3),           // write to the next triplet in memory
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0
      direction[0] = -ebsdlib::constants::k_1OverRoot3D;
      direction[1] = ebsdlib::constants::k_1OverRoot3D;
      direction[2] = ebsdlib::constants::k_1OverRoot3D;
      (gTranspose * direction).copyInto<float>(m_xyz111->getPointer(i * 24 + 6));
      std::transform(m_xyz111->getPointer(i * 24 + 6), m_xyz111->getPointer(i * 24 + 9),
                     m_xyz111->getPointer(i * 24 + 9),           // write to the next triplet in memory
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0
      direction[0] = ebsdlib::constants::k_1OverRoot3D;
      direction[1] = -ebsdlib::constants::k_1OverRoot3D;
      direction[2] = ebsdlib::constants::k_1OverRoot3D;
      (gTranspose * direction).copyInto<float>(m_xyz111->getPointer(i * 24 + 12));
      std::transform(m_xyz111->getPointer(i * 24 + 12), m_xyz111->getPointer(i * 24 + 15),
                     m_xyz111->getPointer(i * 24 + 15),          // write to the next triplet in memory
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0
      direction[0] = ebsdlib::constants::k_1OverRoot3D;
      direction[1] = ebsdlib::constants::k_1OverRoot3D;
      direction[2] = -ebsdlib::constants::k_1OverRoot3D;
      (gTranspose * direction).copyInto<float>(m_xyz111->getPointer(i * 24 + 18));
      std::transform(m_xyz111->getPointer(i * 24 + 18), m_xyz111->getPointer(i * 24 + 21),
                     m_xyz111->getPointer(i * 24 + 21),          // write to the next triplet in memory
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0
    }
  }

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  void operator()(const tbb::blocked_range<size_t>& r) const
  {
    generate(r.begin(), r.end());
  }
#endif
};
} // namespace CubicHigh

// -----------------------------------------------------------------------------
void CubicOps::generateSphereCoordsFromEulers(ebsdlib::FloatArrayType* eulers, ebsdlib::FloatArrayType* xyz001, ebsdlib::FloatArrayType* xyz011, ebsdlib::FloatArrayType* xyz111,
                                              ebsdlib::HexConvention conv) const
{
  size_t nOrientations = eulers->getNumberOfTuples();

  // Sanity Check the size of the arrays
  if(xyz001->getNumberOfTuples() < nOrientations * CubicHigh::k_SymSize0)
  {
    xyz001->resizeTuples(nOrientations * CubicHigh::k_SymSize0 * 3);
  }
  if(xyz011->getNumberOfTuples() < nOrientations * CubicHigh::k_SymSize1)
  {
    xyz011->resizeTuples(nOrientations * CubicHigh::k_SymSize1 * 3);
  }
  if(xyz111->getNumberOfTuples() < nOrientations * CubicHigh::k_SymSize2)
  {
    xyz111->resizeTuples(nOrientations * CubicHigh::k_SymSize2 * 3);
  }

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  bool doParallel = true;
  if(doParallel)
  {
    tbb::parallel_for(tbb::blocked_range<size_t>(0, nOrientations), CubicHigh::GenerateSphereCoordsImpl(eulers, xyz001, xyz011, xyz111), tbb::auto_partitioner());
  }
  else
#endif
  {
    CubicHigh::GenerateSphereCoordsImpl serial(eulers, xyz001, xyz011, xyz111);
    serial.generate(0, nOrientations);
  }
}

/**
 * @brief Sorts the 3 values from low to high
 * @param a
 * @param b
 * @param c
 * @param sorted The array to store the sorted values.
 */
#if 0
template <typename T>
void TripleSort(T a, T b, T c, T* sorted)
{
  if(a > b && a > c)
  {
    sorted[2] = a;
    if(b > c)
    {
      sorted[1] = b;
      sorted[0] = c;
    }
    else
    {
      sorted[1] = c;
      sorted[0] = b;
    }
  }
  else if(b > a && b > c)
  {
    sorted[2] = b;
    if(a > c)
    {
      sorted[1] = a;
      sorted[0] = c;
    }
    else
    {
      sorted[1] = c;
      sorted[0] = a;
    }
  }
  else if(a > b)
  {
    sorted[1] = a;
    sorted[0] = b;
    sorted[2] = c;
  }
  else if(a >= c && b >= c)
  {
    sorted[0] = c;
    sorted[1] = a;
    sorted[2] = b;
  }
  else
  {
    sorted[0] = a;
    sorted[1] = b;
    sorted[2] = c;
  }
}
#endif
/**
 * @brief Sorts the 3 values from low to high
 * @param a Input
 * @param b Input
 * @param c Input
 * @param x Output
 * @param y Output
 * @param z Output
 */
template <typename T>
ebsdlib::Matrix3X1<T> TripletSort(ebsdlib::Matrix3X1<T>& vec)
{
  T x, y, z;
  if(vec[0] > vec[1] && vec[0] > vec[2])
  {
    z = vec[0];
    if(vec[1] > vec[2])
    {
      y = vec[1];
      x = vec[2];
    }
    else
    {
      y = vec[2];
      x = vec[1];
    }
  }
  else if(vec[1] > vec[0] && vec[1] > vec[2])
  {
    z = vec[1];
    if(vec[0] > vec[2])
    {
      y = vec[0];
      x = vec[2];
    }
    else
    {
      y = vec[2];
      x = vec[0];
    }
  }
  else if(vec[0] > vec[1])
  {
    y = vec[0];
    x = vec[1];
    z = vec[2];
  }
  else if(vec[0] >= vec[2] && vec[1] >= vec[2])
  {
    x = vec[2];
    y = vec[0];
    z = vec[1];
  }
  else
  {
    x = vec[0];
    y = vec[1];
    z = vec[2];
  }
  return {x, y, z};
}

bool inUnitTriangleD(double eta, double chi)
{
  double etaDeg = eta * ebsdlib::constants::k_180OverPiD;
  double chiMax;
  if(etaDeg > 45.0)
  {
    chiMax = sqrt(1.0 / (2.0 + std::tan(0.5 * ebsdlib::constants::k_PiD - eta) * std::tan(0.5 * ebsdlib::constants::k_PiD - eta)));
  }
  else
  {
    chiMax = sqrt(1.0 / (2.0 + std::tan(eta) * std::tan(eta)));
  }
  ebsdlib::math::bound(chiMax, -1.0, 1.0);
  chiMax = acos(chiMax);
  return !(eta < 0.0 || eta > (45.0 * ebsdlib::constants::k_PiOver180D) || chi < 0.0 || chi > chiMax);
}

// -----------------------------------------------------------------------------
std::array<double, 3> CubicOps::getIpfColorAngleLimits(double eta) const
{
  double etaDeg = eta * ebsdlib::constants::k_180OverPiD;
  double chiMax;
  if(etaDeg > CubicHigh::k_EtaMax)
  {
    chiMax = std::sqrt(1.0 / (2.0 + std::tan(0.5 * ebsdlib::constants::k_PiD - eta) * std::tan(0.5 * ebsdlib::constants::k_PiD - eta)));
  }
  else
  {
    chiMax = std::sqrt(1.0 / (2.0 + std::tan(eta) * std::tan(eta)));
  }
  ebsdlib::math::bound(chiMax, -1.0, 1.0);
  chiMax = std::acos(chiMax);
  return {CubicHigh::k_EtaMin * ebsdlib::constants::k_DegToRadD, CubicHigh::k_EtaMax * ebsdlib::constants::k_DegToRadD, chiMax};
}

// -----------------------------------------------------------------------------
bool CubicOps::inUnitTriangle(double eta, double chi) const
{
  double etaDeg = eta * ebsdlib::constants::k_180OverPiD;
  double chiMax;
  if(etaDeg > CubicHigh::k_EtaMax)
  {
    chiMax = std::sqrt(1.0 / (2.0 + std::tan(0.5 * ebsdlib::constants::k_PiD - eta) * std::tan(0.5 * ebsdlib::constants::k_PiD - eta)));
  }
  else
  {
    chiMax = std::sqrt(1.0 / (2.0 + std::tan(eta) * std::tan(eta)));
  }
  ebsdlib::math::bound(chiMax, -1.0, 1.0);
  chiMax = acos(chiMax);
  return !(eta < CubicHigh::k_EtaMin || eta > (CubicHigh::k_EtaMax * ebsdlib::constants::k_PiOver180D) || chi < 0.0 || chi > chiMax);
}

// -----------------------------------------------------------------------------
ebsdlib::Rgb CubicOps::generateIPFColor(double* eulers, double* refDir, bool degToRad, ebsdlib::ColorKeyKind kind) const
{
  return computeIPFColor(eulers, refDir, degToRad, keyForKind(kind).get());
}

// -----------------------------------------------------------------------------
ebsdlib::Rgb CubicOps::generateIPFColor(double phi1, double phi, double phi2, double refDir0, double refDir1, double refDir2, bool degToRad, ebsdlib::ColorKeyKind kind) const
{
  double eulers[3] = {phi1, phi, phi2};
  double refDir[3] = {refDir0, refDir1, refDir2};
  return computeIPFColor(eulers, refDir, degToRad, keyForKind(kind).get());
}

// -----------------------------------------------------------------------------
ebsdlib::Rgb CubicOps::generateRodriguesColor(double r1, double r2, double r3) const
{
  double range1 = 2.0f * CubicHigh::k_OdfDimInitValue[0];
  double range2 = 2.0f * CubicHigh::k_OdfDimInitValue[1];
  double range3 = 2.0f * CubicHigh::k_OdfDimInitValue[2];
  double max1 = range1 / 2.0f;
  double max2 = range2 / 2.0f;
  double max3 = range3 / 2.0f;
  double red = (r1 + max1) / range1;
  double green = (r2 + max2) / range2;
  double blue = (r3 + max3) / range3;

  return ebsdlib::RgbColor::dRgb(static_cast<int32_t>(red * 255), static_cast<int32_t>(green * 255), static_cast<int32_t>(blue * 255), 255);
}

// -----------------------------------------------------------------------------
std::array<std::string, 3> CubicOps::getDefaultPoleFigureNames(ebsdlib::HexConvention conv) const
{
  return {"{001}", "{011}", "{111}"};
}

// -----------------------------------------------------------------------------
std::vector<ebsdlib::UInt8ArrayType::Pointer> CubicOps::generatePoleFigure(PoleFigureConfiguration_t& config) const
{
  std::array<std::string, 3> labels = getDefaultPoleFigureNames(ebsdlib::HexConvention::NotApplicable);
  std::string label0 = labels[0];
  std::string label1 = labels[1];
  std::string label2 = labels[2];

  if(!config.labels.empty())
  {
    label0 = config.labels.at(0);
  }
  if(config.labels.size() > 1)
  {
    label1 = config.labels.at(1);
  }
  if(config.labels.size() > 2)
  {
    label2 = config.labels.at(2);
  }

  size_t numOrientations = config.eulers->getNumberOfTuples();

  // Create an Array to hold the XYZ Coordinates which are the coords on the sphere.
  // this is size for CUBIC ONLY, <001> Family
  std::vector<size_t> dims(1, 3);
  ebsdlib::FloatArrayType::Pointer xyz001 = ebsdlib::FloatArrayType::CreateArray(numOrientations * CubicHigh::k_SymSize0, dims, label0 + std::string("xyzCoords"), true);
  // this is size for CUBIC ONLY, <011> Family
  ebsdlib::FloatArrayType::Pointer xyz011 = ebsdlib::FloatArrayType::CreateArray(numOrientations * CubicHigh::k_SymSize1, dims, label1 + std::string("xyzCoords"), true);
  // this is size for CUBIC ONLY, <111> Family
  ebsdlib::FloatArrayType::Pointer xyz111 = ebsdlib::FloatArrayType::CreateArray(numOrientations * CubicHigh::k_SymSize2, dims, label2 + std::string("xyzCoords"), true);

  config.sphereRadius = 1.0f;

  // Generate the coords on the sphere **** Parallelized
  generateSphereCoordsFromEulers(config.eulers, xyz001.get(), xyz011.get(), xyz111.get());

  // These arrays hold the "intensity" images which eventually get converted to an actual Color RGB image
  // Generate the modified Lambert projection images (Squares, 2 of them, 1 for Northern Hemisphere, 1 for Southern Hemisphere
  ebsdlib::DoubleArrayType::Pointer intensity001 = ebsdlib::DoubleArrayType::CreateArray(config.imageDim * config.imageDim, label0 + "_Intensity_Image", true);
  ebsdlib::DoubleArrayType::Pointer intensity011 = ebsdlib::DoubleArrayType::CreateArray(config.imageDim * config.imageDim, label1 + "_Intensity_Image", true);
  ebsdlib::DoubleArrayType::Pointer intensity111 = ebsdlib::DoubleArrayType::CreateArray(config.imageDim * config.imageDim, label2 + "_Intensity_Image", true);

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  bool doParallel = true;

  if(doParallel)
  {
    std::shared_ptr<tbb::task_group> g(new tbb::task_group);
    g->run(ComputeStereographicProjection(xyz001.get(), &config, intensity001.get()));
    g->run(ComputeStereographicProjection(xyz011.get(), &config, intensity011.get()));
    g->run(ComputeStereographicProjection(xyz111.get(), &config, intensity111.get()));
    g->wait(); // Wait for all the threads to complete before moving on.
  }
  else
#endif
  {
    ComputeStereographicProjection m001(xyz001.get(), &config, intensity001.get());
    m001();
    ComputeStereographicProjection m011(xyz011.get(), &config, intensity011.get());
    m011();
    ComputeStereographicProjection m111(xyz111.get(), &config, intensity111.get());
    m111();
  }

  // Find the Max and Min values based on ALL 3 arrays, so we can color scale them all the same
  double max = std::numeric_limits<double>::min();
  double min = std::numeric_limits<double>::max();

  double* dPtr = intensity001->getPointer(0);
  size_t count = intensity001->getNumberOfTuples();
  for(size_t i = 0; i < count; ++i)
  {
    if(dPtr[i] > max)
    {
      max = dPtr[i];
    }
    if(dPtr[i] < min)
    {
      min = dPtr[i];
    }
  }

  dPtr = intensity011->getPointer(0);
  count = intensity011->getNumberOfTuples();
  for(size_t i = 0; i < count; ++i)
  {
    if(dPtr[i] > max)
    {
      max = dPtr[i];
    }
    if(dPtr[i] < min)
    {
      min = dPtr[i];
    }
  }

  dPtr = intensity111->getPointer(0);
  count = intensity111->getNumberOfTuples();
  for(size_t i = 0; i < count; ++i)
  {
    if(dPtr[i] > max)
    {
      max = dPtr[i];
    }
    if(dPtr[i] < min)
    {
      min = dPtr[i];
    }
  }

  config.minScale = min;
  config.maxScale = max;

  dims[0] = 4;
  ebsdlib::UInt8ArrayType::Pointer image001 = ebsdlib::UInt8ArrayType::CreateArray(static_cast<size_t>(config.imageDim * config.imageDim), dims, label0, true);
  ebsdlib::UInt8ArrayType::Pointer image011 = ebsdlib::UInt8ArrayType::CreateArray(static_cast<size_t>(config.imageDim * config.imageDim), dims, label1, true);
  ebsdlib::UInt8ArrayType::Pointer image111 = ebsdlib::UInt8ArrayType::CreateArray(static_cast<size_t>(config.imageDim * config.imageDim), dims, label2, true);

  std::vector<ebsdlib::UInt8ArrayType::Pointer> poleFigures(3);
  if(config.order.size() == 3)
  {
    poleFigures[static_cast<int>(config.order[0])] = image001;
    poleFigures[static_cast<int>(config.order[1])] = image011;
    poleFigures[static_cast<int>(config.order[2])] = image111;
  }
  else
  {
    poleFigures[0] = image001;
    poleFigures[1] = image011;
    poleFigures[2] = image111;
  }

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS

  if(doParallel)
  {
    std::shared_ptr<tbb::task_group> g(new tbb::task_group);
    g->run(GeneratePoleFigureRgbaImageImpl(intensity001.get(), &config, image001.get()));
    g->run(GeneratePoleFigureRgbaImageImpl(intensity011.get(), &config, image011.get()));
    g->run(GeneratePoleFigureRgbaImageImpl(intensity111.get(), &config, image111.get()));
    g->wait(); // Wait for all the threads to complete before moving on.
  }
  else
#endif
  {
    GeneratePoleFigureRgbaImageImpl m001(intensity001.get(), &config, image001.get());
    m001();
    GeneratePoleFigureRgbaImageImpl m011(intensity011.get(), &config, image011.get());
    m011();
    GeneratePoleFigureRgbaImageImpl m111(intensity111.get(), &config, image111.get());
    m111();
  }

#if 0
  size_t dim[3] = {config.imageDim, config.imageDim, 1};
  FloatVec3Type res = {1.0, 1.0, 1.0};
  VtkRectilinearGridWriter::WriteDataArrayToFile("/tmp/" + intensity001->getName() + ".vtk",
                                                 intensity001.get(), dim, res, "double", true );
  VtkRectilinearGridWriter::WriteDataArrayToFile("/tmp/" + intensity011->getName() + ".vtk",
                                                 intensity011.get(), dim, res, "double", true );
  VtkRectilinearGridWriter::WriteDataArrayToFile("/tmp/" + intensity111->getName() + ".vtk",
                                                 intensity111.get(), dim, res, "double", true );
#endif
  return poleFigures;
}

namespace
{
ebsdlib::UInt8ArrayType::Pointer CreateIPFLegend(const CubicOps* ops, int imageDim, bool generateEntirePlane, const ebsdlib::IColorKey* key)
{
  std::vector<size_t> dims(1, 4);
  std::string arrayName = EbsdStringUtils::replace(ops->getSymmetryName(), "/", "_");
  ebsdlib::UInt8ArrayType::Pointer image = ebsdlib::UInt8ArrayType::CreateArray(imageDim * imageDim, dims, arrayName + " Triangle Legend", true);
  uint32_t* pixelPtr = reinterpret_cast<uint32_t*>(image->getPointer(0));

  double indexConst1 = 0.414f / static_cast<double>(imageDim);
  double indexConst2 = 0.207f / static_cast<double>(imageDim);
  double xInc = 1.0f / static_cast<double>(imageDim);
  double yInc = 1.0f / static_cast<double>(imageDim);
  double rad = 1.0f;
  double red1 = 0.0f;

  double x = 0.0f;
  double y = 0.0f;

  double phi = 0.0f;
  double x1alt = 0.0f;
  double theta = 0.0f;
  double k_RootOfHalf = sqrtf(0.5f);

  Matrix3X1D orientation(0.0, 0.0, 0.0);
  ebsdlib::Rgb color;
  size_t idx = 0;
  size_t yScanLineIndex = 0; // We use this to control where the data is drawn. Otherwise, the image will come out flipped vertically
  // Loop over every pixel in the image and project up to the sphere to get the angle and then figure out the RGB from
  // there.
  for(int32_t yIndex = 0; yIndex < imageDim; ++yIndex)
  {
    for(int32_t xIndex = 0; xIndex < imageDim; ++xIndex)
    {
      idx = (imageDim * yScanLineIndex) + xIndex;

      if(generateEntirePlane) // Color is full unit circle
      {
        x = -1.0f + 2.0f * xIndex * xInc;
        y = -1.0f + 2.0f * yIndex * yInc;
      }
      else
      {
        x = xIndex * indexConst1 + indexConst2;
        y = yIndex * indexConst1 + indexConst2;
      }
      double sumSquares = (x * x) + (y * y);

      auto sphericalCoords = stereographic::utils::StereoToSpherical(x, y).normalize();

      red1 = sphericalCoords[0] * (-k_RootOfHalf) + sphericalCoords[2] * k_RootOfHalf;
      phi = acos(red1);
      x1alt = sphericalCoords[0] / k_RootOfHalf;
      x1alt = x1alt / sqrt((x1alt * x1alt) + (sphericalCoords[1] * sphericalCoords[1]));
      theta = acos(x1alt);

      if(sumSquares > 1.0f)
      {
        color = 0xFFFFFFFF;
      }
      else if(!generateEntirePlane && (y < 0.0F || x < 0.0F))
      {
        color = 0xFFFFFFFF;
      }
      else if(!generateEntirePlane && (phi <= (45.0f * ebsdlib::constants::k_PiOver180D) || phi >= (90.0f * ebsdlib::constants::k_PiOver180D) || theta >= (35.26f * ebsdlib::constants::k_PiOver180D)))
      {
        color = 0xFFFFFFFF;
      }
      else
      {
        // 3) move that direction to a single standard triangle - using the 001-011-111 triangle
        sphericalCoords = sphericalCoords.abs();
        // Sort the cd array from smallest to largest
        sphericalCoords = TripletSort(sphericalCoords);

        color = ops->computeIPFColor(orientation.data(), sphericalCoords.data(), false, key);
      }
      pixelPtr[idx] = color;
    }
    yScanLineIndex++;
  }
  return image;
}

// -----------------------------------------------------------------------------
} // namespace

// -----------------------------------------------------------------------------
bool CubicOps::mapPixelToSphereSST(int xPixel, int yPixel, int imageDim, std::array<float, 3>& sphereDir) const
{
  double indexConst1 = 0.414 / static_cast<double>(imageDim);
  double indexConst2 = 0.207 / static_cast<double>(imageDim);

  double x = xPixel * indexConst1 + indexConst2;
  double y = yPixel * indexConst1 + indexConst2;

  double sumSquares = (x * x) + (y * y);
  if(sumSquares > 1.0)
  {
    return false;
  }
  if(y < 0.0 || x < 0.0)
  {
    return false;
  }

  auto sc = stereographic::utils::StereoToSpherical(x, y).normalize();

  double k_RootOfHalf = std::sqrt(0.5);
  double red1 = sc[0] * (-k_RootOfHalf) + sc[2] * k_RootOfHalf;
  double phi = std::acos(red1);
  double x1alt = sc[0] / k_RootOfHalf;
  x1alt = x1alt / std::sqrt((x1alt * x1alt) + (sc[1] * sc[1]));
  double theta = std::acos(x1alt);

  if(phi <= (45.0 * ebsdlib::constants::k_PiOver180D) || phi >= (90.0 * ebsdlib::constants::k_PiOver180D) || theta >= (35.26 * ebsdlib::constants::k_PiOver180D))
  {
    return false;
  }

  sphereDir[0] = static_cast<float>(sc[0]);
  sphereDir[1] = static_cast<float>(sc[1]);
  sphereDir[2] = static_cast<float>(sc[2]);
  return true;
}

// -----------------------------------------------------------------------------
std::array<float, 2> CubicOps::adjustFigureOrigin(std::array<float, 2> figureOrigin, int legendWidth, int legendHeight, const std::vector<float>& margins, float fontPtSize,
                                                  bool generateEntirePlane) const
{
  if(!generateEntirePlane)
  {
    figureOrigin[1] = fontPtSize * 2.0F;
  }
  return figureOrigin;
}

// -----------------------------------------------------------------------------
void CubicOps::drawIPFAnnotations(canvas_ity::canvas& context, int canvasDim, float fontPtSize, const std::vector<float>& margins, std::array<float, 2> figureOrigin, std::array<float, 2> figureCenter,
                                  bool drawFullCircle, ebsdlib::HexConvention conv) const
{
  if(!drawFullCircle)
  {
    int legendHeight = canvasDim - static_cast<int>(margins[0]) - static_cast<int>(margins[2]);
    int legendWidth = canvasDim - static_cast<int>(margins[1]) - static_cast<int>(margins[3]);
    if(legendHeight > legendWidth)
    {
      legendHeight = legendWidth;
    }
    figureCenter = {figureOrigin[0], figureOrigin[1] + static_cast<float>(legendHeight)};
  }

  int legendHeight = canvasDim - margins[0] - margins[2];
  int legendWidth = canvasDim - margins[1] - margins[3];

  if(legendHeight > legendWidth)
  {
    legendHeight = legendWidth;
  }
  else
  {
    legendWidth = legendHeight;
  }
  //  int pageHeight = canvasDim;
  //  int pageWidth = canvasDim;
  int halfWidth = legendWidth / 2;
  int halfHeight = legendHeight / 2;

  std::vector<float> angles = {0.0f, 45.0F, 90.0F, 135.0F, 180.0F, 225.0F, 270.0F, 315.0F};
  std::vector<std::string> labels2 = {
      "[100]", "[110]", "[010]", "[-110]", "[-100]", "[-1-10]", "[0-10]", "[1-10]",
  };

  std::vector<float> xAdj = {0.1F, 0.0F, -0.5F, -1.0F, -1.1F, -1.0F, -0.5F, 0.0F};
  std::vector<float> yAdj = {
      +0.25F, 0.0F, -0.1F, 0.0F, 0.25F, 0.75F, 1.1F, 1.0F,
  };
  std::vector<bool> drawAngle = {false, false, false, false, false, false, false, false};

  for(size_t idx = 0; idx < angles.size(); idx++)
  {
    float radius = 1.0f;
    float angle = angles[idx];
    float rads = angle * ebsdlib::constants::k_DegToRadF;
    float x = radius * (cos(rads));
    float y = radius * (sin(rads));

    // Transform from Unit Circle to our flipped Screen Pixel Coordinates
    // First Scale up to our image dimensions
    x = x * static_cast<float>(halfWidth);
    y = y * static_cast<float>(halfHeight);

    // Next, translate to the center of the image
    x = x + static_cast<float>(halfWidth);
    y = y + static_cast<float>(halfHeight);

    // Now mirror across the x-axis (vertically) because this is the transformation from
    // cartesian coords to screen coords
    y = static_cast<float>(legendHeight) - y;

    x = x + figureOrigin[0];
    y = y + figureOrigin[1];

    // Draw the line from the center point to the point on the circle
    if(drawAngle[idx] || drawFullCircle)
    {
      float penWidth = 1.0f;
      context.set_color(canvas_ity::stroke_style, 0.25f, 0.25f, 0.25f, 1.0f);
      context.set_line_width(penWidth);
      ebsdlib::DrawLine(context, figureCenter[0], figureCenter[1], x, y);
    }
    std::string label = labels2[idx];
    std::string fontWidthString = EbsdStringUtils::replace(label, "-", "");
    float fontWidth = context.measure_text(fontWidthString.c_str());

    x = x + (xAdj[idx] * fontWidth);
    y = y + (yAdj[idx] * fontPtSize);

    context.set_color(canvas_ity::stroke_style, 0.0f, 0.0f, 0.0f, 1.0f);
    if(drawAngle[idx] || drawFullCircle)
    {
      ebsdlib::WriteText(context, label, {x, y}, fontPtSize);
    }
  }

  // Draw the [0001] in the center of the image
  if(drawFullCircle)
  {
    float x = figureCenter[0];
    float y = figureCenter[1] + fontPtSize;

    std::string label("[001]");
    ebsdlib::WriteText(context, label, {x, y}, fontPtSize);

    std::vector<ebsdlib::Point3DType> directions = {
        {1.0, 0.0, 1.0},  // Horizontal Meridian Line
        {0.0, 1.0, 1.0},  // Vertical Meridian Line
        {-1.0, 1.0, 0.0}, // Upper Left to Lower Right
        {1.0, 1.0, 0.0},  // Upper Right to Lower Left
        {1.0, 0.0, 0.0},  // Vertical Axis
        {0.0, 1.0, 0.0},  // Horizontal Axis
    };
    int numPoints = 50;
    float penWidth = 1.0f;
    context.set_color(canvas_ity::stroke_style, 0.25f, 0.25f, 0.25f, 1.0f);
    context.set_line_width(penWidth);
    ebsdlib::DrawStereographicLines(context, directions, numPoints, halfWidth, figureOrigin);
  }

  if(!drawFullCircle)
  {
    float x = figureCenter[0];
    float y = figureCenter[1] + fontPtSize;
    std::string label("[001]");
    ebsdlib::WriteText(context, label, {x, y}, fontPtSize);

    x = figureCenter[0] + legendWidth;
    y = figureCenter[1] + fontPtSize;
    label = "[011]";
    ebsdlib::WriteText(context, label, {x, y}, fontPtSize);

    x = figureCenter[0] + legendWidth * 0.90F;
    y = figureCenter[1] - legendHeight * 0.90F;
    label = "[111]";
    ebsdlib::WriteText(context, label, {x, y}, fontPtSize);
  }
}

// -----------------------------------------------------------------------------
ebsdlib::UInt8ArrayType::Pointer CubicOps::generateIPFTriangleLegend(int canvasDim, bool generateEntirePlane, ebsdlib::HexConvention conv, ebsdlib::ColorKeyKind kind, bool gridded) const
{
  // Compute legend dimensions (same formula as annotateIPFImage uses)
  const float fontPtSize = static_cast<float>(canvasDim) / 24.0f;
  const std::vector<float> margins = {fontPtSize * 3, static_cast<float>(canvasDim / 7.0f), fontPtSize * 2, static_cast<float>(canvasDim / 7.0f)};
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

  ebsdlib::IColorKey::Pointer key = keyForKind(kind);
  if(gridded)
  {
    key = std::make_shared<ebsdlib::GriddedColorKey>(key, 1.0);
  }

  // Generate the colored SST triangle image (ARGB)
  ebsdlib::UInt8ArrayType::Pointer image = CreateIPFLegend(this, legendHeight, generateEntirePlane, key.get());

  // Annotate with title and Miller index labels
  return annotateIPFImage(image, legendHeight, canvasDim, getSymmetryName(), generateEntirePlane, /*hasColorBar=*/false, ebsdlib::HexConvention::NotApplicable);
}

std::vector<std::pair<double, double>> CubicOps::rodri2pair(std::vector<double> x, std::vector<double> y, std::vector<double> z)
{
  std::vector<std::pair<double, double>> result;
  double q0, q1, q2, q3, ang, r, x1, y1, z1, rad, xPair, yPair, k;

  for(std::vector<double>::size_type i = 0; i < x.size(); i++)
  {
    // rodri2volpreserv
    q0 = sqrt(1 / (1 + x[i] * x[i] + y[i] * y[i] + z[i] * z[i]));
    q1 = x[i] * q0;
    q2 = y[i] * q0;
    q3 = z[i] * q0;
    ang = acos(q0);
    r = pow(1.5 * (ang - sin(ang) * cos(ang)), (1.0 / 3.0));
    x1 = q1 * r;
    y1 = q2 * r;
    z1 = q3 * r;
    if(sin(ang) != 0)
    {
      x1 = x1 / sin(ang);
      y1 = y1 / sin(ang);
      z1 = z1 / sin(ang);
    }

    // areapreservingx
    rad = sqrt(x1 * x1 + y1 * y1 + z1 * z1);
    if(rad == 0)
    {
      rad++;
    }
    k = 2 * (1 - std::fabs(x1 / rad));
    if(k < 0)
    {
      k = 0;
    }
    k = rad * sqrt(k);
    xPair = y1 * k;
    yPair = z1 * k;
    k = rad * rad - x1 * x1;
    if(k > 0)
    {
      xPair = xPair / sqrt(k);
      yPair = yPair / sqrt(k);
    }
    result.push_back(std::make_pair(xPair, yPair));
  }
  return result;
}

// -----------------------------------------------------------------------------
CubicOps::Pointer CubicOps::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
std::string CubicOps::getNameOfClass() const
{
  return {"CubicOps"};
}

// -----------------------------------------------------------------------------
std::string CubicOps::ClassName()
{
  return {"CubicOps"};
}

// -----------------------------------------------------------------------------
CubicOps::Pointer CubicOps::New()
{
  Pointer sharedPtr(new(CubicOps));
  return sharedPtr;
}
