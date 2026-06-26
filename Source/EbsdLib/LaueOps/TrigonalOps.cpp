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

#include "TrigonalOps.h"

// Include this FIRST because there is a needed define for some compiles
// to expose some of the constants needed below
#include "EbsdLib/Core/EbsdMacros.h"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Orientation/OrientationFwd.hpp"
#include "EbsdLib/Orientation/Quaternion.hpp"
#include "EbsdLib/Utilities/CanvasUtilities.hpp"
#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/ComputeStereographicProjection.h"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"
#include "EbsdLib/Utilities/Fonts.hpp"
#include "EbsdLib/Utilities/FundamentalSectorGeometry.hpp"
#include "EbsdLib/Utilities/GriddedColorKey.hpp"
#include "EbsdLib/Utilities/NolzeHielscherColorKey.hpp"
#include "EbsdLib/Utilities/PUCMColorKey.hpp"
#include "EbsdLib/Utilities/PoleFigureUtilities.h"
#include "EbsdLib/Utilities/TSLColorKey.hpp"

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/task_group.h>
#endif
using namespace ebsdlib;

namespace
{
ebsdlib::IColorKey::Pointer keyForKind(ebsdlib::ColorKeyKind kind)
{
  static const auto k_TSL = std::make_shared<ebsdlib::TSLColorKey>();
  static const auto k_PUCM = std::make_shared<ebsdlib::PUCMColorKey>("32");
  static const auto k_NH = std::make_shared<ebsdlib::NolzeHielscherColorKey>(ebsdlib::FundamentalSectorGeometry::trigonalHigh());
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

namespace TrigonalHigh
{
constexpr std::array<size_t, 3> k_OdfNumBins = {36, 36, 24}; // Represents a 5Deg bin in homochoric space

static const std::array<double, 3> k_OdfDimInitValue = {std::pow((0.75 * (ebsdlib::constants::k_PiOver2D - std::sin(ebsdlib::constants::k_PiOver2D))), (1.0 / 3.0)),
                                                        std::pow((0.75 * (ebsdlib::constants::k_PiOver2D - std::sin(ebsdlib::constants::k_PiOver2D))), (1.0 / 3.0)),
                                                        std::pow((0.75 * (ebsdlib::constants::k_PiOver3D - std::sin(ebsdlib::constants::k_PiOver3D))), (1.0 / 3.0))};
static const std::array<double, 3> k_OdfDimStepValue = {k_OdfDimInitValue[0] / static_cast<double>(k_OdfNumBins[0] / 2), k_OdfDimInitValue[1] / static_cast<double>(k_OdfNumBins[1] / 2),
                                                        k_OdfDimInitValue[2] / static_cast<double>(k_OdfNumBins[2] / 2)};

constexpr int k_SymSize0 = 2;
constexpr int k_SymSize1 = 6;
constexpr int k_SymSize2 = 6;

constexpr size_t k_OdfSize = 31104;
constexpr size_t k_MdfSize = 31104;
constexpr size_t k_SymOpsCount = 6;
constexpr int k_NumMdfBins = 12;

static double sq32 = std::sqrt(3.0) / 2.0;
static const double sqrtThree = std::sqrt(3.0);

// Rotation Point Group: 32
/* clang-format off */
static const std::vector<QuatD> k_QuatSym ={
  QuatD(0.0, 0.0, 0.0, 1.0),
  QuatD(0.0, 0.0, sq32, 0.5),
  QuatD(0.0, 0.0, sq32, -0.5),
  QuatD(1.0, 0.0, 0.0, 0.0),
  QuatD(0.5, sq32, 0.0, 0.0),
  QuatD(-0.5, sq32, 0.0, 0.0),
};

static const std::vector<RodriguesDType> k_RodSym = {
  {0.0, 0.0, 1.0, 0.0},
  {0.0, 0.0, 1.0, sqrtThree},
  {0.0, 0.0, sq32, 10000000000000.0},
  {1.0, 0.0, 0.0, 10000000000000.0},
  {0.5, sq32, 0.0, 10000000000000.0},
  {-0.5, sq32, 0.0, 10000000000000.0},
};

static const std::vector<Matrix3X3D> k_MatSym = {
  {1.0, 0.0, 0.0,
  0.0, 1.0, 0.0,
  0.0, 0.0, 1.0},

  {-0.5, -sq32, 0.0,
  sq32, -0.5, 0.0,
  0.0, 0.0, 1.0},

  {-0.5, sq32, 0.0,
  -sq32, -0.5, 0.0,
  0.0, 0.0, 1.0},

  {1.0, 0.0, 0.0,
  0.0, -1.0, 0.0,
  0.0, 0.0, -1.0},

  {-0.5, sq32, 0.0,
  sq32, 0.5, 0.0,
  0.0, 0.0, -1.0},

  {-0.5, -sq32, 0.0,
  -sq32, 0.5, 0.0,
  -0.0, 0.0, -1.0},

};
/* clang-format on */
constexpr double k_EtaMin = -90.0;
constexpr double k_EtaMax = -30.0;
constexpr double k_ChiMax = 90.0;

// ---------------------------------------------------------------------------
// SymOps: convention-aware bundle of symmetry operations + plane-family
// direction tables. Mirrors the pattern in HexagonalOps.
//
// CANONICAL = X||a* (the v3 hand-typed values above are the MTEX-validated
// source of truth). X||a is derived via 30°-about-c similarity transform.
//
// For TrigonalHigh (Laue class -3m), the canonical k_QuatSym contains 3
// c-axis rotations + 3 basal-plane 180° flips; the 180°s are basis-
// dependent so the X||a derivation via 30°-about-c similarity transform
// yields different sym op values for the basal entries. The plane-family
// direction tables also rotate by 30° between bases.
//
// Note on sym op ordering: the order of entries in k_QuatSym originates
// from the EMsoftOO project, hand-derived for loop efficiency. There is
// no expected mathematical relationship between consecutive entries.
//
// See Code_Review/v3_phase0_design_notes.md §5 for the design pattern and
// §16 for the canonical-direction reasoning.
// ---------------------------------------------------------------------------
struct SymOps
{
  std::vector<QuatD> quat;
  std::vector<RodriguesDType> rod;
  std::vector<Matrix3X3D> mat;

  std::vector<ebsdlib::Matrix3X1D> dirsFamily0; // {0001} c-axis
  std::vector<ebsdlib::Matrix3X1D> dirsFamily1; // <0-110>-style family
  std::vector<ebsdlib::Matrix3X1D> dirsFamily2; // <1-100>-style family

  template <ebsdlib::HexConvention Conv>
  static SymOps build()
  {
    // Canonical (X||a*) plane-family direction sets. Values are taken from
    // the previous inline-hardcoded blocks of TrigonalOps' GenerateSphereCoordsImpl.
    const std::vector<ebsdlib::Matrix3X1D> canonicalDirsFamily0 = {{0.0, 0.0, 1.0}};
    const std::vector<ebsdlib::Matrix3X1D> canonicalDirsFamily1 = {{-0.5, -ebsdlib::constants::k_Root3Over2D, 0.0}, {1.0, 0.0, 0.0}, {-0.5, ebsdlib::constants::k_Root3Over2D, 0.0}};
    const std::vector<ebsdlib::Matrix3X1D> canonicalDirsFamily2 = {{0.5, -ebsdlib::constants::k_Root3Over2D, 0.0}, {0.5, ebsdlib::constants::k_Root3Over2D, 0.0}, {-1.0, 0.0, 0.0}};

    if constexpr(Conv == ebsdlib::HexConvention::XParallelAStar)
    {
      return SymOps{k_QuatSym, k_RodSym, k_MatSym, canonicalDirsFamily0, canonicalDirsFamily1, canonicalDirsFamily2};
    }
    else // XParallelA -- derive by 30°-about-c similarity transform.
    {
      const double sin15 = std::sin(15.0 * ebsdlib::constants::k_PiOver180D);
      const double cos15 = std::cos(15.0 * ebsdlib::constants::k_PiOver180D);
      const QuatD q30(0.0, 0.0, sin15, cos15);
      const QuatD q30Inv = q30.conjugate();

      const double c30 = ebsdlib::constants::k_Root3Over2D;
      const double s30 = 0.5;
      const ebsdlib::Matrix3X3D rz30(c30, -s30, 0.0, s30, c30, 0.0, 0.0, 0.0, 1.0);

      SymOps out;
      out.quat.reserve(k_QuatSym.size());
      out.rod.reserve(k_QuatSym.size());
      out.mat.reserve(k_QuatSym.size());
      for(const auto& qStar : k_QuatSym)
      {
        const QuatD qA = q30 * qStar * q30Inv;
        out.quat.push_back(qA);
        out.mat.push_back(qA.toOrientationMatrix().toGMatrix());
        out.rod.push_back(qA.toRodrigues());
      }

      out.dirsFamily0 = canonicalDirsFamily0; // c-axis: invariant
      out.dirsFamily1.reserve(canonicalDirsFamily1.size());
      out.dirsFamily2.reserve(canonicalDirsFamily2.size());
      for(const auto& d : canonicalDirsFamily1)
      {
        out.dirsFamily1.push_back(rz30 * d);
      }
      for(const auto& d : canonicalDirsFamily2)
      {
        out.dirsFamily2.push_back(rz30 * d);
      }
      return out;
    }
  }
};

static const SymOps k_SymOps_XParallelAStar = SymOps::build<ebsdlib::HexConvention::XParallelAStar>();
static const SymOps k_SymOps_XParallelA = SymOps::build<ebsdlib::HexConvention::XParallelA>();

} // namespace TrigonalHigh

// -----------------------------------------------------------------------------
TrigonalOps::TrigonalOps() = default;

// -----------------------------------------------------------------------------
TrigonalOps::~TrigonalOps() = default;

// -----------------------------------------------------------------------------
bool TrigonalOps::getHasInversion() const
{
  return true;
}

// -----------------------------------------------------------------------------
size_t TrigonalOps::getODFSize() const
{
  return TrigonalHigh::k_OdfSize;
}

// -----------------------------------------------------------------------------
std::array<int32_t, 3> TrigonalOps::getNumSymmetry() const
{
  return {TrigonalHigh::k_SymSize0, TrigonalHigh::k_SymSize1, TrigonalHigh::k_SymSize2};
}

// -----------------------------------------------------------------------------
size_t TrigonalOps::getMDFSize() const
{
  return TrigonalHigh::k_MdfSize;
}

// -----------------------------------------------------------------------------
int TrigonalOps::getMdfPlotBins() const
{
  return TrigonalHigh::k_NumMdfBins;
}

// -----------------------------------------------------------------------------
size_t TrigonalOps::getNumSymOps() const
{
  return TrigonalHigh::k_SymOpsCount;
}

// -----------------------------------------------------------------------------
std::array<size_t, 3> TrigonalOps::getOdfNumBins() const
{
  return TrigonalHigh::k_OdfNumBins;
}

// -----------------------------------------------------------------------------
std::string TrigonalOps::getSymmetryName() const
{
  return "Trigonal -3m (D3d)";
}
// -----------------------------------------------------------------------------
std::string TrigonalOps::getRotationPointGroup() const
{
  return "32";
}

// -----------------------------------------------------------------------------
int TrigonalOps::getPointGroup() const
{
  return 20;
}

// -----------------------------------------------------------------------------
bool TrigonalOps::isInsideFZ(const QuatD& quat) const
{
  return IsInsideFZ(quat, getFZType(), getAxisOrderingType());
}

// -----------------------------------------------------------------------------
bool TrigonalOps::isInsideFZ(const RodriguesDType& rod) const
{
  return IsInsideFZ(rod, getFZType(), getAxisOrderingType());
}

AxisAngleDType TrigonalOps::calculateMisorientation(const QuatD& q1, const QuatD& q2) const
{
  return calculateMisorientationInternal(TrigonalHigh::k_QuatSym, q1, q2);
}

// -----------------------------------------------------------------------------
QuatD TrigonalOps::getQuatSymOp(size_t i) const
{
  return TrigonalHigh::k_QuatSym[i];
}

size_t TrigonalOps::getNumRodriguesSymOps() const
{
  return TrigonalHigh::k_RodSym.size();
}

RodriguesDType TrigonalOps::getRodSymOp(size_t i) const
{
  return TrigonalHigh::k_RodSym[i];
}

Matrix3X3D TrigonalOps::getMatSymOpD(size_t i) const
{
  return TrigonalHigh::k_MatSym[i];
}

Matrix3X3F TrigonalOps::getMatSymOpF(size_t i) const
{
  return {static_cast<float>(TrigonalHigh::k_MatSym[i](0, 0)), static_cast<float>(TrigonalHigh::k_MatSym[i](0, 1)), static_cast<float>(TrigonalHigh::k_MatSym[i](0, 2)),
          static_cast<float>(TrigonalHigh::k_MatSym[i](1, 0)), static_cast<float>(TrigonalHigh::k_MatSym[i](1, 1)), static_cast<float>(TrigonalHigh::k_MatSym[i](1, 2)),
          static_cast<float>(TrigonalHigh::k_MatSym[i](2, 0)), static_cast<float>(TrigonalHigh::k_MatSym[i](2, 1)), static_cast<float>(TrigonalHigh::k_MatSym[i](2, 2))};
}

// -----------------------------------------------------------------------------
RodriguesDType TrigonalOps::getODFFZRod(const RodriguesDType& rod) const
{
  return _calcRodNearestOrigin(rod);
}

// -----------------------------------------------------------------------------
RodriguesDType TrigonalOps::getMDFFZRod(const RodriguesDType& inRod) const
{
  double w = 0.0, n1 = 0.0, n2 = 0.0, n3 = 0.0;
  double FZn1 = 0.0, FZn2 = 0.0, FZn3 = 0.0, FZw = 0.0;
  double n1n2mag = 0.0f;

  RodriguesDType rod = _calcRodNearestOrigin(inRod);

  AxisAngleDType ax = rod.toAxisAngle();

  n1 = ax[0];
  n2 = ax[1], n3 = ax[2], w = ax[3];

  float denom = static_cast<float>(std::sqrt((n1 * n1 + n2 * n2 + n3 * n3)));
  n1 = n1 / denom;
  n2 = n2 / denom;
  n3 = n3 / denom;
  if(n3 < 0)
  {
    n1 = -n1, n2 = -n2, n3 = -n3;
  }
  float angle = static_cast<float>(180.0 * std::atan2(n2, n1) * ebsdlib::constants::k_1OverPiD);
  if(angle < 0)
  {
    angle = angle + 360.0f;
  }
  FZn1 = n1;
  FZn2 = n2;
  FZn3 = n3;
  if(angle > 60.0f)
  {
    n1n2mag = std::sqrt(n1 * n1 + n2 * n2);
    if(int(angle / 60) % 2 == 0)
    {
      FZw = angle - (60.0f * int(angle / 60.0f));
      FZw = FZw * ebsdlib::constants::k_PiOver180D;
      FZn1 = n1n2mag * std::cos(FZw);
      FZn2 = n1n2mag * std::sin(FZw);
    }
    else
    {
      FZw = angle - (60.0f * int(angle / 60.0f));
      FZw = 60.0f - FZw;
      FZw = FZw * ebsdlib::constants::k_PiOver180D;
      FZn1 = n1n2mag * std::cos(FZw);
      FZn2 = n1n2mag * std::sin(FZw);
    }
  }

  return AxisAngleDType(FZn1, FZn2, FZn3, FZw).toRodrigues();
}

// -----------------------------------------------------------------------------
QuatD TrigonalOps::getNearestQuat(const QuatD& q1, const QuatD& q2) const
{
  return _calcNearestQuat(TrigonalHigh::k_QuatSym, q1, q2);
}
QuatF TrigonalOps::getNearestQuat(const QuatF& q1f, const QuatF& q2f) const
{
  return _calcNearestQuat(TrigonalHigh::k_QuatSym, q1f.to<double>(), q2f.to<double>()).to<float>();
}

// -----------------------------------------------------------------------------
QuatD TrigonalOps::getFZQuat(const QuatD& qr) const
{
  LaueOps::FZType fzType = laue_ops::FZtarray[getPointGroup() - 1];
  LaueOps::AxisOrderingType orderingType = laue_ops::FZoarray[getPointGroup() - 1];
  return ConvertToFZ(TrigonalHigh::k_QuatSym, qr, fzType, orderingType);
}

// -----------------------------------------------------------------------------
int TrigonalOps::getMisoBin(const RodriguesDType& rod) const
{
  double dim[3];
  double bins[3];
  double step[3];

  HomochoricDType ho = rod.toHomochoric();

  dim[0] = TrigonalHigh::k_OdfDimInitValue[0];
  dim[1] = TrigonalHigh::k_OdfDimInitValue[1];
  dim[2] = TrigonalHigh::k_OdfDimInitValue[2];
  step[0] = TrigonalHigh::k_OdfDimStepValue[0];
  step[1] = TrigonalHigh::k_OdfDimStepValue[1];
  step[2] = TrigonalHigh::k_OdfDimStepValue[2];
  bins[0] = static_cast<double>(TrigonalHigh::k_OdfNumBins[0]);
  bins[1] = static_cast<double>(TrigonalHigh::k_OdfNumBins[1]);
  bins[2] = static_cast<double>(TrigonalHigh::k_OdfNumBins[2]);

  return _calcMisoBin(dim, bins, step, ho);
}

// -----------------------------------------------------------------------------
EulerDType TrigonalOps::determineEulerAngles(double random[3], int choose) const
{
  double init[3];
  double step[3];
  int32_t phi[3];
  double h1, h2, h3;

  init[0] = TrigonalHigh::k_OdfDimInitValue[0];
  init[1] = TrigonalHigh::k_OdfDimInitValue[1];
  init[2] = TrigonalHigh::k_OdfDimInitValue[2];
  step[0] = TrigonalHigh::k_OdfDimStepValue[0];
  step[1] = TrigonalHigh::k_OdfDimStepValue[1];
  step[2] = TrigonalHigh::k_OdfDimStepValue[2];
  phi[0] = static_cast<int32_t>(choose % TrigonalHigh::k_OdfNumBins[0]);
  phi[1] = static_cast<int32_t>((choose / TrigonalHigh::k_OdfNumBins[0]) % TrigonalHigh::k_OdfNumBins[1]);
  phi[2] = static_cast<int32_t>(choose / (TrigonalHigh::k_OdfNumBins[0] * TrigonalHigh::k_OdfNumBins[1]));

  _calcDetermineHomochoricValues(random, init, step, phi, h1, h2, h3);

  RodriguesDType ro = HomochoricDType(h1, h2, h3).toRodrigues();
  ro = getODFFZRod(ro);
  EulerDType eu = ro.toEuler();
  return eu;
}

// -----------------------------------------------------------------------------
EulerDType TrigonalOps::randomizeEulerAngles(const EulerDType& synea) const
{
  size_t symOp = getRandomSymmetryOperatorIndex(TrigonalHigh::k_SymOpsCount);
  QuatD quat = synea.toQuaternion();
  QuatD qc = TrigonalHigh::k_QuatSym[symOp] * quat;
  return QuaternionDType(qc).toEuler();
}

// -----------------------------------------------------------------------------
RodriguesDType TrigonalOps::determineRodriguesVector(double random[3], int choose) const
{
  double init[3];
  double step[3];
  int32_t phi[3];
  double h1, h2, h3;

  init[0] = TrigonalHigh::k_OdfDimInitValue[0];
  init[1] = TrigonalHigh::k_OdfDimInitValue[1];
  init[2] = TrigonalHigh::k_OdfDimInitValue[2];
  step[0] = TrigonalHigh::k_OdfDimStepValue[0];
  step[1] = TrigonalHigh::k_OdfDimStepValue[1];
  step[2] = TrigonalHigh::k_OdfDimStepValue[2];
  phi[0] = static_cast<int32_t>(choose % TrigonalHigh::k_OdfNumBins[0]);
  phi[1] = static_cast<int32_t>((choose / TrigonalHigh::k_OdfNumBins[0]) % TrigonalHigh::k_OdfNumBins[1]);
  phi[2] = static_cast<int32_t>(choose / (TrigonalHigh::k_OdfNumBins[0] * TrigonalHigh::k_OdfNumBins[1]));

  _calcDetermineHomochoricValues(random, init, step, phi, h1, h2, h3);
  RodriguesDType ro = HomochoricDType(h1, h2, h3).toRodrigues();
  ro = getMDFFZRod(ro);
  return ro;
}

int TrigonalOps::getOdfBin(const RodriguesDType& rod) const
{
  double dim[3];
  double bins[3];
  double step[3];

  HomochoricDType ho = rod.toHomochoric();

  dim[0] = TrigonalHigh::k_OdfDimInitValue[0];
  dim[1] = TrigonalHigh::k_OdfDimInitValue[1];
  dim[2] = TrigonalHigh::k_OdfDimInitValue[2];
  step[0] = TrigonalHigh::k_OdfDimStepValue[0];
  step[1] = TrigonalHigh::k_OdfDimStepValue[1];
  step[2] = TrigonalHigh::k_OdfDimStepValue[2];
  bins[0] = static_cast<double>(TrigonalHigh::k_OdfNumBins[0]);
  bins[1] = static_cast<double>(TrigonalHigh::k_OdfNumBins[1]);
  bins[2] = static_cast<double>(TrigonalHigh::k_OdfNumBins[2]);

  return _calcODFBin(dim, bins, step, ho);
}

void TrigonalOps::getSchmidFactorAndSS(double load[3], double& schmidfactor, double angleComps[2], int& slipsys) const
{
  schmidfactor = 0;
  slipsys = 0;
}

void TrigonalOps::getSchmidFactorAndSS(double load[3], double plane[3], double direction[3], double& schmidfactor, double angleComps[2], int& slipsys) const
{
  schmidfactor = 0;
  slipsys = 0;
  angleComps[0] = 0;
  angleComps[1] = 0;

  // compute mags
  double loadMag = sqrt(load[0] * load[0] + load[1] * load[1] + load[2] * load[2]);
  double planeMag = sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
  double directionMag = sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
  planeMag *= loadMag;
  directionMag *= loadMag;

  // loop over symmetry operators finding highest schmid factor
  for(int i = 0; i < TrigonalHigh::k_SymOpsCount; i++)
  {
    // compute slip system
    double slipPlane[3] = {0};
    slipPlane[2] = TrigonalHigh::k_MatSym[i](2, 0) * plane[0] + TrigonalHigh::k_MatSym[i](2, 1) * plane[1] + TrigonalHigh::k_MatSym[i](2, 2) * plane[2];

    // dont consider negative z planes (to avoid duplicates)
    if(slipPlane[2] >= 0)
    {
      slipPlane[0] = TrigonalHigh::k_MatSym[i](0, 0) * plane[0] + TrigonalHigh::k_MatSym[i](0, 1) * plane[1] + TrigonalHigh::k_MatSym[i](0, 2) * plane[2];
      slipPlane[1] = TrigonalHigh::k_MatSym[i](1, 0) * plane[0] + TrigonalHigh::k_MatSym[i](1, 1) * plane[1] + TrigonalHigh::k_MatSym[i](1, 2) * plane[2];

      double slipDirection[3] = {0};
      slipDirection[0] = TrigonalHigh::k_MatSym[i](0, 0) * direction[0] + TrigonalHigh::k_MatSym[i](0, 1) * direction[1] + TrigonalHigh::k_MatSym[i](0, 2) * direction[2];
      slipDirection[1] = TrigonalHigh::k_MatSym[i](1, 0) * direction[0] + TrigonalHigh::k_MatSym[i](1, 1) * direction[1] + TrigonalHigh::k_MatSym[i](1, 2) * direction[2];
      slipDirection[2] = TrigonalHigh::k_MatSym[i](2, 0) * direction[0] + TrigonalHigh::k_MatSym[i](2, 1) * direction[1] + TrigonalHigh::k_MatSym[i](2, 2) * direction[2];

      const double cosPhi = fabs(load[0] * slipPlane[0] + load[1] * slipPlane[1] + load[2] * slipPlane[2]) / planeMag;
      const double cosLambda = fabs(load[0] * slipDirection[0] + load[1] * slipDirection[1] + load[2] * slipDirection[2]) / directionMag;

      double schmid = cosPhi * cosLambda;
      if(schmid > schmidfactor)
      {
        schmidfactor = schmid;
        slipsys = i;
        angleComps[0] = acos(cosPhi);
        angleComps[1] = acos(cosLambda);
      }
    }
  }
}

double TrigonalOps::getmPrime(const QuatD& q1, const QuatD& q2, double LD[3]) const
{
  return 0.0;
}

double TrigonalOps::getF1(const QuatD& q1, const QuatD& q2, double LD[3], bool maxS) const
{
  return 0.0;
}

double TrigonalOps::getF1spt(const QuatD& q1, const QuatD& q2, double LD[3], bool maxS) const
{
  return 0.0;
}

double TrigonalOps::getF7(const QuatD& q1, const QuatD& q2, double LD[3], bool maxS) const
{
  return 0.0;
}
// -----------------------------------------------------------------------------

namespace TrigonalHigh
{
class GenerateSphereCoordsImpl
{
  ebsdlib::FloatArrayType* m_Eulers;
  ebsdlib::FloatArrayType* m_xyz001;
  ebsdlib::FloatArrayType* m_xyz011;
  ebsdlib::FloatArrayType* m_xyz111;
  const SymOps* m_Sym;

public:
  GenerateSphereCoordsImpl(ebsdlib::FloatArrayType* eulerAngles, ebsdlib::FloatArrayType* xyz001Coords, ebsdlib::FloatArrayType* xyz011Coords, ebsdlib::FloatArrayType* xyz111Coords, const SymOps* sym)
  : m_Eulers(eulerAngles)
  , m_xyz001(xyz001Coords)
  , m_xyz011(xyz011Coords)
  , m_xyz111(xyz111Coords)
  , m_Sym(sym)
  {
  }
  virtual ~GenerateSphereCoordsImpl() = default;

  static inline void emitDirAndAntipode(const ebsdlib::Matrix3X3D& gTranspose, const ebsdlib::Matrix3X1D& dir, ebsdlib::FloatArrayType* dest, size_t pairOffsetTuples)
  {
    const size_t plus = pairOffsetTuples * 3;
    const size_t minus = plus + 3;
    (gTranspose * dir).copyInto<float>(dest->getPointer(plus));
    std::transform(dest->getPointer(plus), dest->getPointer(plus + 3), dest->getPointer(minus), [](float v) { return v * -1.0F; });
  }

  void generate(size_t start, size_t end) const
  {
    const size_t f0Stride = m_Sym->dirsFamily0.size() * 2;
    const size_t f1Stride = m_Sym->dirsFamily1.size() * 2;
    const size_t f2Stride = m_Sym->dirsFamily2.size() * 2;

    for(size_t i = start; i < end; ++i)
    {
      EulerDType euler(m_Eulers->getValue(i * 3), m_Eulers->getValue(i * 3 + 1), m_Eulers->getValue(i * 3 + 2));
      ebsdlib::Matrix3X3D gTranspose = euler.toOrientationMatrix().toGMatrix().transpose();

      for(size_t k = 0; k < m_Sym->dirsFamily0.size(); ++k)
      {
        emitDirAndAntipode(gTranspose, m_Sym->dirsFamily0[k], m_xyz001, i * f0Stride + k * 2);
      }
      for(size_t k = 0; k < m_Sym->dirsFamily1.size(); ++k)
      {
        emitDirAndAntipode(gTranspose, m_Sym->dirsFamily1[k], m_xyz011, i * f1Stride + k * 2);
      }
      for(size_t k = 0; k < m_Sym->dirsFamily2.size(); ++k)
      {
        emitDirAndAntipode(gTranspose, m_Sym->dirsFamily2[k], m_xyz111, i * f2Stride + k * 2);
      }
    }
  }

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  void operator()(const tbb::blocked_range<size_t>& r) const
  {
    generate(r.begin(), r.end());
  }
#endif
};
} // namespace TrigonalHigh

// -----------------------------------------------------------------------------
void TrigonalOps::generateSphereCoordsFromEulers(ebsdlib::FloatArrayType* eulers, ebsdlib::FloatArrayType* xyz001, ebsdlib::FloatArrayType* xyz011, ebsdlib::FloatArrayType* xyz111,
                                                 ebsdlib::HexConvention conv) const
{
  size_t nOrientations = eulers->getNumberOfTuples();

  // Sanity Check the size of the arrays
  if(xyz001->getNumberOfTuples() < nOrientations * TrigonalHigh::k_SymSize0)
  {
    xyz001->resizeTuples(nOrientations * TrigonalHigh::k_SymSize0 * 3);
  }
  if(xyz011->getNumberOfTuples() < nOrientations * TrigonalHigh::k_SymSize1)
  {
    xyz011->resizeTuples(nOrientations * TrigonalHigh::k_SymSize1 * 3);
  }
  if(xyz111->getNumberOfTuples() < nOrientations * TrigonalHigh::k_SymSize2)
  {
    xyz111->resizeTuples(nOrientations * TrigonalHigh::k_SymSize2 * 3);
  }

  // Pick the convention-appropriate SymOps once.
  const TrigonalHigh::SymOps* sym = (conv == ebsdlib::HexConvention::XParallelAStar) ? &TrigonalHigh::k_SymOps_XParallelAStar : &TrigonalHigh::k_SymOps_XParallelA;

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  bool doParallel = true;
  if(doParallel)
  {
    tbb::parallel_for(tbb::blocked_range<size_t>(0, nOrientations), TrigonalHigh::GenerateSphereCoordsImpl(eulers, xyz001, xyz011, xyz111, sym), tbb::auto_partitioner());
  }
  else
#endif
  {
    TrigonalHigh::GenerateSphereCoordsImpl serial(eulers, xyz001, xyz011, xyz111, sym);
    serial.generate(0, nOrientations);
  }
}

// -----------------------------------------------------------------------------
std::array<double, 3> TrigonalOps::getIpfColorAngleLimits(double eta) const
{
  return {TrigonalHigh::k_EtaMin * ebsdlib::constants::k_DegToRadD, TrigonalHigh::k_EtaMax * ebsdlib::constants::k_DegToRadD, TrigonalHigh::k_ChiMax * ebsdlib::constants::k_DegToRadD};
}

// -----------------------------------------------------------------------------
bool TrigonalOps::inUnitTriangle(double eta, double chi) const
{
  return !(eta < (TrigonalHigh::k_EtaMin * ebsdlib::constants::k_PiOver180D) || eta > (TrigonalHigh::k_EtaMax * ebsdlib::constants::k_PiOver180D) || chi < 0 ||
           chi > (TrigonalHigh::k_ChiMax * ebsdlib::constants::k_PiOver180D));
}

// -----------------------------------------------------------------------------
ebsdlib::Rgb TrigonalOps::generateIPFColor(double* eulers, double* refDir, bool degToRad, ebsdlib::ColorKeyKind kind) const
{
  return computeIPFColor(eulers, refDir, degToRad, keyForKind(kind).get());
}

// -----------------------------------------------------------------------------
ebsdlib::Rgb TrigonalOps::generateIPFColor(double phi1, double phi, double phi2, double refDir0, double refDir1, double refDir2, bool degToRad, ebsdlib::ColorKeyKind kind) const
{
  double eulers[3] = {phi1, phi, phi2};
  double refDir[3] = {refDir0, refDir1, refDir2};
  return computeIPFColor(eulers, refDir, degToRad, keyForKind(kind).get());
}

// -----------------------------------------------------------------------------
ebsdlib::Rgb TrigonalOps::generateRodriguesColor(double r1, double r2, double r3) const
{
  double range1 = 2.0f * TrigonalHigh::k_OdfDimInitValue[0];
  double range2 = 2.0f * TrigonalHigh::k_OdfDimInitValue[1];
  double range3 = 2.0f * TrigonalHigh::k_OdfDimInitValue[2];
  double max1 = range1 / 2.0f;
  double max2 = range2 / 2.0f;
  double max3 = range3 / 2.0f;
  double red = (r1 + max1) / range1;
  double green = (r2 + max2) / range2;
  double blue = (r3 + max3) / range3;

  return ebsdlib::RgbColor::dRgb(static_cast<int32_t>(red * 255), static_cast<int32_t>(green * 255), static_cast<int32_t>(blue * 255), 255);
}

// -----------------------------------------------------------------------------
std::array<std::string, 3> TrigonalOps::getDefaultPoleFigureNames(ebsdlib::HexConvention conv) const
{
  // TrigonalHigh (-3m) has two distinct prism families and the OIM/MTEX
  // labeling-tradition split that hex 6/mmm has does not apply cleanly.
  // The conv parameter is plumbed for API uniformity but the returned
  // strings are the same under both conventions for now; revisit if a
  // user reports a specific OIM/MTEX label divergence here.
  (void)conv;
  return {"{0001}", "{0-110}", "{1-100}"};
}

// -----------------------------------------------------------------------------
std::vector<ebsdlib::UInt8ArrayType::Pointer> TrigonalOps::generatePoleFigure(PoleFigureConfiguration_t& config) const
{
  std::array<std::string, 3> labels = getDefaultPoleFigureNames(config.hexConvention);
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
  ebsdlib::FloatArrayType::Pointer xyz001 = ebsdlib::FloatArrayType::CreateArray(numOrientations * TrigonalHigh::k_SymSize0, dims, label0 + std::string("xyzCoords"), true);
  // this is size for CUBIC ONLY, <011> Family
  ebsdlib::FloatArrayType::Pointer xyz011 = ebsdlib::FloatArrayType::CreateArray(numOrientations * TrigonalHigh::k_SymSize1, dims, label1 + std::string("xyzCoords"), true);
  // this is size for CUBIC ONLY, <111> Family
  ebsdlib::FloatArrayType::Pointer xyz111 = ebsdlib::FloatArrayType::CreateArray(numOrientations * TrigonalHigh::k_SymSize2, dims, label2 + std::string("xyzCoords"), true);

  config.sphereRadius = 1.0f;

  // Generate the coords on the sphere **** Parallelized
  generateSphereCoordsFromEulers(config.eulers, xyz001.get(), xyz011.get(), xyz111.get(), config.hexConvention);

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

  // Find the Max and Min values based on ALL 3 arrays so we can color scale them all the same
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
  ebsdlib::UInt8ArrayType::Pointer image001 = ebsdlib::UInt8ArrayType::CreateArray(config.imageDim * config.imageDim, dims, label0, true);
  ebsdlib::UInt8ArrayType::Pointer image011 = ebsdlib::UInt8ArrayType::CreateArray(config.imageDim * config.imageDim, dims, label1, true);
  ebsdlib::UInt8ArrayType::Pointer image111 = ebsdlib::UInt8ArrayType::CreateArray(config.imageDim * config.imageDim, dims, label2, true);

  std::vector<ebsdlib::UInt8ArrayType::Pointer> poleFigures(3);
  if(config.order.size() == 3)
  {
    poleFigures[config.order[0]] = image001;
    poleFigures[config.order[1]] = image011;
    poleFigures[config.order[2]] = image111;
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

  return poleFigures;
}
namespace
{
// -----------------------------------------------------------------------------
ebsdlib::UInt8ArrayType::Pointer CreateIPFLegend(const TrigonalOps* ops, int imageDim, bool generateEntirePlane, const ebsdlib::IColorKey* key)
{
  std::vector<size_t> dims(1, 4);
  std::string arrayName = EbsdStringUtils::replace(ops->getSymmetryName(), "/", "_");
  ebsdlib::UInt8ArrayType::Pointer image = ebsdlib::UInt8ArrayType::CreateArray(imageDim * imageDim, dims, arrayName + " Triangle Legend", true);
  uint32_t* pixelPtr = reinterpret_cast<uint32_t*>(image->getPointer(0));
  static ebsdlib::Matrix3X1D k_Orientation(0.0, 0.0, 00.0 * ebsdlib::constants::k_PiOver180D);

  double xInc = 1.0f / static_cast<double>(imageDim);
  double yInc = 1.0f / static_cast<double>(imageDim);

  // Find the slope of the bounding line.
  static const double m = std::sin(30.0 * ebsdlib::constants::k_PiOver180D) / std::cos(30.0 * ebsdlib::constants::k_PiOver180D);

  size_t yScanLineIndex = 0; // We use this to control where the data is drawn. Otherwise, the image will come out flipped vertically
  // Loop over every pixel in the image and project up to the sphere to get the angle and then figure out the RGB from
  // there.
  for(int32_t yIndex = 0; yIndex < imageDim; ++yIndex)
  {
    for(int32_t xIndex = 0; xIndex < imageDim; ++xIndex)
    {
      size_t idx = (imageDim * yScanLineIndex) + xIndex;

      double x = -1.0f + 2.0f * xIndex * xInc;
      double y = -1.0f + 2.0f * yIndex * yInc;
      if(!generateEntirePlane)
      {
        x = -1.0f + 2.0f * xIndex * xInc;
        y = -1.0f + 2.0f * yIndex * yInc;
      }
      auto sphericalCoords = stereographic::utils::StereoToSpherical(x, y).normalize();
      ebsdlib::Rgb color = 0xFFFFFFFF;
      double sumSquares = (x * x) + (y * y);

      if(sumSquares > 1.0f) // Outside unit circle
      {
        color = 0xFFFFFFFF;
      }
      else if(!generateEntirePlane && (x < 0.0 || y > 0.0))
      {
        color = 0xFFFFFFFF;
      }
      else if(!generateEntirePlane && std::fabs(sphericalCoords[1] / sphericalCoords[0]) < m)
      {
        color = 0xFFFFFFFF;
      }
      else
      {
        color = ops->computeIPFColor(k_Orientation.data(), sphericalCoords.data(), false, key);
      }

      pixelPtr[idx] = color;
    }
    yScanLineIndex++;
  }
  return image;
}

} // namespace

// -----------------------------------------------------------------------------
bool TrigonalOps::mapPixelToSphereSST(int xPixel, int yPixel, int imageDim, std::array<float, 3>& sphereDir) const
{
  double xInc = 1.0 / static_cast<double>(imageDim);
  double yInc = 1.0 / static_cast<double>(imageDim);

  double x = -1.0 + 2.0 * xPixel * xInc;
  double y = -1.0 + 2.0 * yPixel * yInc;

  double sumSquares = (x * x) + (y * y);
  if(sumSquares > 1.0)
  {
    return false;
  }

  if(x < 0.0 || y > 0.0)
  {
    return false;
  }

  auto sc = stereographic::utils::StereoToSpherical(x, y).normalize();

  // Find the slope of the bounding line.
  static const double m = std::sin(30.0 * ebsdlib::constants::k_PiOver180D) / std::cos(30.0 * ebsdlib::constants::k_PiOver180D);

  if(std::fabs(sc[1] / sc[0]) < m)
  {
    return false;
  }

  sphereDir[0] = static_cast<float>(sc[0]);
  sphereDir[1] = static_cast<float>(sc[1]);
  sphereDir[2] = static_cast<float>(sc[2]);
  return true;
}

// -----------------------------------------------------------------------------
std::array<float, 2> TrigonalOps::adjustFigureOrigin(std::array<float, 2> figureOrigin, int legendWidth, int legendHeight, const std::vector<float>& margins, float fontPtSize,
                                                     bool generateEntirePlane) const
{
  if(!generateEntirePlane)
  {
    figureOrigin[0] = -(legendWidth / 2) * 0.25;
    figureOrigin[1] = 0.0F - (legendHeight / 2) * .5;
  }
  return figureOrigin;
}

// -----------------------------------------------------------------------------
void TrigonalOps::drawIPFAnnotations(canvas_ity::canvas& context, int canvasDim, float fontPtSize, const std::vector<float>& margins, std::array<float, 2> figureOrigin,
                                     std::array<float, 2> figureCenter, bool drawFullCircle, ebsdlib::HexConvention conv) const
{
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
  int pageHeight = canvasDim;
  int pageWidth = canvasDim;
  int halfWidth = legendWidth / 2;
  int halfHeight = legendHeight / 2;

  std::vector<float> angles = {0.0f, 30.0f, 60.0f, 90.0f, 120.0f, 150.0f, 180.0f, 210.0f, 240.0f, 270.0f, 300.0f, 330.0f};

  // See HexagonalOps::drawIPFAnnotations for the X||a / X||a* label-table reasoning.
  static const std::vector<std::string> labels_X_a = {"[2-1-10]", "[10-10]", "[11-20]", "[01-10]", "[-12-10]", "[-1100]", "[-2110]", "[-1010]", "[-1-120]", "[0-110]", "[1-210]", "[1-100]"};
  static const std::vector<std::string> labels_X_astar = {"[10-10]", "[11-20]", "[01-10]", "[-12-10]", "[-1100]", "[-2110]", "[-1010]", "[-1-120]", "[0-110]", "[1-210]", "[1-100]", "[2-1-10]"};
  const std::vector<std::string>& labels2 = (conv == ebsdlib::HexConvention::XParallelA) ? labels_X_a : labels_X_astar;

  std::vector<float> xAdj = {
      0.1F, 0.0F, 0.0F, -0.5F, -1.0F, -1.0F, -1.1F, -1.1F, -1.1F, -0.5F, 0.0F, 0.0F,
  };
  std::vector<float> yAdj = {
      +0.25F, 0.0F, 0.0F, -0.1F, 0.0F, 0.0F, 0.25F, 0.5F, 1.0F, 1.1F, 1.0F, 1.0F,
  };
  std::vector<bool> drawAngle = {false, false, false, false, false, false, false, false, false, true, true, true};

  for(size_t idx = 0; idx < angles.size(); idx++)
  {
    float radius = 1.0f;
    float angle = angles[idx];
    float rads = angle * ebsdlib::constants::k_DegToRadF;
    float x = radius * (cos(rads));
    float y = radius * (sin(rads));

    // Transform from Unit Circle to our flipped Screen Pixel Coordinates
    // First Scale up to our image dimensions
    x = x * halfWidth;
    y = y * halfHeight;

    // Next, translate to the center of the image
    x = x + halfWidth;
    y = y + halfHeight;

    // Now mirror across the x-axis (vertically) because this is the transformation from
    // cartesian coords to screen coords
    y = legendHeight - y;

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
  {
    std::string label("[0001]");
    std::string fontWidthString = EbsdStringUtils::replace(label, "-", "");
    float fontWidth = context.measure_text(fontWidthString.c_str());
    float x = figureCenter[0] - fontWidth * 0.5;
    float y = figureCenter[1] - fontPtSize * 0.25;
    ebsdlib::WriteText(context, label, {x, y}, fontPtSize);
  }
}

// -----------------------------------------------------------------------------
ebsdlib::UInt8ArrayType::Pointer TrigonalOps::generateIPFTriangleLegend(int canvasDim, bool generateEntirePlane, ebsdlib::HexConvention conv, ebsdlib::ColorKeyKind kind, bool gridded) const
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
  return annotateIPFImage(image, legendHeight, canvasDim, getSymmetryName(), generateEntirePlane, false, conv);
}

// -----------------------------------------------------------------------------
TrigonalOps::Pointer TrigonalOps::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
std::string TrigonalOps::getNameOfClass() const
{
  return {"TrigonalOps"};
}

// -----------------------------------------------------------------------------
std::string TrigonalOps::ClassName()
{
  return {"TrigonalOps"};
}

// -----------------------------------------------------------------------------
TrigonalOps::Pointer TrigonalOps::New()
{
  Pointer sharedPtr(new(TrigonalOps));
  return sharedPtr;
}
