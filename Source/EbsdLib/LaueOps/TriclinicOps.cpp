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

#include "TriclinicOps.h"

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
#include <tbb/partitioner.h>
#include <tbb/task.h>
#include <tbb/task_group.h>
#endif
using namespace ebsdlib;

namespace
{
ebsdlib::IColorKey::Pointer keyForKind(ebsdlib::ColorKeyKind kind)
{
  static const auto k_TSL = std::make_shared<ebsdlib::TSLColorKey>();
  static const auto k_PUCM = std::make_shared<ebsdlib::PUCMColorKey>("1");
  static const auto k_NH = std::make_shared<ebsdlib::NolzeHielscherColorKey>(ebsdlib::FundamentalSectorGeometry::triclinic());
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

namespace Triclinic
{
constexpr std::array<size_t, 3> k_OdfNumBins = {72, 72, 72}; // Represents a 5Deg bin in homochoric space

static const std::array<double, 3> k_OdfDimInitValue = {std::pow((0.75 * ((ebsdlib::constants::k_PiD)-std::sin((ebsdlib::constants::k_PiD)))), (1.0 / 3.0)),
                                                        std::pow((0.75 * ((ebsdlib::constants::k_PiD)-std::sin((ebsdlib::constants::k_PiD)))), (1.0 / 3.0)),
                                                        std::pow((0.75 * ((ebsdlib::constants::k_PiD)-std::sin((ebsdlib::constants::k_PiD)))), (1.0 / 3.0))};
static const std::array<double, 3> k_OdfDimStepValue = {k_OdfDimInitValue[0] / static_cast<double>(k_OdfNumBins[0] / 2), k_OdfDimInitValue[1] / static_cast<double>(k_OdfNumBins[1] / 2),
                                                        k_OdfDimInitValue[2] / static_cast<double>(k_OdfNumBins[2] / 2)};

constexpr int k_SymSize0 = 2;
constexpr int k_SymSize1 = 2;
constexpr int k_SymSize2 = 2;

constexpr size_t k_OdfSize = 373248;
constexpr size_t k_MdfSize = 373248;
constexpr size_t k_SymOpsCount = 1;
constexpr int k_NumMdfBins = 36;
// Rotation Point Group: 1
/* clang-format off */
static const std::vector<QuatD> k_QuatSym ={
  QuatD(0.0, 0.0, 0.0, 1.0),
};

static const std::vector<RodriguesDType> k_RodSym = {
  {0.0, 0.0, 1.0, 0.0},
};

static const std::vector<Matrix3X3D> k_MatSym = {
  {1.0, 0.0, 0.0,
  0.0, 1.0, 0.0,
  0.0, 0.0, 1.0},

};
/* clang-format on */

constexpr double k_EtaMin = 0.0;
constexpr double k_EtaMax = 180.0;
constexpr double k_ChiMax = 90.0;

} // namespace Triclinic

// -----------------------------------------------------------------------------
TriclinicOps::TriclinicOps() = default;

// -----------------------------------------------------------------------------
TriclinicOps::~TriclinicOps() = default;

// -----------------------------------------------------------------------------
bool TriclinicOps::getHasInversion() const
{
  return true;
}

// -----------------------------------------------------------------------------
size_t TriclinicOps::getODFSize() const
{
  return Triclinic::k_OdfSize;
}

// -----------------------------------------------------------------------------
std::array<int32_t, 3> TriclinicOps::getNumSymmetry() const
{
  return {Triclinic::k_SymSize0, Triclinic::k_SymSize1, Triclinic::k_SymSize2};
}

// -----------------------------------------------------------------------------
size_t TriclinicOps::getMDFSize() const
{
  return Triclinic::k_MdfSize;
}

// -----------------------------------------------------------------------------
int TriclinicOps::getMdfPlotBins() const
{
  return Triclinic::k_NumMdfBins;
}

// -----------------------------------------------------------------------------
size_t TriclinicOps::getNumSymOps() const
{
  return Triclinic::k_SymOpsCount;
}

// -----------------------------------------------------------------------------
std::array<size_t, 3> TriclinicOps::getOdfNumBins() const
{
  return Triclinic::k_OdfNumBins;
}

// -----------------------------------------------------------------------------
std::string TriclinicOps::getSymmetryName() const
{
  return "Triclinic -1 (Ci)";
}

// -----------------------------------------------------------------------------
std::string TriclinicOps::getRotationPointGroup() const
{
  return "1";
}

// -----------------------------------------------------------------------------
int TriclinicOps::getPointGroup() const
{
  return 2;
}

// -----------------------------------------------------------------------------
bool TriclinicOps::isInsideFZ(const QuatD& quat) const
{
  return IsInsideFZ(quat, getFZType(), getAxisOrderingType());
}

// -----------------------------------------------------------------------------
bool TriclinicOps::isInsideFZ(const RodriguesDType& rod) const
{
  return IsInsideFZ(rod, getFZType(), getAxisOrderingType());
}

// -----------------------------------------------------------------------------
AxisAngleDType TriclinicOps::calculateMisorientation(const QuatD& q1, const QuatD& q2) const
{
  return calculateMisorientationInternal(Triclinic::k_QuatSym, q1, q2);
}

QuatD TriclinicOps::getQuatSymOp(size_t i) const
{
  return Triclinic::k_QuatSym[i];
}

size_t TriclinicOps::getNumRodriguesSymOps() const
{
  return Triclinic::k_RodSym.size();
}

RodriguesDType TriclinicOps::getRodSymOp(size_t i) const
{
  return Triclinic::k_RodSym[i];
}

Matrix3X3D TriclinicOps::getMatSymOpD(size_t i) const
{
  return Triclinic::k_MatSym[i];
}

Matrix3X3F TriclinicOps::getMatSymOpF(size_t i) const
{
  return {static_cast<float>(Triclinic::k_MatSym[i](0, 0)), static_cast<float>(Triclinic::k_MatSym[i](0, 1)), static_cast<float>(Triclinic::k_MatSym[i](0, 2)),
          static_cast<float>(Triclinic::k_MatSym[i](1, 0)), static_cast<float>(Triclinic::k_MatSym[i](1, 1)), static_cast<float>(Triclinic::k_MatSym[i](1, 2)),
          static_cast<float>(Triclinic::k_MatSym[i](2, 0)), static_cast<float>(Triclinic::k_MatSym[i](2, 1)), static_cast<float>(Triclinic::k_MatSym[i](2, 2))};
}

// -----------------------------------------------------------------------------
RodriguesDType TriclinicOps::getODFFZRod(const RodriguesDType& rod) const
{
  return _calcRodNearestOrigin(rod);
}

// -----------------------------------------------------------------------------
RodriguesDType TriclinicOps::getMDFFZRod(const RodriguesDType& inRod) const
{
  throw ebsdlib::method_not_implemented("TriclinicOps::getMDFFZRod not implemented");

  RodriguesDType rod = LaueOps::_calcRodNearestOrigin(inRod);

  AxisAngleDType ax = rod.toAxisAngle();
  /// FIXME: Are we missing code for TriclinicOps MDF FZ Rodrigues calculation?

  return ax.toRodrigues();
}

// -----------------------------------------------------------------------------
QuatD TriclinicOps::getNearestQuat(const QuatD& q1, const QuatD& q2) const
{
  return _calcNearestQuat(Triclinic::k_QuatSym, q1, q2);
}
QuatF TriclinicOps::getNearestQuat(const QuatF& q1f, const QuatF& q2f) const
{
  return _calcNearestQuat(Triclinic::k_QuatSym, q1f.to<double>(), q2f.to<double>()).to<float>();
}

// -----------------------------------------------------------------------------
QuatD TriclinicOps::getFZQuat(const QuatD& qr) const
{
  LaueOps::FZType fzType = laue_ops::FZtarray[getPointGroup() - 1];
  LaueOps::AxisOrderingType orderingType = laue_ops::FZoarray[getPointGroup() - 1];
  return ConvertToFZ(Triclinic::k_QuatSym, qr, fzType, orderingType);
}

// -----------------------------------------------------------------------------
int TriclinicOps::getMisoBin(const RodriguesDType& rod) const
{
  double dim[3];
  double bins[3];
  double step[3];

  HomochoricDType ho = rod.toHomochoric();

  dim[0] = Triclinic::k_OdfDimInitValue[0];
  dim[1] = Triclinic::k_OdfDimInitValue[1];
  dim[2] = Triclinic::k_OdfDimInitValue[2];
  step[0] = Triclinic::k_OdfDimStepValue[0];
  step[1] = Triclinic::k_OdfDimStepValue[1];
  step[2] = Triclinic::k_OdfDimStepValue[2];
  bins[0] = static_cast<double>(Triclinic::k_OdfNumBins[0]);
  bins[1] = static_cast<double>(Triclinic::k_OdfNumBins[1]);
  bins[2] = static_cast<double>(Triclinic::k_OdfNumBins[2]);

  return _calcMisoBin(dim, bins, step, ho);
}

// -----------------------------------------------------------------------------
EulerDType TriclinicOps::determineEulerAngles(double random[3], int choose) const
{
  double init[3];
  double step[3];
  int32_t phi[3];
  double h1, h2, h3;

  init[0] = Triclinic::k_OdfDimInitValue[0];
  init[1] = Triclinic::k_OdfDimInitValue[1];
  init[2] = Triclinic::k_OdfDimInitValue[2];
  step[0] = Triclinic::k_OdfDimStepValue[0];
  step[1] = Triclinic::k_OdfDimStepValue[1];
  step[2] = Triclinic::k_OdfDimStepValue[2];
  phi[0] = static_cast<int32_t>(choose % Triclinic::k_OdfNumBins[0]);
  phi[1] = static_cast<int32_t>((choose / Triclinic::k_OdfNumBins[0]) % Triclinic::k_OdfNumBins[1]);
  phi[2] = static_cast<int32_t>(choose / (Triclinic::k_OdfNumBins[0] * Triclinic::k_OdfNumBins[1]));

  _calcDetermineHomochoricValues(random, init, step, phi, h1, h2, h3);

  RodriguesDType ro = HomochoricDType(h1, h2, h3).toRodrigues();
  ro = getODFFZRod(ro);
  EulerDType eu = ro.toEuler();
  return eu;
}

// -----------------------------------------------------------------------------
EulerDType TriclinicOps::randomizeEulerAngles(const EulerDType& synea) const
{
  size_t symOp = getRandomSymmetryOperatorIndex(Triclinic::k_SymOpsCount);
  QuatD quat = synea.toQuaternion();
  QuatD qc = Triclinic::k_QuatSym[symOp] * quat;
  return QuaternionDType(qc).toEuler();
}

// -----------------------------------------------------------------------------
RodriguesDType TriclinicOps::determineRodriguesVector(double random[3], int choose) const
{
  double init[3];
  double step[3];
  int32_t phi[3];
  double h1, h2, h3;

  init[0] = Triclinic::k_OdfDimInitValue[0];
  init[1] = Triclinic::k_OdfDimInitValue[1];
  init[2] = Triclinic::k_OdfDimInitValue[2];
  step[0] = Triclinic::k_OdfDimStepValue[0];
  step[1] = Triclinic::k_OdfDimStepValue[1];
  step[2] = Triclinic::k_OdfDimStepValue[2];
  phi[0] = static_cast<int32_t>(choose % Triclinic::k_OdfNumBins[0]);
  phi[1] = static_cast<int32_t>((choose / Triclinic::k_OdfNumBins[0]) % Triclinic::k_OdfNumBins[1]);
  phi[2] = static_cast<int32_t>(choose / (Triclinic::k_OdfNumBins[0] * Triclinic::k_OdfNumBins[1]));

  _calcDetermineHomochoricValues(random, init, step, phi, h1, h2, h3);
  RodriguesDType ro = HomochoricDType(h1, h2, h3).toRodrigues();
  ro = getMDFFZRod(ro);
  return ro;
}

// -----------------------------------------------------------------------------
int TriclinicOps::getOdfBin(const RodriguesDType& rod) const
{
  double dim[3];
  double bins[3];
  double step[3];

  HomochoricDType ho = rod.toHomochoric();

  dim[0] = Triclinic::k_OdfDimInitValue[0];
  dim[1] = Triclinic::k_OdfDimInitValue[1];
  dim[2] = Triclinic::k_OdfDimInitValue[2];
  step[0] = Triclinic::k_OdfDimStepValue[0];
  step[1] = Triclinic::k_OdfDimStepValue[1];
  step[2] = Triclinic::k_OdfDimStepValue[2];
  bins[0] = static_cast<double>(Triclinic::k_OdfNumBins[0]);
  bins[1] = static_cast<double>(Triclinic::k_OdfNumBins[1]);
  bins[2] = static_cast<double>(Triclinic::k_OdfNumBins[2]);

  return _calcODFBin(dim, bins, step, ho);
}

void TriclinicOps::getSchmidFactorAndSS(double load[3], double& schmidfactor, double angleComps[2], int& slipsys) const
{
  schmidfactor = 0;
  slipsys = 0;
}

void TriclinicOps::getSchmidFactorAndSS(double load[3], double plane[3], double direction[3], double& schmidfactor, double angleComps[2], int& slipsys) const
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
  for(int i = 0; i < Triclinic::k_SymOpsCount; i++)
  {
    // compute slip system
    double slipPlane[3] = {0};
    slipPlane[2] = Triclinic::k_MatSym[i](2, 0) * plane[0] + Triclinic::k_MatSym[i](2, 1) * plane[1] + Triclinic::k_MatSym[i](2, 2) * plane[2];

    // dont consider negative z planes (to avoid duplicates)
    if(slipPlane[2] >= 0)
    {
      slipPlane[0] = Triclinic::k_MatSym[i](0, 0) * plane[0] + Triclinic::k_MatSym[i](0, 1) * plane[1] + Triclinic::k_MatSym[i](0, 2) * plane[2];
      slipPlane[1] = Triclinic::k_MatSym[i](1, 0) * plane[0] + Triclinic::k_MatSym[i](1, 1) * plane[1] + Triclinic::k_MatSym[i](1, 2) * plane[2];

      double slipDirection[3] = {0};
      slipDirection[0] = Triclinic::k_MatSym[i](0, 0) * direction[0] + Triclinic::k_MatSym[i](0, 1) * direction[1] + Triclinic::k_MatSym[i](0, 2) * direction[2];
      slipDirection[1] = Triclinic::k_MatSym[i](1, 0) * direction[0] + Triclinic::k_MatSym[i](1, 1) * direction[1] + Triclinic::k_MatSym[i](1, 2) * direction[2];
      slipDirection[2] = Triclinic::k_MatSym[i](2, 0) * direction[0] + Triclinic::k_MatSym[i](2, 1) * direction[1] + Triclinic::k_MatSym[i](2, 2) * direction[2];

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

double TriclinicOps::getmPrime(const QuatD& q1, const QuatD& q2, double LD[3]) const
{
  return 0.0;
}

double TriclinicOps::getF1(const QuatD& q1, const QuatD& q2, double LD[3], bool maxS) const
{
  return 0.0;
}

double TriclinicOps::getF1spt(const QuatD& q1, const QuatD& q2, double LD[3], bool maxS) const
{
  return 0.0;
}

double TriclinicOps::getF7(const QuatD& q1, const QuatD& q2, double LD[3], bool maxS) const
{
  return 0.0;
}
// -----------------------------------------------------------------------------

namespace TriclinicHigh
{
class GenerateSphereCoordsImpl
{
  ebsdlib::FloatArrayType* m_Eulers;
  ebsdlib::FloatArrayType* m_xyz001;
  ebsdlib::FloatArrayType* m_xyz011;
  ebsdlib::FloatArrayType* m_xyz111;

public:
  GenerateSphereCoordsImpl(ebsdlib::FloatArrayType* eulerAngles, ebsdlib::FloatArrayType* xyz001Coords, ebsdlib::FloatArrayType* xyz011Coords, ebsdlib::FloatArrayType* xyz111Coords)
  : m_Eulers(eulerAngles)
  , m_xyz001(xyz001Coords)
  , m_xyz011(xyz011Coords)
  , m_xyz111(xyz111Coords)
  {
  }
  virtual ~GenerateSphereCoordsImpl() = default;

  void generate(size_t start, size_t end) const
  {
    ebsdlib::Matrix3X1D direction(0.0, 0.0, 0.0);

    // Generate all the Coordinates
    for(size_t i = start; i < end; ++i)
    {
      EulerDType euler(m_Eulers->getValue(i * 3), m_Eulers->getValue(i * 3 + 1), m_Eulers->getValue(i * 3 + 2));
      ebsdlib::Matrix3X3D gTranspose = euler.toOrientationMatrix().toGMatrix().transpose();

      // -----------------------------------------------------------------------------
      // 001 Family
      direction[0] = 0.0;
      direction[1] = 0.0;
      direction[2] = 1.0;
      (gTranspose * direction).copyInto<float>(m_xyz001->getPointer(i * 6));
      std::transform(m_xyz001->getPointer(i * 6), m_xyz001->getPointer(i * 6 + 3),
                     m_xyz001->getPointer(i * 6 + 3),            // write to the next triplet in memory
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0

      // -----------------------------------------------------------------------------
      // 011 Family
      direction[0] = 1.0;
      direction[1] = 0.0;
      direction[2] = 0.0;
      (gTranspose * direction).copyInto<float>(m_xyz011->getPointer(i * 6));
      std::transform(m_xyz011->getPointer(i * 6), m_xyz011->getPointer(i * 6 + 3),
                     m_xyz011->getPointer(i * 6 + 3),            // write to the next triplet in memory
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0

      // -----------------------------------------------------------------------------
      // 111 Family
      direction[0] = 0.0;
      direction[1] = 1.0;
      direction[2] = 0;
      (gTranspose * direction).copyInto<float>(m_xyz111->getPointer(i * 6));
      std::transform(m_xyz111->getPointer(i * 6), m_xyz111->getPointer(i * 6 + 3),
                     m_xyz111->getPointer(i * 6 + 3),            // write to the next triplet in memory
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
} // namespace TriclinicHigh

// -----------------------------------------------------------------------------
void TriclinicOps::generateSphereCoordsFromEulers(ebsdlib::FloatArrayType* eulers, ebsdlib::FloatArrayType* xyz001, ebsdlib::FloatArrayType* xyz011, ebsdlib::FloatArrayType* xyz111,
                                                  ebsdlib::HexConvention conv) const
{
  size_t nOrientations = eulers->getNumberOfTuples();

  // Sanity Check the size of the arrays
  if(xyz001->getNumberOfTuples() < nOrientations * Triclinic::k_SymSize0)
  {
    xyz001->resizeTuples(nOrientations * Triclinic::k_SymSize0 * 3);
  }
  if(xyz011->getNumberOfTuples() < nOrientations * Triclinic::k_SymSize1)
  {
    xyz011->resizeTuples(nOrientations * Triclinic::k_SymSize1 * 3);
  }
  if(xyz111->getNumberOfTuples() < nOrientations * Triclinic::k_SymSize2)
  {
    xyz111->resizeTuples(nOrientations * Triclinic::k_SymSize2 * 3);
  }

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  bool doParallel = true;
  if(doParallel)
  {
    tbb::parallel_for(tbb::blocked_range<size_t>(0, nOrientations), TriclinicHigh::GenerateSphereCoordsImpl(eulers, xyz001, xyz011, xyz111), tbb::auto_partitioner());
  }
  else
#endif
  {
    TriclinicHigh::GenerateSphereCoordsImpl serial(eulers, xyz001, xyz011, xyz111);
    serial.generate(0, nOrientations);
  }
}

// -----------------------------------------------------------------------------
std::array<double, 3> TriclinicOps::getIpfColorAngleLimits(double eta) const
{
  return {Triclinic::k_EtaMin * ebsdlib::constants::k_DegToRadD, Triclinic::k_EtaMax * ebsdlib::constants::k_DegToRadD, Triclinic::k_ChiMax * ebsdlib::constants::k_DegToRadD};
}

// -----------------------------------------------------------------------------
bool TriclinicOps::inUnitTriangle(double eta, double chi) const
{
  return !(eta < (Triclinic::k_EtaMin * ebsdlib::constants::k_PiOver180D) || eta > (Triclinic::k_EtaMax * ebsdlib::constants::k_PiOver180D) || chi < 0 ||
           chi > (Triclinic::k_ChiMax * ebsdlib::constants::k_PiOver180D));
}

// -----------------------------------------------------------------------------
ebsdlib::Rgb TriclinicOps::generateIPFColor(double* eulers, double* refDir, bool degToRad, ebsdlib::ColorKeyKind kind) const
{
  return computeIPFColor(eulers, refDir, degToRad, keyForKind(kind).get());
}

// -----------------------------------------------------------------------------
ebsdlib::Rgb TriclinicOps::generateIPFColor(double phi1, double phi, double phi2, double refDir0, double refDir1, double refDir2, bool degToRad, ebsdlib::ColorKeyKind kind) const
{
  double eulers[3] = {phi1, phi, phi2};
  double refDir[3] = {refDir0, refDir1, refDir2};
  return computeIPFColor(eulers, refDir, degToRad, keyForKind(kind).get());
}

// -----------------------------------------------------------------------------
ebsdlib::Rgb TriclinicOps::generateRodriguesColor(double r1, double r2, double r3) const
{
  double range1 = 2.0f * Triclinic::k_OdfDimInitValue[0];
  double range2 = 2.0f * Triclinic::k_OdfDimInitValue[1];
  double range3 = 2.0f * Triclinic::k_OdfDimInitValue[2];
  double max1 = range1 / 2.0f;
  double max2 = range2 / 2.0f;
  double max3 = range3 / 2.0f;
  double red = (r1 + max1) / range1;
  double green = (r2 + max2) / range2;
  double blue = (r3 + max3) / range3;

  // Scale values from 0 to 1.0
  red = red / max1;
  green = green / max1;
  blue = blue / max2;

  return ebsdlib::RgbColor::dRgb(static_cast<int32_t>(red * 255), static_cast<int32_t>(green * 255), static_cast<int32_t>(blue * 255), 255);
}

// -----------------------------------------------------------------------------
std::array<std::string, 3> TriclinicOps::getDefaultPoleFigureNames(ebsdlib::HexConvention conv) const
{
  return {"{001}", "{100}", "{010}"};
}

// -----------------------------------------------------------------------------
std::vector<ebsdlib::UInt8ArrayType::Pointer> TriclinicOps::generatePoleFigure(PoleFigureConfiguration_t& config) const
{
  std::array<std::string, 3> labels = getDefaultPoleFigureNames();
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
  ebsdlib::FloatArrayType::Pointer xyz001 = ebsdlib::FloatArrayType::CreateArray(numOrientations * Triclinic::k_SymSize0, dims, label0 + std::string("xyzCoords"), true);
  // this is size for CUBIC ONLY, <011> Family
  ebsdlib::FloatArrayType::Pointer xyz011 = ebsdlib::FloatArrayType::CreateArray(numOrientations * Triclinic::k_SymSize1, dims, label1 + std::string("xyzCoords"), true);
  // this is size for CUBIC ONLY, <111> Family
  ebsdlib::FloatArrayType::Pointer xyz111 = ebsdlib::FloatArrayType::CreateArray(numOrientations * Triclinic::k_SymSize2, dims, label2 + std::string("xyzCoords"), true);

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
ebsdlib::UInt8ArrayType::Pointer CreateIPFLegend(const TriclinicOps* ops, int imageDim, bool generateEntirePlane, const ebsdlib::IColorKey* key)
{
  std::vector<size_t> dims(1, 4);
  std::string arrayName = EbsdStringUtils::replace(ops->getSymmetryName(), "/", "_");
  ebsdlib::UInt8ArrayType::Pointer image = ebsdlib::UInt8ArrayType::CreateArray(imageDim * imageDim, dims, arrayName + " Triangle Legend", true);
  uint32_t* pixelPtr = reinterpret_cast<uint32_t*>(image->getPointer(0));

  double xInc = 1.0f / static_cast<double>(imageDim);
  double yInc = 1.0f / static_cast<double>(imageDim);
  static ebsdlib::Matrix3X1D k_Orientation(0.0, 0.0, 0.0);

  size_t yScanLineIndex = 0; // We use this to control where the data is drawn. Otherwise, the image will come out flipped vertically
  // Loop over every pixel in the image and project up to the sphere to get the angle and then figure out the RGB from
  // there.
  for(int32_t yIndex = 0; yIndex < imageDim; ++yIndex)
  {

    for(int32_t xIndex = 0; xIndex < imageDim; ++xIndex)
    {
      size_t idx = (imageDim * yScanLineIndex) + xIndex;

      // Always compute entire unit circle
      double x = -1.0f + 2.0f * xIndex * xInc;
      double y = -1.0f + 2.0f * yIndex * yInc;

      double sumSquares = (x * x) + (y * y);
      ebsdlib::Rgb color = 0xFFFFFFFF; // Default to white

      if(sumSquares > 1.0) // Outside unit circle
      {
        color = 0xFFFFFFFF;
      }
      else
      {
        auto sphericalCoords = stereographic::utils::StereoToSpherical(x, y).normalize();
        color = ops->computeIPFColor(k_Orientation.data(), sphericalCoords.data(), false, key);
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
bool TriclinicOps::mapPixelToSphereSST(int xPixel, int yPixel, int imageDim, std::array<float, 3>& sphereDir) const
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

  auto sc = stereographic::utils::StereoToSpherical(x, y).normalize();

  sphereDir[0] = static_cast<float>(sc[0]);
  sphereDir[1] = static_cast<float>(sc[1]);
  sphereDir[2] = static_cast<float>(sc[2]);
  return true;
}

// -----------------------------------------------------------------------------
void TriclinicOps::drawIPFAnnotations(canvas_ity::canvas& context, int canvasDim, float fontPtSize, const std::vector<float>& margins, std::array<float, 2> figureOrigin,
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

  std::vector<float> angles = {0.0f, 45.0F, 90.0F, 135.0F, 180.0F, 225.0F, 270.0F, 315.0F};
  std::vector<std::string> labels2 = {
      "[100]", "[110]", "[010]", "[-110]", "[-100]", "[-1-10]", "[0-10]", "[1-10]",
  };

  std::vector<float> xAdj = {0.1F, 0.0F, -0.5F, -1.0F, -1.1F, -1.0F, -0.5F, 0.0F};
  std::vector<float> yAdj = {+0.25F, 0.0F, -0.2F, 0.0F, 0.25F, 0.75F, 1.1F, 1.0F};
  std::vector<bool> drawAngle = {true, false, true, false, true, false, true, false};

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
    //    if(drawAngle[idx] || drawFullCircle)
    //    {
    //      float penWidth = 1.0f;
    //      context.set_color(canvas_ity::stroke_style, 0.25f, 0.25f, 0.25f, 1.0f);
    //      context.set_line_width(penWidth);
    //      ebsdlib::DrawLine(context, figureCenter[0], figureCenter[1], x, y);
    //    }
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
    float x = figureCenter[0];
    float y = figureCenter[1] - fontPtSize * 0.2F;

    std::string label("[001]");
    ebsdlib::WriteText(context, label, {x, y}, fontPtSize);
  }

  if(drawFullCircle)
  {
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
}

// -----------------------------------------------------------------------------
ebsdlib::UInt8ArrayType::Pointer TriclinicOps::generateIPFTriangleLegend(int canvasDim, bool generateEntirePlane, ebsdlib::HexConvention conv, ebsdlib::ColorKeyKind kind, bool gridded) const
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

  // Generate the colored SST triangle image (ARGB)
  ebsdlib::IColorKey::Pointer key = keyForKind(kind);
  if(gridded)
  {
    key = std::make_shared<ebsdlib::GriddedColorKey>(key, 1.0);
  }
  ebsdlib::UInt8ArrayType::Pointer image = CreateIPFLegend(this, legendHeight, generateEntirePlane, key.get());

  // Annotate with title and Miller index labels
  return annotateIPFImage(image, legendHeight, canvasDim, getSymmetryName(), generateEntirePlane, /*hasColorBar=*/false, ebsdlib::HexConvention::NotApplicable);
}

// -----------------------------------------------------------------------------
TriclinicOps::Pointer TriclinicOps::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
std::string TriclinicOps::getNameOfClass() const
{
  return {"TriclinicOps"};
}

// -----------------------------------------------------------------------------
std::string TriclinicOps::ClassName()
{
  return {"TriclinicOps"};
}

// -----------------------------------------------------------------------------
TriclinicOps::Pointer TriclinicOps::New()
{
  Pointer sharedPtr(new(TriclinicOps));
  return sharedPtr;
}
