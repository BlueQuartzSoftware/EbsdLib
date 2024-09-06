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

#include "TriclinicOps.h"

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/partitioner.h>
#include <tbb/task.h>
#include <tbb/task_group.h>
#endif

// Include this FIRST because there is a needed define for some compiles
// to expose some of the constants needed below
#include "EbsdLib/Core/EbsdMacros.h"
#include "EbsdLib/Core/Orientation.hpp"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/ComputeStereographicProjection.h"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"
#include "EbsdLib/Utilities/PoleFigureUtilities.h"

namespace Triclinic
{
static const std::array<size_t, 3> OdfNumBins = {72, 72, 72}; // Represents a 5Deg bin

static const std::array<double, 3> OdfDimInitValue = {std::pow((0.75 * ((EbsdLib::Constants::k_PiD)-std::sin((EbsdLib::Constants::k_PiD)))), (1.0 / 3.0)),
                                                      std::pow((0.75 * ((EbsdLib::Constants::k_PiD)-std::sin((EbsdLib::Constants::k_PiD)))), (1.0 / 3.0)),
                                                      std::pow((0.75 * ((EbsdLib::Constants::k_PiD)-std::sin((EbsdLib::Constants::k_PiD)))), (1.0 / 3.0))};
static const std::array<double, 3> OdfDimStepValue = {OdfDimInitValue[0] / static_cast<double>(OdfNumBins[0] / 2), OdfDimInitValue[1] / static_cast<double>(OdfNumBins[1] / 2),
                                                      OdfDimInitValue[2] / static_cast<double>(OdfNumBins[2] / 2)};

static const int symSize0 = 2;
static const int symSize1 = 2;
static const int symSize2 = 2;

static const int k_OdfSize = 373248;
static const int k_MdfSize = 373248;
static const int k_SymOpsCount = 1;
static const int k_NumMdfBins = 36;
// Rotation Point Group: 1
// clang-format off
static const std::vector<QuatD> QuatSym ={
    QuatD(0.0, 0.0, 0.0, 1.0),
};

static const std::vector<OrientationD> RodSym = {
    {0.0, 0.0, 1.0, 0.0},
};

static const double MatSym[k_SymOpsCount][3][3] = {
    {{1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}},
    
};
// clang-format on

static const double k_EtaMin = 0.0;
static const double k_EtaMax = 180.0;
static const double k_ChiMax = 90.0;

} // namespace Triclinic

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
TriclinicOps::TriclinicOps() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
TriclinicOps::~TriclinicOps() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool TriclinicOps::getHasInversion() const
{
  return true;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int TriclinicOps::getODFSize() const
{
  return Triclinic::k_OdfSize;
}

// -----------------------------------------------------------------------------
std::array<int32_t, 3> TriclinicOps::getNumSymmetry() const
{
  return {Triclinic::symSize0, Triclinic::symSize1, Triclinic::symSize2};
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int TriclinicOps::getMDFSize() const
{
  return Triclinic::k_MdfSize;
}

// -----------------------------------------------------------------------------
int TriclinicOps::getMdfPlotBins() const
{
  return Triclinic::k_NumMdfBins;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int TriclinicOps::getNumSymOps() const
{
  return Triclinic::k_SymOpsCount;
}

// -----------------------------------------------------------------------------
std::array<size_t, 3> TriclinicOps::getOdfNumBins() const
{
  return Triclinic::OdfNumBins;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string TriclinicOps::getSymmetryName() const
{
  return "Triclinic -1 (Ci)";
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string TriclinicOps::getRotationPointGroup() const
{
  return "1";
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
OrientationD TriclinicOps::calculateMisorientation(const QuatD& q1, const QuatD& q2) const
{
  return calculateMisorientationInternal(Triclinic::QuatSym, q1, q2);
}

// -----------------------------------------------------------------------------
OrientationF TriclinicOps::calculateMisorientation(const QuatF& q1f, const QuatF& q2f) const
{
  QuatD q1 = q1f.to<double>();
  QuatD q2 = q2f.to<double>();
  OrientationD axisAngle = calculateMisorientationInternal(Triclinic::QuatSym, q1, q2);
  return axisAngle;
}

QuatD TriclinicOps::getQuatSymOp(int32_t i) const
{
  return Triclinic::QuatSym[i];
}

void TriclinicOps::getRodSymOp(int i, double* r) const
{
  r[0] = Triclinic::RodSym[i][0];
  r[1] = Triclinic::RodSym[i][1];
  r[2] = Triclinic::RodSym[i][2];
}

EbsdLib::Matrix3X3D TriclinicOps::getMatSymOpD(int i) const
{
  return {Triclinic::MatSym[i][0][0], Triclinic::MatSym[i][0][1], Triclinic::MatSym[i][0][2], Triclinic::MatSym[i][1][0], Triclinic::MatSym[i][1][1],
          Triclinic::MatSym[i][1][2], Triclinic::MatSym[i][2][0], Triclinic::MatSym[i][2][1], Triclinic::MatSym[i][2][2]};
}

EbsdLib::Matrix3X3F TriclinicOps::getMatSymOpF(int i) const
{
  return {static_cast<float>(Triclinic::MatSym[i][0][0]), static_cast<float>(Triclinic::MatSym[i][0][1]), static_cast<float>(Triclinic::MatSym[i][0][2]),
          static_cast<float>(Triclinic::MatSym[i][1][0]), static_cast<float>(Triclinic::MatSym[i][1][1]), static_cast<float>(Triclinic::MatSym[i][1][2]),
          static_cast<float>(Triclinic::MatSym[i][2][0]), static_cast<float>(Triclinic::MatSym[i][2][1]), static_cast<float>(Triclinic::MatSym[i][2][2])};
}

void TriclinicOps::getMatSymOp(int i, double g[3][3]) const
{
  g[0][0] = Triclinic::MatSym[i][0][0];
  g[0][1] = Triclinic::MatSym[i][0][1];
  g[0][2] = Triclinic::MatSym[i][0][2];
  g[1][0] = Triclinic::MatSym[i][1][0];
  g[1][1] = Triclinic::MatSym[i][1][1];
  g[1][2] = Triclinic::MatSym[i][1][2];
  g[2][0] = Triclinic::MatSym[i][2][0];
  g[2][1] = Triclinic::MatSym[i][2][1];
  g[2][2] = Triclinic::MatSym[i][2][2];
}

void TriclinicOps::getMatSymOp(int i, float g[3][3]) const
{
  g[0][0] = static_cast<float>(Triclinic::MatSym[i][0][0]);
  g[0][1] = static_cast<float>(Triclinic::MatSym[i][0][1]);
  g[0][2] = static_cast<float>(Triclinic::MatSym[i][0][2]);
  g[1][0] = static_cast<float>(Triclinic::MatSym[i][1][0]);
  g[1][1] = static_cast<float>(Triclinic::MatSym[i][1][1]);
  g[1][2] = static_cast<float>(Triclinic::MatSym[i][1][2]);
  g[2][0] = static_cast<float>(Triclinic::MatSym[i][2][0]);
  g[2][1] = static_cast<float>(Triclinic::MatSym[i][2][1]);
  g[2][2] = static_cast<float>(Triclinic::MatSym[i][2][2]);
}
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
OrientationType TriclinicOps::getODFFZRod(const OrientationType& rod) const
{
  return _calcRodNearestOrigin(Triclinic::RodSym, rod);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
OrientationType TriclinicOps::getMDFFZRod(const OrientationType& inRod) const
{
  throw EbsdLib::method_not_implemented("TriclinicOps::getMDFFZRod not implemented");

  OrientationType rod = LaueOps::_calcRodNearestOrigin(Triclinic::RodSym, inRod);

  OrientationType ax = OrientationTransformation::ro2ax<OrientationType, OrientationType>(rod);
  /// FIXME: Are we missing code for TriclinicOps MDF FZ Rodrigues calculation?

  return OrientationTransformation::ax2ro<OrientationType, OrientationType>(ax);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QuatD TriclinicOps::getNearestQuat(const QuatD& q1, const QuatD& q2) const
{
  return _calcNearestQuat(Triclinic::QuatSym, q1, q2);
}
QuatF TriclinicOps::getNearestQuat(const QuatF& q1f, const QuatF& q2f) const
{
  return _calcNearestQuat(Triclinic::QuatSym, q1f.to<double>(), q2f.to<double>()).to<float>();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int TriclinicOps::getMisoBin(const OrientationType& rod) const
{
  double dim[3];
  double bins[3];
  double step[3];

  OrientationType ho = OrientationTransformation::ro2ho<OrientationType, OrientationType>(rod);

  dim[0] = Triclinic::OdfDimInitValue[0];
  dim[1] = Triclinic::OdfDimInitValue[1];
  dim[2] = Triclinic::OdfDimInitValue[2];
  step[0] = Triclinic::OdfDimStepValue[0];
  step[1] = Triclinic::OdfDimStepValue[1];
  step[2] = Triclinic::OdfDimStepValue[2];
  bins[0] = static_cast<double>(Triclinic::OdfNumBins[0]);
  bins[1] = static_cast<double>(Triclinic::OdfNumBins[1]);
  bins[2] = static_cast<double>(Triclinic::OdfNumBins[2]);

  return _calcMisoBin(dim, bins, step, ho);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
OrientationType TriclinicOps::determineEulerAngles(double random[3], int choose) const
{
  double init[3];
  double step[3];
  int32_t phi[3];
  double h1, h2, h3;

  init[0] = Triclinic::OdfDimInitValue[0];
  init[1] = Triclinic::OdfDimInitValue[1];
  init[2] = Triclinic::OdfDimInitValue[2];
  step[0] = Triclinic::OdfDimStepValue[0];
  step[1] = Triclinic::OdfDimStepValue[1];
  step[2] = Triclinic::OdfDimStepValue[2];
  phi[0] = static_cast<int32_t>(choose % Triclinic::OdfNumBins[0]);
  phi[1] = static_cast<int32_t>((choose / Triclinic::OdfNumBins[0]) % Triclinic::OdfNumBins[1]);
  phi[2] = static_cast<int32_t>(choose / (Triclinic::OdfNumBins[0] * Triclinic::OdfNumBins[1]));

  _calcDetermineHomochoricValues(random, init, step, phi, h1, h2, h3);

  OrientationType ho(h1, h2, h3);
  OrientationType ro = OrientationTransformation::ho2ro<OrientationType, OrientationType>(ho);
  ro = getODFFZRod(ro);
  OrientationType eu = OrientationTransformation::ro2eu<OrientationType, OrientationType>(ro);
  return eu;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
OrientationType TriclinicOps::randomizeEulerAngles(const OrientationType& synea) const
{
  size_t symOp = getRandomSymmetryOperatorIndex(Triclinic::k_SymOpsCount);
  QuatD quat = OrientationTransformation::eu2qu<OrientationType, QuatD>(synea);
  QuatD qc = Triclinic::QuatSym[symOp] * quat;
  return OrientationTransformation::qu2eu<QuatD, OrientationType>(qc);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
OrientationType TriclinicOps::determineRodriguesVector(double random[3], int choose) const
{
  double init[3];
  double step[3];
  int32_t phi[3];
  double h1, h2, h3;

  init[0] = Triclinic::OdfDimInitValue[0];
  init[1] = Triclinic::OdfDimInitValue[1];
  init[2] = Triclinic::OdfDimInitValue[2];
  step[0] = Triclinic::OdfDimStepValue[0];
  step[1] = Triclinic::OdfDimStepValue[1];
  step[2] = Triclinic::OdfDimStepValue[2];
  phi[0] = static_cast<int32_t>(choose % Triclinic::OdfNumBins[0]);
  phi[1] = static_cast<int32_t>((choose / Triclinic::OdfNumBins[0]) % Triclinic::OdfNumBins[1]);
  phi[2] = static_cast<int32_t>(choose / (Triclinic::OdfNumBins[0] * Triclinic::OdfNumBins[1]));

  _calcDetermineHomochoricValues(random, init, step, phi, h1, h2, h3);
  OrientationType ho(h1, h2, h3);
  OrientationType ro = OrientationTransformation::ho2ro<OrientationType, OrientationType>(ho);
  ro = getMDFFZRod(ro);
  return ro;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int TriclinicOps::getOdfBin(const OrientationType& rod) const
{
  double dim[3];
  double bins[3];
  double step[3];

  OrientationType ho = OrientationTransformation::ro2ho<OrientationType, OrientationType>(rod);

  dim[0] = Triclinic::OdfDimInitValue[0];
  dim[1] = Triclinic::OdfDimInitValue[1];
  dim[2] = Triclinic::OdfDimInitValue[2];
  step[0] = Triclinic::OdfDimStepValue[0];
  step[1] = Triclinic::OdfDimStepValue[1];
  step[2] = Triclinic::OdfDimStepValue[2];
  bins[0] = static_cast<double>(Triclinic::OdfNumBins[0]);
  bins[1] = static_cast<double>(Triclinic::OdfNumBins[1]);
  bins[2] = static_cast<double>(Triclinic::OdfNumBins[2]);

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
    slipPlane[2] = Triclinic::MatSym[i][2][0] * plane[0] + Triclinic::MatSym[i][2][1] * plane[1] + Triclinic::MatSym[i][2][2] * plane[2];

    // dont consider negative z planes (to avoid duplicates)
    if(slipPlane[2] >= 0)
    {
      slipPlane[0] = Triclinic::MatSym[i][0][0] * plane[0] + Triclinic::MatSym[i][0][1] * plane[1] + Triclinic::MatSym[i][0][2] * plane[2];
      slipPlane[1] = Triclinic::MatSym[i][1][0] * plane[0] + Triclinic::MatSym[i][1][1] * plane[1] + Triclinic::MatSym[i][1][2] * plane[2];

      double slipDirection[3] = {0};
      slipDirection[0] = Triclinic::MatSym[i][0][0] * direction[0] + Triclinic::MatSym[i][0][1] * direction[1] + Triclinic::MatSym[i][0][2] * direction[2];
      slipDirection[1] = Triclinic::MatSym[i][1][0] * direction[0] + Triclinic::MatSym[i][1][1] * direction[1] + Triclinic::MatSym[i][1][2] * direction[2];
      slipDirection[2] = Triclinic::MatSym[i][2][0] * direction[0] + Triclinic::MatSym[i][2][1] * direction[1] + Triclinic::MatSym[i][2][2] * direction[2];

      double cosPhi = fabs(load[0] * slipPlane[0] + load[1] * slipPlane[1] + load[2] * slipPlane[2]) / planeMag;
      double cosLambda = fabs(load[0] * slipDirection[0] + load[1] * slipDirection[1] + load[2] * slipDirection[2]) / directionMag;

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
//
// -----------------------------------------------------------------------------

namespace TriclinicHigh
{
class GenerateSphereCoordsImpl
{
  EbsdLib::FloatArrayType* m_Eulers;
  EbsdLib::FloatArrayType* m_xyz001;
  EbsdLib::FloatArrayType* m_xyz011;
  EbsdLib::FloatArrayType* m_xyz111;

public:
  GenerateSphereCoordsImpl(EbsdLib::FloatArrayType* eulerAngles, EbsdLib::FloatArrayType* xyz001Coords, EbsdLib::FloatArrayType* xyz011Coords, EbsdLib::FloatArrayType* xyz111Coords)
  : m_Eulers(eulerAngles)
  , m_xyz001(xyz001Coords)
  , m_xyz011(xyz011Coords)
  , m_xyz111(xyz111Coords)
  {
  }
  virtual ~GenerateSphereCoordsImpl() = default;

  void generate(size_t start, size_t end) const
  {
    EbsdLib::Matrix3X3D gTranspose;
    EbsdLib::Matrix3X1D direction(0.0, 0.0, 0.0);

    // Geneate all the Coordinates
    for(size_t i = start; i < end; ++i)
    {
      OrientationType eu(m_Eulers->getValue(i * 3), m_Eulers->getValue(i * 3 + 1), m_Eulers->getValue(i * 3 + 2));
      EbsdLib::Matrix3X3D g(OrientationTransformation::eu2om<OrientationType, OrientationType>(eu).data());

      gTranspose = g.transpose();

      // -----------------------------------------------------------------------------
      // 001 Family
      direction[0] = 0.0;
      direction[1] = 0.0;
      direction[2] = 1.0;
      (gTranspose * direction).copyInto<float>(m_xyz001->getPointer(i * 6));
      std::transform(m_xyz001->getPointer(i * 6), m_xyz001->getPointer(i * 6 + 3),
                     m_xyz001->getPointer(i * 6 + 3),             // write to the next triplet in memory
                     [](float value) { return value *= -1.0F; }); // Multiply each value by -1.0

      // -----------------------------------------------------------------------------
      // 011 Family
      direction[0] = 1.0;
      direction[1] = 0.0;
      direction[2] = 0.0;
      (gTranspose * direction).copyInto<float>(m_xyz011->getPointer(i * 6));
      std::transform(m_xyz011->getPointer(i * 6), m_xyz011->getPointer(i * 6 + 3),
                     m_xyz011->getPointer(i * 6 + 3),             // write to the next triplet in memory
                     [](float value) { return value *= -1.0F; }); // Multiply each value by -1.0

      // -----------------------------------------------------------------------------
      // 111 Family
      direction[0] = 0.0;
      direction[1] = 1.0;
      direction[2] = 0;
      (gTranspose * direction).copyInto<float>(m_xyz111->getPointer(i * 6));
      std::transform(m_xyz111->getPointer(i * 6), m_xyz111->getPointer(i * 6 + 3),
                     m_xyz111->getPointer(i * 6 + 3),             // write to the next triplet in memory
                     [](float value) { return value *= -1.0F; }); // Multiply each value by -1.0
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
//
// -----------------------------------------------------------------------------
void TriclinicOps::generateSphereCoordsFromEulers(EbsdLib::FloatArrayType* eulers, EbsdLib::FloatArrayType* xyz001, EbsdLib::FloatArrayType* xyz011, EbsdLib::FloatArrayType* xyz111) const
{
  size_t nOrientations = eulers->getNumberOfTuples();

  // Sanity Check the size of the arrays
  if(xyz001->getNumberOfTuples() < nOrientations * Triclinic::symSize0)
  {
    xyz001->resizeTuples(nOrientations * Triclinic::symSize0 * 3);
  }
  if(xyz011->getNumberOfTuples() < nOrientations * Triclinic::symSize1)
  {
    xyz011->resizeTuples(nOrientations * Triclinic::symSize1 * 3);
  }
  if(xyz111->getNumberOfTuples() < nOrientations * Triclinic::symSize2)
  {
    xyz111->resizeTuples(nOrientations * Triclinic::symSize2 * 3);
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
  return {Triclinic::k_EtaMin * EbsdLib::Constants::k_DegToRadD, Triclinic::k_EtaMax * EbsdLib::Constants::k_DegToRadD, Triclinic::k_ChiMax * EbsdLib::Constants::k_DegToRadD};
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool TriclinicOps::inUnitTriangle(double eta, double chi) const
{
  return !(eta < (Triclinic::k_EtaMin * EbsdLib::Constants::k_PiOver180D) || eta > (Triclinic::k_EtaMax * EbsdLib::Constants::k_PiOver180D) || chi < 0 ||
           chi > (Triclinic::k_ChiMax * EbsdLib::Constants::k_PiOver180D));
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdLib::Rgb TriclinicOps::generateIPFColor(double* eulers, double* refDir, bool degToRad) const
{
  return computeIPFColor(eulers, refDir, degToRad);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdLib::Rgb TriclinicOps::generateIPFColor(double phi1, double phi, double phi2, double refDir0, double refDir1, double refDir2, bool degToRad) const
{
  double eulers[3] = {phi1, phi, phi2};
  double refDir[3] = {refDir0, refDir1, refDir2};
  return computeIPFColor(eulers, refDir, degToRad);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdLib::Rgb TriclinicOps::generateRodriguesColor(double r1, double r2, double r3) const
{
  double range1 = 2.0f * Triclinic::OdfDimInitValue[0];
  double range2 = 2.0f * Triclinic::OdfDimInitValue[1];
  double range3 = 2.0f * Triclinic::OdfDimInitValue[2];
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

  return EbsdLib::RgbColor::dRgb(static_cast<int32_t>(red * 255), static_cast<int32_t>(green * 255), static_cast<int32_t>(blue * 255), 255);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::array<std::string, 3> TriclinicOps::getDefaultPoleFigureNames() const
{
  return {"<001>", "<100>", "<010>"};
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::vector<EbsdLib::UInt8ArrayType::Pointer> TriclinicOps::generatePoleFigure(PoleFigureConfiguration_t& config) const
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
  EbsdLib::FloatArrayType::Pointer xyz001 = EbsdLib::FloatArrayType::CreateArray(numOrientations * Triclinic::symSize0, dims, label0 + std::string("xyzCoords"), true);
  // this is size for CUBIC ONLY, <011> Family
  EbsdLib::FloatArrayType::Pointer xyz011 = EbsdLib::FloatArrayType::CreateArray(numOrientations * Triclinic::symSize1, dims, label1 + std::string("xyzCoords"), true);
  // this is size for CUBIC ONLY, <111> Family
  EbsdLib::FloatArrayType::Pointer xyz111 = EbsdLib::FloatArrayType::CreateArray(numOrientations * Triclinic::symSize2, dims, label2 + std::string("xyzCoords"), true);

  config.sphereRadius = 1.0f;

  // Generate the coords on the sphere **** Parallelized
  generateSphereCoordsFromEulers(config.eulers, xyz001.get(), xyz011.get(), xyz111.get());

  // These arrays hold the "intensity" images which eventually get converted to an actual Color RGB image
  // Generate the modified Lambert projection images (Squares, 2 of them, 1 for northern hemisphere, 1 for southern hemisphere
  EbsdLib::DoubleArrayType::Pointer intensity001 = EbsdLib::DoubleArrayType::CreateArray(config.imageDim * config.imageDim, label0 + "_Intensity_Image", true);
  EbsdLib::DoubleArrayType::Pointer intensity011 = EbsdLib::DoubleArrayType::CreateArray(config.imageDim * config.imageDim, label1 + "_Intensity_Image", true);
  EbsdLib::DoubleArrayType::Pointer intensity111 = EbsdLib::DoubleArrayType::CreateArray(config.imageDim * config.imageDim, label2 + "_Intensity_Image", true);
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
  EbsdLib::UInt8ArrayType::Pointer image001 = EbsdLib::UInt8ArrayType::CreateArray(config.imageDim * config.imageDim, dims, label0, true);
  EbsdLib::UInt8ArrayType::Pointer image011 = EbsdLib::UInt8ArrayType::CreateArray(config.imageDim * config.imageDim, dims, label1, true);
  EbsdLib::UInt8ArrayType::Pointer image111 = EbsdLib::UInt8ArrayType::CreateArray(config.imageDim * config.imageDim, dims, label2, true);
  std::vector<EbsdLib::UInt8ArrayType::Pointer> poleFigures(3);
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

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdLib::UInt8ArrayType::Pointer TriclinicOps::generateIPFTriangleLegend(int imageDim, bool generateEntirePlane) const
{

  std::vector<size_t> dims(1, 4);
  std::string arrayName = EbsdStringUtils::replace(getSymmetryName(), "/", "_");
  EbsdLib::UInt8ArrayType::Pointer image = EbsdLib::UInt8ArrayType::CreateArray(imageDim * imageDim, dims, arrayName + " Triangle Legend", true);
  uint32_t* pixelPtr = reinterpret_cast<uint32_t*>(image->getPointer(0));

  double xInc = 1.0f / static_cast<double>(imageDim);
  double yInc = 1.0f / static_cast<double>(imageDim);
  double rad = 1.0f;

  double x = 0.0f;
  double y = 0.0f;
  double a = 0.0f;
  double b = 0.0f;
  double c = 0.0f;

  double val = 0.0f;
  double x1 = 0.0f;
  double y1 = 0.0f;
  double z1 = 0.0f;
  double denom = 0.0f;

  EbsdLib::Rgb color;
  size_t idx = 0;
  size_t yScanLineIndex = 0; // We use this to control where the data is drawn. Otherwise, the image will come out flipped vertically
  // Loop over every pixel in the image and project up to the sphere to get the angle and then figure out the RGB from
  // there.
  for(int32_t yIndex = 0; yIndex < imageDim; ++yIndex)
  {

    for(int32_t xIndex = 0; xIndex < imageDim; ++xIndex)
    {
      idx = (imageDim * yScanLineIndex) + xIndex;

      x = -1.0f + 2.0f * xIndex * xInc;
      y = -1.0f + 2.0f * yIndex * yInc;

      double sumSquares = (x * x) + (y * y);
      if(sumSquares > 1.0) // Outside unit circle
      {
        color = 0xFFFFFFFF;
      }
      else if(sumSquares > (rad - 2 * xInc) && sumSquares < (rad + 2 * xInc)) // Black Borderline
      {
        color = 0xFF000000;
      }
      else
      {
        a = (x * x + y * y + 1);
        b = (2 * x * x + 2 * y * y);
        c = (x * x + y * y - 1);

        val = (-b + std::sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
        x1 = (1 + val) * x;
        y1 = (1 + val) * y;
        z1 = val;
        denom = (x1 * x1) + (y1 * y1) + (z1 * z1);
        denom = std::sqrt(denom);
        x1 = x1 / denom;
        y1 = y1 / denom;
        z1 = z1 / denom;

        color = generateIPFColor(0.0, 0.0, 0.0, x1, y1, z1, false);
      }

      pixelPtr[idx] = color;
    }
    yScanLineIndex++;
  }
  return image;
}

// -----------------------------------------------------------------------------
TriclinicOps::Pointer TriclinicOps::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
std::string TriclinicOps::getNameOfClass() const
{
  return std::string("TriclinicOps");
}

// -----------------------------------------------------------------------------
std::string TriclinicOps::ClassName()
{
  return std::string("TriclinicOps");
}

// -----------------------------------------------------------------------------
TriclinicOps::Pointer TriclinicOps::New()
{
  Pointer sharedPtr(new(TriclinicOps));
  return sharedPtr;
}
