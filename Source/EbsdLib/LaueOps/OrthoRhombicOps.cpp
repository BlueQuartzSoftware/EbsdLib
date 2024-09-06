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
#include "OrthoRhombicOps.h"

// Include this FIRST because there is a needed define for some compiles
// to expose some of the constants needed below
#include "EbsdLib/Core/EbsdMacros.h"
#include "EbsdLib/Core/Orientation.hpp"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Utilities/CanvasUtilities.hpp"
#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/ComputeStereographicProjection.h"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"
#include "EbsdLib/Utilities/PoleFigureUtilities.h"

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif

#define EBSD_LIB_GENERATE_ENTIRE_CIRCLE

namespace OrthoRhombic
{
static const std::array<size_t, 3> OdfNumBins = {36, 36, 36}; // Represents a 5Deg bin

static const std::array<double, 3> OdfDimInitValue = {std::pow((0.75 * ((EbsdLib::Constants::k_PiOver2D)-std::sin((EbsdLib::Constants::k_PiOver2D)))), (1.0 / 3.0)),
                                                      std::pow((0.75 * ((EbsdLib::Constants::k_PiOver2D)-std::sin((EbsdLib::Constants::k_PiOver2D)))), (1.0 / 3.0)),
                                                      std::pow((0.75 * ((EbsdLib::Constants::k_PiOver2D)-std::sin((EbsdLib::Constants::k_PiOver2D)))), (1.0 / 3.0))};
static const std::array<double, 3> OdfDimStepValue = {OdfDimInitValue[0] / static_cast<double>(OdfNumBins[0] / 2), OdfDimInitValue[1] / static_cast<double>(OdfNumBins[1] / 2),
                                                      OdfDimInitValue[2] / static_cast<double>(OdfNumBins[2] / 2)};

static const int symSize0 = 2;
static const int symSize1 = 2;
static const int symSize2 = 2;

static const int k_OdfSize = 46656;
static const int k_MdfSize = 46656;
static const int k_SymOpsCount = 4;
static const int k_NumMdfBins = 36;
// Rotation Point Group: 222
// clang-format off
static const std::vector<QuatD> QuatSym ={
    QuatD(0.0, 0.0, 0.0, 1.0),
    QuatD(1.0, 0.0, 0.0, 0.0),
    QuatD(0.0, 1.0, 0.0, 0.0),
    QuatD(0.0, 0.0, 1.0, 0.0),
};

static const std::vector<OrientationD> RodSym = {
    {0.0, 0.0, 1.0, 0.0},
    {1.0, 0.0, 0.0, 10000000000000.0},
    {0.0, 1.0, 0.0, 10000000000000.0},
    {0.0, 0.0, 1.0, 10000000000000.0},
};

static const double MatSym[k_SymOpsCount][3][3] = {
    {{1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}},
    
    {{1.0, 0.0, 0.0},
    {0.0, -1.0, 0.0},
    {0.0, 0.0, -1.0}},
    
    {{-1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, -1.0}},
    
    {{-1.0, 0.0, 0.0},
    {0.0, -1.0, 0.0},
    {0.0, 0.0, 1.0}},
    
};
// clang-format on
static const double k_EtaMin = 0.0;
static const double k_EtaMax = 90.0;
static const double k_ChiMax = 90.0;
} // namespace OrthoRhombic

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
OrthoRhombicOps::OrthoRhombicOps() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
OrthoRhombicOps::~OrthoRhombicOps() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool OrthoRhombicOps::getHasInversion() const
{
  return true;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int OrthoRhombicOps::getODFSize() const
{
  return OrthoRhombic::k_OdfSize;
}

// -----------------------------------------------------------------------------
std::array<int32_t, 3> OrthoRhombicOps::getNumSymmetry() const
{
  return {OrthoRhombic::symSize0, OrthoRhombic::symSize1, OrthoRhombic::symSize2};
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int OrthoRhombicOps::getMDFSize() const
{
  return OrthoRhombic::k_MdfSize;
}

// -----------------------------------------------------------------------------
int OrthoRhombicOps::getMdfPlotBins() const
{
  return OrthoRhombic::k_NumMdfBins;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int OrthoRhombicOps::getNumSymOps() const
{
  return OrthoRhombic::k_SymOpsCount;
}

// -----------------------------------------------------------------------------
std::array<size_t, 3> OrthoRhombicOps::getOdfNumBins() const
{
  return OrthoRhombic::OdfNumBins;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string OrthoRhombicOps::getSymmetryName() const
{
  return "Orthorhombic mmm (D2h)";
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string OrthoRhombicOps::getRotationPointGroup() const
{
  return "222";
}

OrientationD OrthoRhombicOps::calculateMisorientation(const QuatD& q1, const QuatD& q2) const
{
  return calculateMisorientationInternal(OrthoRhombic::QuatSym, q1, q2);
}

// -----------------------------------------------------------------------------
OrientationF OrthoRhombicOps::calculateMisorientation(const QuatF& q1f, const QuatF& q2f) const

{
  QuatD q1 = q1f.to<double>();
  QuatD q2 = q2f.to<double>();
  OrientationD axisAngle = calculateMisorientationInternal(OrthoRhombic::QuatSym, q1, q2);
  return axisAngle;
}

QuatD OrthoRhombicOps::getQuatSymOp(int32_t i) const
{
  return OrthoRhombic::QuatSym[i];
}

void OrthoRhombicOps::getRodSymOp(int i, double* r) const
{
  r[0] = OrthoRhombic::RodSym[i][0];
  r[1] = OrthoRhombic::RodSym[i][1];
  r[2] = OrthoRhombic::RodSym[i][2];
}

EbsdLib::Matrix3X3D OrthoRhombicOps::getMatSymOpD(int i) const
{
  return {OrthoRhombic::MatSym[i][0][0], OrthoRhombic::MatSym[i][0][1], OrthoRhombic::MatSym[i][0][2], OrthoRhombic::MatSym[i][1][0], OrthoRhombic::MatSym[i][1][1],
          OrthoRhombic::MatSym[i][1][2], OrthoRhombic::MatSym[i][2][0], OrthoRhombic::MatSym[i][2][1], OrthoRhombic::MatSym[i][2][2]};
}

EbsdLib::Matrix3X3F OrthoRhombicOps::getMatSymOpF(int i) const
{
  return {static_cast<float>(OrthoRhombic::MatSym[i][0][0]), static_cast<float>(OrthoRhombic::MatSym[i][0][1]), static_cast<float>(OrthoRhombic::MatSym[i][0][2]),
          static_cast<float>(OrthoRhombic::MatSym[i][1][0]), static_cast<float>(OrthoRhombic::MatSym[i][1][1]), static_cast<float>(OrthoRhombic::MatSym[i][1][2]),
          static_cast<float>(OrthoRhombic::MatSym[i][2][0]), static_cast<float>(OrthoRhombic::MatSym[i][2][1]), static_cast<float>(OrthoRhombic::MatSym[i][2][2])};
}

void OrthoRhombicOps::getMatSymOp(int i, double g[3][3]) const
{
  g[0][0] = OrthoRhombic::MatSym[i][0][0];
  g[0][1] = OrthoRhombic::MatSym[i][0][1];
  g[0][2] = OrthoRhombic::MatSym[i][0][2];
  g[1][0] = OrthoRhombic::MatSym[i][1][0];
  g[1][1] = OrthoRhombic::MatSym[i][1][1];
  g[1][2] = OrthoRhombic::MatSym[i][1][2];
  g[2][0] = OrthoRhombic::MatSym[i][2][0];
  g[2][1] = OrthoRhombic::MatSym[i][2][1];
  g[2][2] = OrthoRhombic::MatSym[i][2][2];
}

void OrthoRhombicOps::getMatSymOp(int i, float g[3][3]) const
{
  g[0][0] = static_cast<float>(OrthoRhombic::MatSym[i][0][0]);
  g[0][1] = static_cast<float>(OrthoRhombic::MatSym[i][0][1]);
  g[0][2] = static_cast<float>(OrthoRhombic::MatSym[i][0][2]);
  g[1][0] = static_cast<float>(OrthoRhombic::MatSym[i][1][0]);
  g[1][1] = static_cast<float>(OrthoRhombic::MatSym[i][1][1]);
  g[1][2] = static_cast<float>(OrthoRhombic::MatSym[i][1][2]);
  g[2][0] = static_cast<float>(OrthoRhombic::MatSym[i][2][0]);
  g[2][1] = static_cast<float>(OrthoRhombic::MatSym[i][2][1]);
  g[2][2] = static_cast<float>(OrthoRhombic::MatSym[i][2][2]);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
OrientationType OrthoRhombicOps::getODFFZRod(const OrientationType& rod) const
{
  return _calcRodNearestOrigin(OrthoRhombic::RodSym, rod);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
OrientationType OrthoRhombicOps::getMDFFZRod(const OrientationType& inRod) const
{
  throw EbsdLib::method_not_implemented("OrthoRhombicOps::getMDFFZRod not implemented");

  double FZn1 = 0.0f, FZn2 = 0.0f, FZn3 = 0.0f, FZw = 0.0f;

  OrientationType rod = _calcRodNearestOrigin(OrthoRhombic::RodSym, inRod);
  OrientationType ax = OrientationTransformation::ro2ax<OrientationType, OrientationType>(rod);
  //  double n1 = ax[0];
  //  double n2 = ax[1];
  //  double n3 = ax[2];
  //  double w = ax[3];

  /// FIXME: Are we missing code for OrthoRhombic MDF FZ Rodrigues calculation?

  return OrientationTransformation::ax2ro<OrientationType, OrientationType>(OrientationType(FZn1, FZn2, FZn3, FZw));
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QuatD OrthoRhombicOps::getNearestQuat(const QuatD& q1, const QuatD& q2) const
{
  return _calcNearestQuat(OrthoRhombic::QuatSym, q1, q2);
}

QuatF OrthoRhombicOps::getNearestQuat(const QuatF& q1f, const QuatF& q2f) const
{
  return _calcNearestQuat(OrthoRhombic::QuatSym, q1f.to<double>(), q2f.to<double>()).to<float>();
}

QuatD OrthoRhombicOps::getFZQuat(const QuatD& qr) const
{
  return _calcQuatNearestOrigin(OrthoRhombic::QuatSym, qr);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int OrthoRhombicOps::getMisoBin(const OrientationType& rod) const
{
  double dim[3];
  double bins[3];
  double step[3];
  OrientationType ho = OrientationTransformation::ro2ho<OrientationType, OrientationType>(rod);

  dim[0] = OrthoRhombic::OdfDimInitValue[0];
  dim[1] = OrthoRhombic::OdfDimInitValue[1];
  dim[2] = OrthoRhombic::OdfDimInitValue[2];
  step[0] = OrthoRhombic::OdfDimStepValue[0];
  step[1] = OrthoRhombic::OdfDimStepValue[1];
  step[2] = OrthoRhombic::OdfDimStepValue[2];
  bins[0] = static_cast<double>(OrthoRhombic::OdfNumBins[0]);
  bins[1] = static_cast<double>(OrthoRhombic::OdfNumBins[1]);
  bins[2] = static_cast<double>(OrthoRhombic::OdfNumBins[2]);

  return _calcMisoBin(dim, bins, step, ho);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
OrientationType OrthoRhombicOps::determineEulerAngles(double random[3], int choose) const
{
  double init[3];
  double step[3];
  int32_t phi[3];
  double h1, h2, h3;

  init[0] = OrthoRhombic::OdfDimInitValue[0];
  init[1] = OrthoRhombic::OdfDimInitValue[1];
  init[2] = OrthoRhombic::OdfDimInitValue[2];
  step[0] = OrthoRhombic::OdfDimStepValue[0];
  step[1] = OrthoRhombic::OdfDimStepValue[1];
  step[2] = OrthoRhombic::OdfDimStepValue[2];
  phi[0] = static_cast<int32_t>(choose % OrthoRhombic::OdfNumBins[0]);
  phi[1] = static_cast<int32_t>((choose / OrthoRhombic::OdfNumBins[0]) % OrthoRhombic::OdfNumBins[1]);
  phi[2] = static_cast<int32_t>(choose / (OrthoRhombic::OdfNumBins[0] * OrthoRhombic::OdfNumBins[1]));

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
OrientationType OrthoRhombicOps::randomizeEulerAngles(const OrientationType& synea) const
{
  size_t symOp = getRandomSymmetryOperatorIndex(OrthoRhombic::k_SymOpsCount);
  QuatD quat = OrientationTransformation::eu2qu<OrientationType, QuatD>(synea);
  QuatD qc = OrthoRhombic::QuatSym[symOp] * quat;
  return OrientationTransformation::qu2eu<QuatD, OrientationType>(qc);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
OrientationType OrthoRhombicOps::determineRodriguesVector(double random[3], int choose) const
{
  double init[3];
  double step[3];
  int32_t phi[3];
  double h1, h2, h3;

  init[0] = OrthoRhombic::OdfDimInitValue[0];
  init[1] = OrthoRhombic::OdfDimInitValue[1];
  init[2] = OrthoRhombic::OdfDimInitValue[2];
  step[0] = OrthoRhombic::OdfDimStepValue[0];
  step[1] = OrthoRhombic::OdfDimStepValue[1];
  step[2] = OrthoRhombic::OdfDimStepValue[2];
  phi[0] = static_cast<int32_t>(choose % OrthoRhombic::OdfNumBins[0]);
  phi[1] = static_cast<int32_t>((choose / OrthoRhombic::OdfNumBins[0]) % OrthoRhombic::OdfNumBins[1]);
  phi[2] = static_cast<int32_t>(choose / (OrthoRhombic::OdfNumBins[0] * OrthoRhombic::OdfNumBins[1]));

  _calcDetermineHomochoricValues(random, init, step, phi, h1, h2, h3);
  OrientationType ho(h1, h2, h3);
  OrientationType ro = OrientationTransformation::ho2ro<OrientationType, OrientationType>(ho);
  ro = getMDFFZRod(ro);
  return ro;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int OrthoRhombicOps::getOdfBin(const OrientationType& rod) const
{
  double dim[3];
  double bins[3];
  double step[3];

  OrientationType ho = OrientationTransformation::ro2ho<OrientationType, OrientationType>(rod);

  dim[0] = OrthoRhombic::OdfDimInitValue[0];
  dim[1] = OrthoRhombic::OdfDimInitValue[1];
  dim[2] = OrthoRhombic::OdfDimInitValue[2];
  step[0] = OrthoRhombic::OdfDimStepValue[0];
  step[1] = OrthoRhombic::OdfDimStepValue[1];
  step[2] = OrthoRhombic::OdfDimStepValue[2];
  bins[0] = static_cast<double>(OrthoRhombic::OdfNumBins[0]);
  bins[1] = static_cast<double>(OrthoRhombic::OdfNumBins[1]);
  bins[2] = static_cast<double>(OrthoRhombic::OdfNumBins[2]);

  return _calcODFBin(dim, bins, step, ho);
}

void OrthoRhombicOps::getSchmidFactorAndSS(double load[3], double& schmidfactor, double angleComps[2], int& slipsys) const
{
  schmidfactor = 0;
  slipsys = 0;
}

void OrthoRhombicOps::getSchmidFactorAndSS(double load[3], double plane[3], double direction[3], double& schmidfactor, double angleComps[2], int& slipsys) const
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
  for(int i = 0; i < OrthoRhombic::k_SymOpsCount; i++)
  {
    // compute slip system
    double slipPlane[3] = {0};
    slipPlane[2] = OrthoRhombic::MatSym[i][2][0] * plane[0] + OrthoRhombic::MatSym[i][2][1] * plane[1] + OrthoRhombic::MatSym[i][2][2] * plane[2];

    // dont consider negative z planes (to avoid duplicates)
    if(slipPlane[2] >= 0)
    {
      slipPlane[0] = OrthoRhombic::MatSym[i][0][0] * plane[0] + OrthoRhombic::MatSym[i][0][1] * plane[1] + OrthoRhombic::MatSym[i][0][2] * plane[2];
      slipPlane[1] = OrthoRhombic::MatSym[i][1][0] * plane[0] + OrthoRhombic::MatSym[i][1][1] * plane[1] + OrthoRhombic::MatSym[i][1][2] * plane[2];

      double slipDirection[3] = {0};
      slipDirection[0] = OrthoRhombic::MatSym[i][0][0] * direction[0] + OrthoRhombic::MatSym[i][0][1] * direction[1] + OrthoRhombic::MatSym[i][0][2] * direction[2];
      slipDirection[1] = OrthoRhombic::MatSym[i][1][0] * direction[0] + OrthoRhombic::MatSym[i][1][1] * direction[1] + OrthoRhombic::MatSym[i][1][2] * direction[2];
      slipDirection[2] = OrthoRhombic::MatSym[i][2][0] * direction[0] + OrthoRhombic::MatSym[i][2][1] * direction[1] + OrthoRhombic::MatSym[i][2][2] * direction[2];

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

double OrthoRhombicOps::getmPrime(const QuatD& q1, const QuatD& q2, double LD[3]) const
{
  return 0.0;
}

double OrthoRhombicOps::getF1(const QuatD& q1, const QuatD& q2, double LD[3], bool maxS) const
{
  return 0.0;
}

double OrthoRhombicOps::getF1spt(const QuatD& q1, const QuatD& q2, double LD[3], bool maxS) const
{
  return 0.0;
}

double OrthoRhombicOps::getF7(const QuatD& q1, const QuatD& q2, double LD[3], bool maxS) const
{
  return 0.0;
}
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

namespace OrthoRhombic
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
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0

      // -----------------------------------------------------------------------------
      // 011 Family
      direction[0] = 1.0;
      direction[1] = 0.0;
      direction[2] = 0.0;
      (gTranspose * direction).copyInto<float>(m_xyz011->getPointer(i * 6));
      std::transform(m_xyz011->getPointer(i * 6), m_xyz011->getPointer(i * 6 + 3),
                     m_xyz011->getPointer(i * 6 + 3),             // write to the next triplet in memory
                     [](float value) { return value * -1.0F; }); // Multiply each value by -1.0

      // -----------------------------------------------------------------------------
      // 111 Family
      direction[0] = 0.0;
      direction[1] = 1.0;
      direction[2] = 0.0;
      (gTranspose * direction).copyInto<float>(m_xyz111->getPointer(i * 6));
      std::transform(m_xyz111->getPointer(i * 6), m_xyz111->getPointer(i * 6 + 3),
                     m_xyz111->getPointer(i * 6 + 3),             // write to the next triplet in memory
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
} // namespace OrthoRhombic

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void OrthoRhombicOps::generateSphereCoordsFromEulers(EbsdLib::FloatArrayType* eulers, EbsdLib::FloatArrayType* xyz001, EbsdLib::FloatArrayType* xyz011, EbsdLib::FloatArrayType* xyz111) const
{
  size_t nOrientations = eulers->getNumberOfTuples();

  // Sanity Check the size of the arrays
  if(xyz001->getNumberOfTuples() < nOrientations * OrthoRhombic::symSize0)
  {
    xyz001->resizeTuples(nOrientations * OrthoRhombic::symSize0 * 3);
  }
  if(xyz011->getNumberOfTuples() < nOrientations * OrthoRhombic::symSize1)
  {
    xyz011->resizeTuples(nOrientations * OrthoRhombic::symSize1 * 3);
  }
  if(xyz111->getNumberOfTuples() < nOrientations * OrthoRhombic::symSize2)
  {
    xyz111->resizeTuples(nOrientations * OrthoRhombic::symSize2 * 3);
  }

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  tbb::parallel_for(tbb::blocked_range<size_t>(0, nOrientations), OrthoRhombic::GenerateSphereCoordsImpl(eulers, xyz001, xyz011, xyz111), tbb::auto_partitioner());
#else
  OrthoRhombic::GenerateSphereCoordsImpl serial(eulers, xyz001, xyz011, xyz111);
  serial.generate(0, nOrientations);
#endif
}

// -----------------------------------------------------------------------------
std::array<double, 3> OrthoRhombicOps::getIpfColorAngleLimits(double eta) const
{
  return {OrthoRhombic::k_EtaMin * EbsdLib::Constants::k_DegToRadD, OrthoRhombic::k_EtaMax * EbsdLib::Constants::k_DegToRadD, OrthoRhombic::k_ChiMax * EbsdLib::Constants::k_DegToRadD};
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool OrthoRhombicOps::inUnitTriangle(double eta, double chi) const
{
  return !(eta < (OrthoRhombic::k_EtaMin * EbsdLib::Constants::k_PiOver180D) || eta > (OrthoRhombic::k_EtaMax * EbsdLib::Constants::k_PiOver180D) || chi < 0 ||
           chi > (OrthoRhombic::k_ChiMax * EbsdLib::Constants::k_PiOver180D));
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdLib::Rgb OrthoRhombicOps::generateIPFColor(double* eulers, double* refDir, bool degToRad) const
{
  return computeIPFColor(eulers, refDir, degToRad);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdLib::Rgb OrthoRhombicOps::generateIPFColor(double phi1, double phi, double phi2, double refDir0, double refDir1, double refDir2, bool degToRad) const
{
  double eulers[3] = {phi1, phi, phi2};
  double refDir[3] = {refDir0, refDir1, refDir2};
  return computeIPFColor(eulers, refDir, degToRad);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdLib::Rgb OrthoRhombicOps::generateRodriguesColor(double r1, double r2, double r3) const
{
  double range1 = 2.0f * OrthoRhombic::OdfDimInitValue[0];
  double range2 = 2.0f * OrthoRhombic::OdfDimInitValue[1];
  double range3 = 2.0f * OrthoRhombic::OdfDimInitValue[2];
  double max1 = range1 / 2.0;
  double max2 = range2 / 2.0;
  double max3 = range3 / 2.0;
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
std::array<std::string, 3> OrthoRhombicOps::getDefaultPoleFigureNames() const
{
  return {"<001>", "<100>", "<010>"};
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::vector<EbsdLib::UInt8ArrayType::Pointer> OrthoRhombicOps::generatePoleFigure(PoleFigureConfiguration_t& config) const
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
  std::vector<size_t> dims(1, 3);
  std::vector<EbsdLib::FloatArrayType::Pointer> coords(3);
  coords[0] = EbsdLib::FloatArrayType::CreateArray(numOrientations * OrthoRhombic::symSize0, dims, label0 + std::string("001_Coords"), true);
  coords[1] = EbsdLib::FloatArrayType::CreateArray(numOrientations * OrthoRhombic::symSize1, dims, label1 + std::string("100_Coords"), true);
  coords[2] = EbsdLib::FloatArrayType::CreateArray(numOrientations * OrthoRhombic::symSize2, dims, label2 + std::string("010_Coords"), true);

  config.sphereRadius = 1.0;

  // Generate the coords on the sphere **** Parallelized
  generateSphereCoordsFromEulers(config.eulers, coords[0].get(), coords[1].get(), coords[2].get());

  // These arrays hold the "intensity" images which eventually get converted to an actual Color RGB image
  // Generate the modified Lambert projection images (Squares, 2 of them, 1 for northern hemisphere, 1 for southern hemisphere
  EbsdLib::DoubleArrayType::Pointer intensity001 = EbsdLib::DoubleArrayType::CreateArray(config.imageDim * config.imageDim, label0 + "_Intensity_Image", true);
  EbsdLib::DoubleArrayType::Pointer intensity100 = EbsdLib::DoubleArrayType::CreateArray(config.imageDim * config.imageDim, label1 + "_Intensity_Image", true);
  EbsdLib::DoubleArrayType::Pointer intensity010 = EbsdLib::DoubleArrayType::CreateArray(config.imageDim * config.imageDim, label2 + "_Intensity_Image", true);
#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  bool doParallel = true;

  if(doParallel)
  {
    std::shared_ptr<tbb::task_group> g(new tbb::task_group);
    g->run(ComputeStereographicProjection(coords[0].get(), &config, intensity001.get()));
    g->run(ComputeStereographicProjection(coords[1].get(), &config, intensity100.get()));
    g->run(ComputeStereographicProjection(coords[2].get(), &config, intensity010.get()));
    g->wait(); // Wait for all the threads to complete before moving on.
  }
  else
#endif
  {
    ComputeStereographicProjection m001(coords[0].get(), &config, intensity001.get());
    m001();
    ComputeStereographicProjection m011(coords[1].get(), &config, intensity100.get());
    m011();
    ComputeStereographicProjection m111(coords[2].get(), &config, intensity010.get());
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

  dPtr = intensity100->getPointer(0);
  count = intensity100->getNumberOfTuples();
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

  dPtr = intensity010->getPointer(0);
  count = intensity010->getNumberOfTuples();
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
  EbsdLib::UInt8ArrayType::Pointer image100 = EbsdLib::UInt8ArrayType::CreateArray(config.imageDim * config.imageDim, dims, label1, true);
  EbsdLib::UInt8ArrayType::Pointer image010 = EbsdLib::UInt8ArrayType::CreateArray(config.imageDim * config.imageDim, dims, label2, true);

  std::vector<EbsdLib::UInt8ArrayType::Pointer> poleFigures(3);
  if(config.order.size() == 3)
  {
    poleFigures[config.order[0]] = image001;
    poleFigures[config.order[1]] = image100;
    poleFigures[config.order[2]] = image010;
  }
  else
  {
    poleFigures[0] = image001;
    poleFigures[1] = image100;
    poleFigures[2] = image010;
  }

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  if(doParallel)
  {
    std::shared_ptr<tbb::task_group> g(new tbb::task_group);
    g->run(GeneratePoleFigureRgbaImageImpl(intensity001.get(), &config, image001.get()));
    g->run(GeneratePoleFigureRgbaImageImpl(intensity100.get(), &config, image100.get()));
    g->run(GeneratePoleFigureRgbaImageImpl(intensity010.get(), &config, image010.get()));
    g->wait(); // Wait for all the threads to complete before moving on.
  }
  else
#endif
  {
    GeneratePoleFigureRgbaImageImpl m001(intensity001.get(), &config, image001.get());
    m001();
    GeneratePoleFigureRgbaImageImpl m011(intensity100.get(), &config, image100.get());
    m011();
    GeneratePoleFigureRgbaImageImpl m111(intensity010.get(), &config, image010.get());
    m111();
  }

  return poleFigures;
}

namespace
{
// -----------------------------------------------------------------------------
EbsdLib::UInt8ArrayType::Pointer CreateIPFLegend(const OrthoRhombicOps* ops, int imageDim, bool generateEntirePlane)
{
  std::vector<size_t> dims(1, 4);
  std::string arrayName = EbsdStringUtils::replace(ops->getSymmetryName(), "/", "_");
  EbsdLib::UInt8ArrayType::Pointer image = EbsdLib::UInt8ArrayType::CreateArray(imageDim * imageDim, dims, arrayName + " Triangle Legend", true);
  uint32_t* pixelPtr = reinterpret_cast<uint32_t*>(image->getPointer(0));

  double xInc = 1.0f / static_cast<double>(imageDim);
  double yInc = 1.0f / static_cast<double>(imageDim);
  double rad = 1.0;

  double x = 0.0;
  double y = 0.0;
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;

  double val = 0.0;
  double x1 = 0.0;
  double y1 = 0.0;
  double z1 = 0.0;
  double denom = 0.0;

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

      if(generateEntirePlane)
      {
        x = -1.0f + 2.0f * xIndex * xInc;
        y = -1.0f + 2.0f * yIndex * yInc;
      }
      else
      {
        x = xIndex * xInc;
        y = yIndex * yInc;
      }
      double sumSquares = (x * x) + (y * y);
      if(sumSquares > 1.0) // Outside unit circle
      {
        color = 0xFFFFFFFF;
      }
      else if(sumSquares > (rad - 2 * xInc) && sumSquares < (rad + 2 * xInc)) // Black Borderline
      {
        color = 0xFF000000;
      }
      else if(!generateEntirePlane && (xIndex == 0 || yIndex == 0)) // Black Borderline
      {
        color = 0xFF000000;
      }
      else if(x < 0.0f || y < 0.0f)
      {
        color = 0xFF808080;
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

        color = ops->generateIPFColor(0.0, 0.0, 0.0, x1, y1, z1, false);
      }

      pixelPtr[idx] = color;
    }
    yScanLineIndex++;
  }
  return image;
}

// -----------------------------------------------------------------------------
void DrawFullCircleAnnotations(canvas_ity::canvas& context, int canvasDim, float fontPtSize, std::vector<float> margins, std::array<float, 2> figureOrigin, std::array<float, 2> figureCenter)
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

  std::vector<float> angles = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 225.0f, 270.0f, 315.0f};
  std::vector<std::string> labels2 = {"[100]", "[110]", "[010]", "[-110]", "[-100]", "[-1-10]", "[0-10]", "[1-10]"};

  float radius = 1.0; // Work with a Unit Circle.
  for(size_t idx = 0; idx < labels2.size(); idx++)
  {
    float angle = angles[idx];
    float rads = angle * M_PI / 180.0f;
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
    float penWidth = 2.0f;
    context.set_line_width(penWidth);
    EbsdLib::DrawLine(context, figureCenter[0], figureCenter[1], x, y);

    std::string label = labels2[idx];
    float fontWidth = context.measure_text(label.c_str());

    // Special Adjustments based on idx
    if(idx == 0)
    {
      // x = x - fontWidth / 2.0f;
    }
    if(idx == 1)
    {
      // x = x - fontWidth / 2.0f;
      y = y - fontPtSize * 0.1f;
    }
    if(idx == 2)
    {
      x = x - fontWidth / 2.0f;
      y = y - (fontPtSize * 0.1f);
    }
    if(idx == 3)
    {
      x = x - fontWidth;
      // y = y + fontPtSize;
    }
    if(idx == 4)
    {
      x = x - fontWidth / 2.0f;
      y = y + fontPtSize;
    }
    if(idx == 5)
    {
      x = x - (fontWidth * 0.6f);
      y = y + fontPtSize;
    }
    if(idx == 6)
    {
      x = x - (fontWidth * 0.5f);
      y = y + fontPtSize;
    }
    if(idx == 7)
    {
      //      x = x + (fontWidth * 0.2f);
      y = y + fontPtSize;
    }
    EbsdLib::WriteText(context, label, {x, y}, fontPtSize);
  }

  // Draw the [0001] in the center of the image
  {
    float x = figureCenter[0];
    float y = figureCenter[1] + fontPtSize;

    std::string label("[001]");
    EbsdLib::WriteText(context, label, {x, y}, fontPtSize);
  }
}

// -----------------------------------------------------------------------------
void DrawReducedAnnotations(canvas_ity::canvas& context, int canvasDim, float fontPtSize, std::vector<float> margins, std::array<float, 2> figureOrigin, std::array<float, 2> figureCenter)
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

  // Draw the [010]
  {

    std::string label("[010]"); // Blue
    float fontWidth = context.measure_text(label.c_str());
    float x = margins[3] - fontWidth * 0.5f;
    float y = margins[0] - (margins[0] * 0.10f);
    EbsdLib::WriteText(context, label, {x, y}, fontPtSize);
  }

  // Draw the [100]
  {
    std::string label("[100]"); // Green
    float fontWidth = context.measure_text(label.c_str());
    float x = margins[3] + legendWidth - (fontWidth * 0.5f);
    float y = pageHeight - fontPtSize;
    EbsdLib::WriteText(context, label, {x, y}, fontPtSize);
  }

  // Draw the [001]
  {
    std::string label("[001]");
    float fontWidth = context.measure_text(label.c_str());
    float x = margins[3] - fontWidth * 0.5f;
    float y = pageHeight - fontPtSize;

    EbsdLib::WriteText(context, label, {x, y}, fontPtSize);
  }
}

} // namespace
// -----------------------------------------------------------------------------
EbsdLib::UInt8ArrayType::Pointer OrthoRhombicOps::generateIPFTriangleLegend(int canvasDim, bool generateEntirePlane) const
{
  // Figure out the Legend Pixel Size
  const float fontPtSize = static_cast<float>(canvasDim) / 24.0f;
  const std::vector<float> margins = {fontPtSize * 3, static_cast<float>(canvasDim / 16.0f), fontPtSize * 2, static_cast<float>(canvasDim / 16.0f)};

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

  std::array<float, 2> figureOrigin = {margins[3], margins[0]};
  std::array<float, 2> figureCenter = {figureOrigin[0] + halfWidth, figureOrigin[1] + halfHeight};

  EbsdLib::UInt8ArrayType::Pointer image = CreateIPFLegend(this, legendHeight, generateEntirePlane);

  // Create a Canvas to draw into
  canvas_ity::canvas context(pageWidth, pageHeight);

  context.set_font(m_LatoBold.data(), static_cast<int>(m_LatoBold.size()), fontPtSize);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
  canvas_ity::baseline_style const baselines[] = {canvas_ity::alphabetic, canvas_ity::top, canvas_ity::middle, canvas_ity::bottom, canvas_ity::hanging, canvas_ity::ideographic};
  context.text_baseline = baselines[0];

  // Fill the whole background with white
  context.move_to(0.0f, 0.0f);
  context.line_to(static_cast<float>(pageWidth), 0.0f);
  context.line_to(static_cast<float>(pageWidth), static_cast<float>(pageHeight));
  context.line_to(0.0f, static_cast<float>(pageHeight));
  context.line_to(0.0f, 0.0f);
  context.close_path();
  context.set_color(canvas_ity::fill_style, 1.0f, 1.0f, 1.0f, 1.0f);
  context.fill();

  image = EbsdLib::MirrorImage(image.get(), legendHeight);
  image = EbsdLib::ConvertColorOrder(image.get(), legendHeight);

  context.draw_image(image->getPointer(0), legendWidth, legendHeight, legendWidth * image->getNumberOfComponents(), figureOrigin[0], figureOrigin[1], static_cast<float>(legendWidth),
                     static_cast<float>(legendHeight));

  //  context.set_line_width(5.0f);
  //  EbsdLib::DrawLine(context, 0.0f, margins[0], pageWidth, margins[0]);
  //  EbsdLib::DrawLine(context, 0.0f, pageHeight - margins[2], pageWidth, pageHeight - margins[2]);
  //  EbsdLib::DrawLine(context, margins[3], 0, margins[3], pageHeight);
  //  EbsdLib::DrawLine(context, pageWidth - margins[1], 0, pageWidth - margins[1], pageHeight);

  // Draw Title of Legend
  context.set_font(m_LatoBold.data(), static_cast<int>(m_LatoBold.size()), fontPtSize * 1.5);
  EbsdLib::WriteText(context, getSymmetryName(), {margins[0], static_cast<float>(fontPtSize * 1.5)}, fontPtSize * 1.5);

  if(generateEntirePlane)
  {
    context.set_font(m_LatoRegular.data(), static_cast<int>(m_LatoRegular.size()), fontPtSize);
    DrawFullCircleAnnotations(context, canvasDim, fontPtSize, margins, figureOrigin, figureCenter);
  }
  else
  {
    context.set_font(m_LatoRegular.data(), static_cast<int>(m_LatoRegular.size()), fontPtSize);
    DrawReducedAnnotations(context, canvasDim, fontPtSize, margins, figureOrigin, figureCenter);
  }

  // Fetch the rendered RGBA pixels from the entire canvas.
  EbsdLib::UInt8ArrayType::Pointer rgbaCanvasImage = EbsdLib::UInt8ArrayType::CreateArray(pageHeight * pageWidth, {4ULL}, "Triangle Legend", true);
  // std::vector<unsigned char> rgbaCanvasImage(static_cast<size_t>(pageHeight * pageWidth * 4));
  context.get_image_data(rgbaCanvasImage->getPointer(0), pageWidth, pageHeight, pageWidth * 4, 0, 0);

  return rgbaCanvasImage;
}

// -----------------------------------------------------------------------------
OrthoRhombicOps::Pointer OrthoRhombicOps::NullPointer()
{
  return Pointer(static_cast<Self*>(nullptr));
}

// -----------------------------------------------------------------------------
std::string OrthoRhombicOps::getNameOfClass() const
{
  return std::string("OrthorhombicOps");
}

// -----------------------------------------------------------------------------
std::string OrthoRhombicOps::ClassName()
{
  return std::string("OrthorhombicOps");
}

// -----------------------------------------------------------------------------
OrthoRhombicOps::Pointer OrthoRhombicOps::New()
{
  Pointer sharedPtr(new(OrthoRhombicOps));
  return sharedPtr;
}
