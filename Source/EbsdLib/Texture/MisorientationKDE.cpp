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

#include "MisorientationKDE.h"

#include "EbsdLib/Orientation/Homochoric.hpp"
#include "EbsdLib/Orientation/Rodrigues.hpp"
#include "EbsdLib/Texture/RandomAngleDistribution.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ebsdlib
{
MisorientationKDE::MisorientationKDE(LaueOps::Pointer ops, uint32_t crystalStructure, double halfwidthRadians)
: m_Ops(std::move(ops))
, m_CrystalStructure(crystalStructure)
, m_Kernel(halfwidthRadians)
, m_BinWeights(m_Ops->getMDFSize(), 0.0)
, m_BinQuatSum(m_Ops->getMDFSize(), std::array<double, 4>{0.0, 0.0, 0.0, 0.0})
{
  size_t numSymOps = m_Ops->getNumSymOps();
  m_SymQuats.reserve(numSymOps);
  for(size_t i = 0; i < numSymOps; i++)
  {
    m_SymQuats.push_back(m_Ops->getQuatSymOp(i));
  }
}

void MisorientationKDE::addMisorientation(const QuatD& misoQuat, double weight)
{
  RodriguesDType rod = m_Ops->getMDFFZRod(misoQuat.toRodrigues());
  int binIndex = m_Ops->getMisoBin(rod);
  const size_t bin = static_cast<size_t>(binIndex);
  m_BinWeights[bin] += weight;
  m_TotalWeight += weight;

  // Accumulate the weighted circular mean of the FZ misorientation quaternion within its bin.
  // All misorientations sharing a bin lie within one ~5-degree cell, so a sign-aligned linear
  // sum (aligned to the bin's running accumulator, or to itself when the bin is first seen) and
  // a final renormalization is an accurate mean on that small patch of SO(3). Using this mean as
  // the KDE center (in finalize) instead of the geometric bin center removes the bin-quantization
  // bias without storing the individual misorientations (memory stays bounded by getMDFSize).
  const QuatD fzQuat = rod.toQuaternion();
  std::array<double, 4>& acc = m_BinQuatSum[bin];
  const double alignDot = fzQuat.x() * acc[0] + fzQuat.y() * acc[1] + fzQuat.z() * acc[2] + fzQuat.w() * acc[3];
  const double sign = (alignDot < 0.0) ? -1.0 : 1.0;
  acc[0] += weight * sign * fzQuat.x();
  acc[1] += weight * sign * fzQuat.y();
  acc[2] += weight * sign * fzQuat.z();
  acc[3] += weight * sign * fzQuat.w();
}

void MisorientationKDE::finalize()
{
  m_Centers.clear();
  if(m_TotalWeight <= 0.0)
  {
    return;
  }
  for(size_t binIndex = 0; binIndex < m_BinWeights.size(); binIndex++)
  {
    if(m_BinWeights[binIndex] > 0.0)
    {
      // Use the bin's weighted circular-mean quaternion (accumulated in addMisorientation) as the
      // representative center rather than the geometric bin center. This eliminates the ~5-degree
      // MDF-bin quantization bias that otherwise shifts the extracted angle-distribution curve.
      const std::array<double, 4>& acc = m_BinQuatSum[binIndex];
      const double norm = std::sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2] + acc[3] * acc[3]);
      QuatD quat = (norm > 0.0) ? QuatD(acc[0] / norm, acc[1] / norm, acc[2] / norm, acc[3] / norm) : binCenter(static_cast<int>(binIndex));
      m_Centers.push_back({quat, quat.conjugate(), m_BinWeights[binIndex] / m_TotalWeight});
    }
  }
}

double MisorientationKDE::totalWeight() const
{
  return m_TotalWeight;
}

QuatD MisorientationKDE::binCenter(int binIndex) const
{
  double center[3] = {0.5, 0.5, 0.5};
  RodriguesDType rod = m_Ops->determineRodriguesVector(center, binIndex);
  return rod.toQuaternion();
}

double MisorientationKDE::evaluate(const QuatD& query) const
{
  const double cutoffCos = std::cos(m_Kernel.cutoffAngle() / 2.0);
  const size_t numSymOps = m_SymQuats.size();

  // Symmetrize the query once: s_i * q * s_j for all crystal-symmetry pairs.
  std::vector<QuatD> symQueries;
  symQueries.reserve(numSymOps * numSymOps);
  for(size_t i = 0; i < numSymOps; i++)
  {
    QuatD left = m_SymQuats[i] * query;
    for(size_t j = 0; j < numSymOps; j++)
    {
      symQueries.push_back(left * m_SymQuats[j]);
    }
  }

  double density = 0.0;
  for(const Center& center : m_Centers)
  {
    double kernelSum = 0.0;
    for(const QuatD& symQuery : symQueries)
    {
      double dotForward = std::fabs(symQuery.dotProduct(center.Quat));
      if(dotForward >= cutoffCos)
      {
        kernelSum += m_Kernel.evaluate(dotForward);
      }
      double dotInverse = std::fabs(symQuery.dotProduct(center.QuatInverse));
      if(dotInverse >= cutoffCos)
      {
        kernelSum += m_Kernel.evaluate(dotInverse);
      }
    }
    // Sum of the forward K(g, c) and grain-exchange K(g, inv(c)) kernels, averaged
    // over the |CS| x |CS| crystal-symmetry pairs, then halved. The forward and inverse
    // terms together span the grain-exchange-extended symmetry orbit (2 * |CS| * |CS|
    // elements), so the correct mean-1 normalization divides by 2 * |CS|^2, i.e. the
    // 0.5 antipodal factor. This was pinned by a direct numerical cross-check against
    // MTEX 6.1.0 (calcDensity 'exact'): with exact (un-gridified) centers our density
    // equals MTEX's mdf to a constant ratio of exactly 2.000 across all misorientation
    // angles, and MTEX's mdf has mean(mdf) == 1 over SO(3). Dropping this factor gives a
    // mean of ~2 (and a triclinic modal peak of K(0) instead of K(0)/2). See
    // MisorientationKDETest.cpp::CubicAngleCurveVsMTEX for the cross-check.
    density += center.Weight * kernelSum / static_cast<double>(2 * numSymOps * numSymOps);
  }
  return density;
}

std::vector<double> MisorientationKDE::evaluateAtBinCenters() const
{
  size_t mdfSize = m_Ops->getMDFSize();
  std::vector<double> densities(mdfSize, 0.0);
  for(size_t binIndex = 0; binIndex < mdfSize; binIndex++)
  {
    densities[binIndex] = evaluate(binCenter(static_cast<int>(binIndex)));
  }
  return densities;
}

MisorientationKDE::AngleCurve MisorientationKDE::computeAngleCurve(size_t numPoints) const
{
  AngleCurve curve;
  const uint32_t structure = m_CrystalStructure;
  const double maxAngle = random_angle_distribution::MaxMisorientationAngle(structure);

  curve.Angles.resize(numPoints);
  for(size_t i = 0; i < numPoints; i++)
  {
    curve.Angles[i] = maxAngle * static_cast<double>(i) / static_cast<double>(numPoints - 1);
  }
  curve.RandomDensity = random_angle_distribution::Compute(structure, curve.Angles);
  curve.Density = curve.RandomDensity; // start from the uniform reference, MTEX-style

  const double resolution = 0.5 * constants::k_DegToRadD;                // MTEX default 'resolution'
  const double gridScale = 2.0 * static_cast<double>(m_SymQuats.size()); // full-sphere grid vs MTEX sector grid
  const size_t maxAxes = 20000;
  constexpr double k_GoldenAngle = 2.399963229728653;

  for(size_t i = 0; i < numPoints; i++)
  {
    const double omega = curve.Angles[i];
    const double sinHalf = std::sin(omega / 2.0);
    const double cosHalf = std::cos(omega / 2.0);
    size_t numAxes = static_cast<size_t>(std::lround(gridScale * (4.0 / 3.0) * sinHalf * sinHalf / (resolution * resolution)));
    numAxes = std::clamp<size_t>(numAxes, 1, maxAxes);

    double sum = 0.0;
    size_t accepted = 0;
    for(size_t a = 0; a < numAxes; a++)
    {
      // Fibonacci sphere point a of numAxes.
      const double z = 1.0 - 2.0 * (static_cast<double>(a) + 0.5) / static_cast<double>(numAxes);
      const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
      const double phi = static_cast<double>(a) * k_GoldenAngle;
      const double axisX = r * std::cos(phi);
      const double axisY = r * std::sin(phi);
      const double axisZ = z;

      QuatD q(axisX * sinHalf, axisY * sinHalf, axisZ * sinHalf, cosHalf);
      // Keep only MDF-fundamental-zone representatives: fold with getMDFFZRod and
      // compare in homochoric space; each misorientation class is counted once.
      RodriguesDType rod = q.toRodrigues();
      HomochoricDType hoOriginal = rod.toHomochoric();
      HomochoricDType hoFolded = m_Ops->getMDFFZRod(rod).toHomochoric();
      const double tolerance = 1.0e-6;
      if(std::fabs(hoOriginal[0] - hoFolded[0]) > tolerance || std::fabs(hoOriginal[1] - hoFolded[1]) > tolerance || std::fabs(hoOriginal[2] - hoFolded[2]) > tolerance)
      {
        continue;
      }
      sum += evaluate(q);
      accepted++;
    }
    if(accepted > 0)
    {
      curve.Density[i] *= std::max(0.0, sum / static_cast<double>(accepted));
    }
    // MTEX leaves density(i) at the uniform value when the slice grid is empty.
  }
  return curve;
}
} // namespace ebsdlib
