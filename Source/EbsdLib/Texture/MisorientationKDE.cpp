#include "MisorientationKDE.h"

#include "EbsdLib/Orientation/Rodrigues.hpp"

#include <cmath>
#include <utility>

namespace ebsdlib
{
MisorientationKDE::MisorientationKDE(LaueOps::Pointer ops, uint32_t crystalStructure, double halfwidthRadians)
: m_Ops(std::move(ops))
, m_CrystalStructure(crystalStructure)
, m_Kernel(halfwidthRadians)
, m_BinWeights(m_Ops->getMDFSize(), 0.0)
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
  m_BinWeights[static_cast<size_t>(binIndex)] += weight;
  m_TotalWeight += weight;
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
      QuatD quat = binCenter(static_cast<int>(binIndex));
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
    // over the |CS| x |CS| crystal-symmetry pairs. getMDFFZRod() has already folded
    // grain exchange into the bin assignment, so the inverse term here only enforces
    // query-side grain-exchange invariance f(g) == f(g^-1); it is NOT additionally
    // halved (halving would under-normalize and drop the modal peak to K(0)/2).
    density += center.Weight * kernelSum / static_cast<double>(numSymOps * numSymOps);
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
} // namespace ebsdlib
