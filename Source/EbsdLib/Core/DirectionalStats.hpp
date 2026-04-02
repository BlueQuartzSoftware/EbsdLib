#pragma once

#include "EbsdLib/Core/EbsdLibConstants.h"

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/LaueOps/LaueOps.h"

#include <array>
#include <string>
#include <vector>

namespace ebsdlib
{
class EbsdLib_EXPORT DirectionalStats
{
public:
  DirectionalStats(const std::string& DSType, LaueOps::Pointer laueOps);
  virtual ~DirectionalStats();

  DirectionalStats(const DirectionalStats&) = delete;
  DirectionalStats(DirectionalStats&&) noexcept = delete;
  DirectionalStats& operator=(const DirectionalStats&) = delete;
  DirectionalStats& operator=(DirectionalStats&&) noexcept = delete;

  void setNumEM(int numEM)
  {
    m_NumEM = numEM;
  }
  void setNumIter(int numIter)
  {
    m_NumIter = numIter;
  }

  void EMforDS(uint32_t& seed, QuatD& muhat, double& kappahat, bool verbose);

  std::vector<double> Estep_(const QuatD& Mu, double Kappa) const;

  std::vector<double> Density_(const QuatD& mu, double kappa, double C) const;

  double logCp_(double kappa) const;

  std::array<double, 5> Mstep_(const std::vector<double>& R, int N, int Pmdims) const;

  void getQandL_(const std::array<double, 5>& MuKa, const std::vector<double>& R, double& Q, double& L) const;

  QuatD getQuatfromArray(int i) const
  {
    return m_XQuats[i];
  }

  void setQuatArray(const std::vector<QuatD>& quats)
  {
    m_XQuats = quats;
  }

  void setQuatArray(std::vector<QuatD>&& quats)
  {
    m_XQuats = std::move(quats);
  }

  int getN() const
  {
    return m_XQuats.size();
  }

private:
  int m_NumEM = 0;
  int m_NumIter = 0;
  std::string m_DSType;

  int m_ApNum = 3500;
  std::vector<double> m_XAp;
  std::vector<double> m_YAp;

  std::vector<QuatD> m_XQuats;

  LaueOps::Pointer m_LaueOps;
};
} // namespace ebsdlib
