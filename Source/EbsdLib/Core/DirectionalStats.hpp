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

  void setNumEM(int NumEM)
  {
    NumEM_ = NumEM;
  }
  void setNumIter(int NumIter)
  {
    NumIter_ = NumIter;
  }

  void EMforDS(uint32_t& seed, QuatD& muhat, double& kappahat, bool verbose);

  std::vector<double> Estep_(const QuatD& Mu, double Kappa) const;

  std::vector<double> Density_(const QuatD& mu, double kappa, double C) const;

  double logCp_(double kappa) const;

  std::array<double, 5> Mstep_(const std::vector<double>& R, int N, int Pmdims) const;

  void getQandL_(const std::array<double, 5>& MuKa, const std::vector<double>& R, double& Q, double& L) const;

  // struct qsym_
  // {
  //
  //
  //   QuatD getQuatfromArray(int i) const
  //   {
  //     return laueOps->getQuatSymOp(i);
  //   }
  //
  // } qsym;

  // struct Xquats_
  // {
  //
  //   std::vector<QuatD> Quats;
  // } Xquats;

  // int getQnumber() const
  // {
  //   return laueOps->getNumSymOps();
  // }

  QuatD getQuatfromArray(int i) const
  {
    return m_XQuats[i];
  }

  void setQuatArray(const std::vector<QuatD>& quats)
  {
    m_XQuats = quats; // THIS IS GOING TO MAKE COPY!!! THIS IS REALLY BAD.
  }

  int getN() const
  {
    return m_XQuats.size();
  }

  private:
  int NumEM_ = 0;
  int NumIter_ = 0;
  std::string DStype = "";
  // int Pmdims_ = 0; // This is just the number of Symmetry operators for a given LaueClass

  int Apnum = 3500;
  std::vector<double> xAp;
  std::vector<double> yAp;

  std::vector<QuatD> m_XQuats;

  LaueOps::Pointer m_LaueOps;
};
}
