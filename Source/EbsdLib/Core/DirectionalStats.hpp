/* ============================================================================
 * Copyright (c) 2026 BlueQuartz Software, LLC
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
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* ============================================================================
 * THIRD-PARTY ATTRIBUTION: EMsoft
 *
 * This file is a C++ port of the directional statistics routines of the EMsoft
 * package, module 'dictmod' (Source/EMsoftLib/dictmod.f90) together with the
 * supporting Bessel function and pseudo-random number generators of
 * Source/EMsoftLib/math.f90. EMsoft is written by the Marc De Graef Research
 * Group at Carnegie Mellon University. The EMsoft source attributes the original
 * Expectation-Maximization implementation to Yu-Hui Chen (University of Michigan)
 * and Marc De Graef (Carnegie Mellon University).
 *
 * Routine correspondence, EbsdLib <- EMsoft:
 *
 *   DirectionalStats::EMforDS    <- dictmod::DI_EMforDD
 *   DirectionalStats::Estep_     <- dictmod::DD_Estep
 *   DirectionalStats::Mstep_     <- dictmod::DD_Mstep
 *   DirectionalStats::getQandL_  <- dictmod::DD_getQandL
 *   DirectionalStats::Density_   <- dictmod::DD_Density
 *   DirectionalStats::logCp_     <- dictmod::logCp
 *   BesselI0 / BesselI1 / BesselIn <- math::BesselI0 / BesselI1 / BesselIn
 *   r8_uniform_01                <- math::r8_uniform_01
 *   r8vec_uniform_01             <- math::r8vec_uniform_01
 *   r8vec_normal_01              <- math::r8vec_normal_01
 *
 * The 'VMF' and 'WAT' distribution selectors correspond to the EMsoft 'Dtype'
 * argument. The estimator is the modified, symmetry group invariant, von
 * Mises-Fisher and axial Watson distribution described in:
 *
 *  [1] Y.H. Chen, S.U. Park, D. Wei, G. Newstadt, M.A. Jackson, J.P. Simmons,
 *      M. De Graef and A.O. Hero, "A Dictionary Approach to Electron Backscatter
 *      Diffraction Indexing", Microscopy and Microanalysis 21(3), 739-752 (2015).
 *      DOI: 10.1017/S1431927615000756
 *
 *  [2] Y.H. Chen, D. Wei, G. Newstadt, M. De Graef, J.P. Simmons and A.O. Hero,
 *      "Parameter Estimation in Spherical Symmetry Groups", IEEE Signal Processing
 *      Letters 22(8), 1152-1155 (2015). DOI: 10.1109/LSP.2014.2387206
 *
 * EMsoft is available at https://github.com/EMsoft-org/EMsoft and is distributed
 * under the BSD 3-Clause license reproduced below. That notice, the list of
 * conditions and the disclaimer are retained here as the license requires.
 * ----------------------------------------------------------------------------
 * Copyright (c) 2014-2022, Marc De Graef Research Group/Carnegie Mellon University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright notice,
 *        this list of conditions and the following disclaimer.
 *     - Redistributions in binary form must reproduce the above copyright notice,
 *        this list of conditions and the following disclaimer in the documentation
 *        and/or other materials provided with the distribution.
 *     - Neither the names of Marc De Graef, Carnegie Mellon University nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

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
