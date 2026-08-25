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

#include "SO3DeLaValleePoussinKernel.h"

#include "EbsdLib/Math/EbsdLibMath.h"

#include <algorithm>
#include <cmath>

namespace
{
// std::beta is unavailable on libc++; use lgamma
double BetaFunction(double a, double b)
{
  return std::exp(std::lgamma(a) + std::lgamma(b) - std::lgamma(a + b));
}
} // namespace

namespace ebsdlib
{
SO3DeLaValleePoussinKernel::SO3DeLaValleePoussinKernel(double halfwidthRadians)
: m_Halfwidth(halfwidthRadians)
{
  m_Kappa = 0.5 * std::log(0.5) / std::log(std::cos(halfwidthRadians / 2.0));
  m_C = BetaFunction(1.5, 0.5) / BetaFunction(1.5, m_Kappa + 0.5);
}

double SO3DeLaValleePoussinKernel::kappa() const
{
  return m_Kappa;
}

double SO3DeLaValleePoussinKernel::constant() const
{
  return m_C;
}

double SO3DeLaValleePoussinKernel::halfwidth() const
{
  return m_Halfwidth;
}

double SO3DeLaValleePoussinKernel::evaluate(double cosHalfOmega) const
{
  return m_C * std::pow(cosHalfOmega, 2.0 * m_Kappa);
}

double SO3DeLaValleePoussinKernel::cutoffAngle() const
{
  return std::min(constants::k_PiD, 3.5 * m_Halfwidth);
}
} // namespace ebsdlib
