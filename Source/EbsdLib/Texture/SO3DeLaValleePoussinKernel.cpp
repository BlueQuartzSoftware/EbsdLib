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
