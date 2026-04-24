#include "EbsdLib/Utilities/TSLColorKey.hpp"

#include <algorithm>
#include <cmath>

namespace ebsdlib
{

TSLColorKey::Vec3 TSLColorKey::direction2Color(double eta, double chi, const Vec3& angleLimits) const
{
  double etaMin = angleLimits[0];
  double etaMax = angleLimits[1];
  double chiMax = angleLimits[2];

  double r = 1.0 - chi / chiMax;
  double b = std::abs(eta - etaMin) / (etaMax - etaMin);
  double g = 1.0 - b;
  g *= chi / chiMax;
  b *= chi / chiMax;

  r = std::sqrt(r);
  g = std::sqrt(g);
  b = std::sqrt(b);

  double maxVal = std::max({r, g, b});
  if(maxVal > 0.0)
  {
    r /= maxVal;
    g /= maxVal;
    b /= maxVal;
  }

  return {std::clamp(r, 0.0, 1.0), std::clamp(g, 0.0, 1.0), std::clamp(b, 0.0, 1.0)};
}

TSLColorKey::Vec3 TSLColorKey::direction2Color(const Vec3& direction) const
{
  double chi = std::acos(std::clamp(direction[2], -1.0, 1.0));
  double eta = std::atan2(direction[1], direction[0]);
  return direction2Color(eta, chi, m_DefaultAngleLimits);
}

void TSLColorKey::setDefaultAngleLimits(const Vec3& limits)
{
  m_DefaultAngleLimits = limits;
}

std::string TSLColorKey::name() const
{
  return "TSL";
}

} // namespace ebsdlib
