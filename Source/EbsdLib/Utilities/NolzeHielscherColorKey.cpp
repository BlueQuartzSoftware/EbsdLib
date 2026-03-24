#include "EbsdLib/Utilities/NolzeHielscherColorKey.hpp"

#include "EbsdLib/Utilities/ColorSpaceUtils.hpp"

#include <algorithm>
#include <cmath>

namespace ebsdlib
{

namespace
{
constexpr double k_Pi = 3.14159265358979323846;
constexpr double k_TwoPi = 2.0 * k_Pi;
constexpr double k_HalfPi = k_Pi / 2.0;

/**
 * @brief Wrap an angle in degrees to [-180, 180].
 */
double wrapDeg(double x)
{
  x = std::fmod(x + 180.0, 360.0);
  if(x < 0.0)
  {
    x += 360.0;
  }
  return x - 180.0;
}
} // namespace

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------
NolzeHielscherColorKey::NolzeHielscherColorKey(const FundamentalSectorGeometry& sector, double lambdaL, double lambdaS)
: m_Sector(sector)
, m_LambdaL(lambdaL)
, m_LambdaS(lambdaS)
{
}

// -----------------------------------------------------------------------
// hueSpeedFunction  (Paper Appendix A.1, Eq. 5)
// -----------------------------------------------------------------------
double NolzeHielscherColorKey::hueSpeedFunction(double rhoDeg, double distance)
{
  double v = 0.5;
  v += std::exp(-std::abs(wrapDeg(rhoDeg)) / 4.0);
  v += std::exp(-std::abs(wrapDeg(rhoDeg - 120.0)) / 4.0);
  v += std::exp(-std::abs(wrapDeg(rhoDeg + 120.0)) / 4.0);
  return v * distance;
}

// -----------------------------------------------------------------------
// lightness  (Paper Appendix A.2)
// -----------------------------------------------------------------------
double NolzeHielscherColorKey::lightness(double theta, double lambdaL)
{
  double sinHalf = std::sin(theta / 2.0);
  return lambdaL * (theta / k_HalfPi) + (1.0 - lambdaL) * sinHalf * sinHalf;
}

// -----------------------------------------------------------------------
// saturation  (Paper Appendix A.2)
// -----------------------------------------------------------------------
double NolzeHielscherColorKey::saturation(double L, double lambdaS)
{
  return std::clamp(1.0 - 2.0 * lambdaS * std::abs(L - 0.5), 0.0, 1.0);
}

// -----------------------------------------------------------------------
// direction2Color
// -----------------------------------------------------------------------
NolzeHielscherColorKey::Vec3 NolzeHielscherColorKey::direction2Color(const Vec3& direction) const
{
  // 1. Get polar coordinates from the fundamental sector geometry
  auto [radius, rho] = m_Sector.polarCoordinates(direction);

  // 2. Hue from azimuthal angle
  // rho is in [0, 2*pi) -- normalize to [0, 1) for HSL conversion
  double hue = rho / k_TwoPi;

  // 3. Lightness from radial distance
  // theta maps radius [0,1] to angular distance [0, pi/2]
  double theta = radius * k_HalfPi;
  double lRaw = lightness(theta, m_LambdaL);

  // Compute the lightness at the boundary (radius=1, theta=pi/2) for normalization
  double lBoundary = lightness(k_HalfPi, m_LambdaL);

  // Normalize to [0, 1]
  double lNormalized = (lBoundary > 1.0e-10) ? (lRaw / lBoundary) : radius;

  // Map to HSL lightness: center (radius=0) -> white (L=1.0),
  // boundary (radius=1) -> fully saturated (L=0.5)
  double lHsl = 1.0 - 0.5 * lNormalized;

  // 4. Saturation
  double s = saturation(lHsl, m_LambdaS);

  // 5. Convert HSL to RGB
  auto rgb = color::hslToRgb(hue, s, lHsl);
  return {rgb[0], rgb[1], rgb[2]};
}

// -----------------------------------------------------------------------
// name
// -----------------------------------------------------------------------
std::string NolzeHielscherColorKey::name() const
{
  return "NolzeHielscher";
}

} // namespace ebsdlib
