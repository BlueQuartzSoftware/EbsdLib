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
/**
 * @brief Build a supergroup FundamentalSectorGeometry from a crystal structure index.
 *
 * The indices come from FundamentalSectorGeometry::supergroupIndex() and
 * correspond to the EbsdLibConstants.h crystal structure numbering.
 */
std::unique_ptr<FundamentalSectorGeometry> buildSupergroupSector(int32_t index)
{
  switch(index)
  {
  case 0:
    return std::make_unique<FundamentalSectorGeometry>(FundamentalSectorGeometry::hexagonalHigh());
  case 1:
    return std::make_unique<FundamentalSectorGeometry>(FundamentalSectorGeometry::cubicHigh());
  case 6:
    return std::make_unique<FundamentalSectorGeometry>(FundamentalSectorGeometry::orthorhombic());
  case 8:
    return std::make_unique<FundamentalSectorGeometry>(FundamentalSectorGeometry::tetragonalHigh());
  default:
    return nullptr;
  }
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
  // For extended color keys, construct the supergroup's sector
  if(m_Sector.colorKeyMode() == "extended" && m_Sector.supergroupIndex() >= 0)
  {
    m_SupergroupSector = buildSupergroupSector(m_Sector.supergroupIndex());
  }
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
  double lHsl = 0.5; // default = fully saturated

  if(m_Sector.colorKeyMode() == "standard" || m_Sector.colorKeyMode() == "impossible")
  {
    // Standard: white center only
    // Center (radius=0) -> white (L=1.0), boundary (radius=1) -> saturated (L=0.5)
    double theta = radius * k_HalfPi;
    double lRaw = lightness(theta, m_LambdaL);
    double lBoundary = lightness(k_HalfPi, m_LambdaL);
    double lNormalized = (lBoundary > 1.0e-10) ? (lRaw / lBoundary) : radius;
    lHsl = 1.0 - 0.5 * lNormalized;
  }
  else if(m_Sector.colorKeyMode() == "extended" && m_SupergroupSector)
  {
    // Extended: check if direction is in the supergroup sector
    bool inSupergroup = m_SupergroupSector->isInside(direction);

    if(inSupergroup)
    {
      // White center half: L goes from 1.0 (center) to 0.5 (boundary)
      // Use the supergroup's polar coordinates for smoother mapping
      auto [sgRadius, sgRho] = m_SupergroupSector->polarCoordinates(direction);
      hue = sgRho / k_TwoPi; // use supergroup's azimuthal angle for hue
      double theta = sgRadius * k_HalfPi;
      double lRaw = lightness(theta, m_LambdaL);
      double lBoundary = lightness(k_HalfPi, m_LambdaL);
      double lNormalized = (lBoundary > 1.0e-10) ? (lRaw / lBoundary) : sgRadius;
      lHsl = 1.0 - 0.5 * lNormalized; // maps [0,1] -> [1.0, 0.5]
    }
    else
    {
      // Black center half: L goes from 0.0 (center) to 0.5 (boundary)
      // Use the main sector's polar coords but invert the lightness mapping
      double theta = radius * k_HalfPi;
      double lRaw = lightness(theta, m_LambdaL);
      double lBoundary = lightness(k_HalfPi, m_LambdaL);
      double lNormalized = (lBoundary > 1.0e-10) ? (lRaw / lBoundary) : radius;
      lHsl = 0.5 * lNormalized; // maps [0,1] -> [0.0, 0.5]
    }
  }

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
