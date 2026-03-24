#include "EbsdLib/Utilities/NolzeHielscherColorKey.hpp"
#include "EbsdLib/Utilities/FundamentalSectorGeometry.hpp"
#include "EbsdLib/Utilities/ColorSpaceUtils.hpp"

#include <algorithm>
#include <memory>

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
//
// Implements the Nolze-Hielscher coloring approach from the paper:
//   1. Polar coordinates (radius, rho) from the sector geometry
//   2. Hue from azimuthal angle rho
//   3. Lightness from radial distance using a gray gradient blending
//      that produces a compact white/gray center with saturated colors
//      covering most of the sector area
//   4. Saturation modulated by lightness
//   5. HSL -> RGB
//
// The gray gradient approach (Paper Section 2.4, Appendix A.2):
//   - Maps radius [0,1] to a theta parameter in [0.5, 1.0] (white center)
//   - Blends linear and cosine curves for the transition
//   - Applies a gray value that controls how white the center is
//   - The result: center is near-white, colors saturate quickly
// -----------------------------------------------------------------------
NolzeHielscherColorKey::Vec3 NolzeHielscherColorKey::direction2Color(const Vec3& direction) const
{
  // 1. Get polar coordinates from the fundamental sector geometry
  auto [radius, rho] = m_Sector.polarCoordinates(direction);

  // 2. Hue from azimuthal angle
  // rho is in [0, 2*pi) -- normalize to [0, 1) for HSL conversion
  double hue = rho / k_TwoPi;

  // 3. Lightness from radial distance via gray gradient blending
  //
  // The approach derived from the paper (Appendix A.2):
  //   theta_mapped = radius_mapped (in [0.5, 1.0] for white center)
  //   Apply nonlinear blend: th = (2*gg*th + (1-gg)*(1-cos(th*pi)))/2
  //   where gg = grayGradient (0.5 default)
  //   Then compute gray and saturation from the corrected theta
  constexpr double k_GrayGradient = 0.5;
  constexpr double k_GrayValueWhite = 0.2; // controls how white the center is (lower = more saturated center)
  constexpr double k_GrayValueBlack = 0.5; // controls how black the dark center is

  double lHsl = 0.5; // default = fully saturated
  double sHsl = 1.0;

  if(m_Sector.colorKeyMode() == "standard" || m_Sector.colorKeyMode() == "impossible")
  {
    // Standard: white center only
    // Map radius [0,1] -> [1.0, 0.5]
    // Center (r=0) -> 1.0 (north pole of color sphere = white/gray)
    // Boundary (r=1) -> 0.5 (equator of color sphere = fully saturated)
    double radiusMapped = 1.0 - radius / 2.0;

    // Apply gray gradient correction (blends linear and cosine curves)
    double th = (2.0 * k_GrayGradient * radiusMapped + (1.0 - k_GrayGradient) * (1.0 - std::cos(radiusMapped * k_Pi))) / 2.0;

    // Compute gray value: controls saturation envelope
    double gray = 1.0 - 2.0 * k_GrayValueWhite * std::abs(th - 0.5);

    // Compute HSL lightness and saturation
    lHsl = (th - 0.5) * gray + 0.5;
    double denominator = 1.0 - std::abs(2.0 * lHsl - 1.0);
    sHsl = (denominator > 1.0e-10) ? gray * (1.0 - std::abs(2.0 * th - 1.0)) / denominator : 0.0;
    sHsl = std::clamp(sHsl, 0.0, 1.0);
  }
  else if(m_Sector.colorKeyMode() == "extended" && m_SupergroupSector)
  {
    // Extended: check if direction is in the supergroup sector
    bool inSupergroup = m_SupergroupSector->isInside(direction);

    double radiusMapped;
    double grayValue;

    if(inSupergroup)
    {
      // White center half: radius_mapped [1.0, 0.5]
      // Center (r=0) -> 1.0 (north pole = white), boundary (r=1) -> 0.5 (equator = saturated)
      auto [sgRadius, sgRho] = m_SupergroupSector->polarCoordinates(direction);
      hue = sgRho / k_TwoPi;
      radiusMapped = 1.0 - sgRadius / 2.0;
      grayValue = k_GrayValueWhite;
    }
    else
    {
      // Black center half: radius_mapped [0.0, 0.5]
      // Center (r=0) -> 0.0 (south pole = black), boundary (r=1) -> 0.5 (equator = saturated)
      radiusMapped = radius / 2.0;
      grayValue = k_GrayValueBlack;
    }

    // Apply gray gradient correction
    double th = (2.0 * k_GrayGradient * radiusMapped + (1.0 - k_GrayGradient) * (1.0 - std::cos(radiusMapped * k_Pi))) / 2.0;

    // Compute gray value
    double gray = 1.0 - 2.0 * grayValue * std::abs(th - 0.5);

    // Compute HSL lightness and saturation
    lHsl = (th - 0.5) * gray + 0.5;
    double denominator = 1.0 - std::abs(2.0 * lHsl - 1.0);
    sHsl = (denominator > 1.0e-10) ? gray * (1.0 - std::abs(2.0 * th - 1.0)) / denominator : 0.0;
    sHsl = std::clamp(sHsl, 0.0, 1.0);
  }

  // 5. Convert HSL to RGB
  auto rgb = color::hslToRgb(hue, sHsl, lHsl);
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
