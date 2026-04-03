#include "EbsdLib/Utilities/NolzeHielscherColorKey.hpp"
#include "EbsdLib/Utilities/ColorSpaceUtils.hpp"
#include "EbsdLib/Utilities/FundamentalSectorGeometry.hpp"

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
  precomputeHueCdf();
}

// -----------------------------------------------------------------------
// precomputeHueCdf
//
// Build a CDF from Gaussian bumps at R(0), G(1/3), B(2/3) positions.
// This redistributes hue so that yellow, cyan, and magenta get
// proportionally more angular space (they are compressed in raw HSV).
//
// From the paper (Appendix A.1): the hue speed function has Gaussian
// peaks at the three primary positions. The CDF of this function
// remaps hue to equalize the color distribution.
// -----------------------------------------------------------------------
void NolzeHielscherColorKey::precomputeHueCdf()
{
  // Build the speed function f(z) with Gaussian bumps
  constexpr double k_GaussWidth = 200.0; // Controls bump sharpness (larger = narrower bumps)
  constexpr double k_Baseline = 0.5;     // Constant baseline
  std::array<double, k_HueCdfSize> f = {};

  for(size_t i = 0; i < k_HueCdfSize; i++)
  {
    double z = static_cast<double>(i) / static_cast<double>(k_HueCdfSize);
    double val = k_Baseline;
    // Three Gaussian bumps at red (0), green (1/3), blue (2/3)
    for(double center : {0.0, 1.0 / 3.0, 2.0 / 3.0})
    {
      double dx = std::fmod(z - center + 0.5, 1.0) - 0.5; // periodic wrap to [-0.5, 0.5]
      val += std::exp(-k_GaussWidth * dx * dx);
    }
    f[i] = val;
  }

  // Normalize to probability distribution
  double sum = 0.0;
  for(auto v : f)
  {
    sum += v;
  }
  for(auto& v : f)
  {
    v /= sum;
  }

  // Cumulative sum -> CDF
  m_HueCdf[0] = f[0];
  for(size_t i = 1; i < k_HueCdfSize; i++)
  {
    m_HueCdf[i] = m_HueCdf[i - 1] + f[i];
  }
  // Ensure last entry is exactly 1.0
  m_HueCdf[k_HueCdfSize - 1] = 1.0;
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
// correctHue -- Gaussian CDF-based hue redistribution
// -----------------------------------------------------------------------
double NolzeHielscherColorKey::correctHue(double hueIn) const
{
  // hueIn is in [0, 1)
  double h = std::fmod(hueIn, 1.0);
  if(h < 0.0)
  {
    h += 1.0;
  }

  // Fractional index into CDF table
  double fIdx = h * static_cast<double>(k_HueCdfSize);
  size_t idx0 = static_cast<size_t>(fIdx);
  double frac = fIdx - static_cast<double>(idx0);

  if(idx0 >= k_HueCdfSize - 1)
  {
    return m_HueCdf[k_HueCdfSize - 1];
  }

  // Linear interpolation
  return m_HueCdf[idx0] * (1.0 - frac) + m_HueCdf[idx0 + 1] * frac;
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
  // First apply boundary-distance-weighted azimuthal correction to smooth
  // the transitions between boundary zones and equalize vertex hue sectors.
  // Then apply Gaussian CDF correction to expand yellow/cyan/magenta regions.
  double rhoCorrected = m_Sector.correctAzimuthalAngle(rho);
  double hue = correctHue(rhoCorrected / k_TwoPi);

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

  // Common lightness/saturation computation using the color sphere model.
  // The radius [0,1] maps to a position on the color sphere:
  //   Center (r=0) -> white (HSL L=1, desaturated)
  //   Boundary (r=1) -> fully saturated (HSL L=0.5, full saturation)
  //
  // The key insight: map radius to the color sphere's polar angle theta,
  // then extract HSL from the sphere position. The sphere model:
  //   theta=0 (north pole) = white, theta=pi/2 (equator) = saturated, theta=pi (south pole) = black
  //
  // For white center: radius [0,1] -> theta [pi, pi/2] (from pole to equator)
  // For black center: radius [0,1] -> theta [0, pi/2]

  auto computeColorFromSphere = [&](double r, double grayValue) -> void {
    // Map radius to color sphere theta.
    // Use a nonlinear mapping that compresses the neutral center:
    //   Apply gray gradient blending between linear and cosine curves
    double th = (2.0 * k_GrayGradient * r + (1.0 - k_GrayGradient) * (1.0 - std::cos(r * k_Pi))) / 2.0;

    // Compute gray value envelope: peak saturation at th=0.5, reduced at poles
    double gray = 1.0 - 2.0 * grayValue * std::abs(th - 0.5);

    // HSL lightness: th=0 maps to L=0.5, th=0.5 maps to L=0.5, th=1 maps to L=0.5
    // Actually: L = (th - 0.5)*gray + 0.5
    //   At th=0: L = -0.5*gray + 0.5 (dark)
    //   At th=0.5: L = 0.5 (fully saturated)
    //   At th=1: L = 0.5*gray + 0.5 (light/white)
    lHsl = (th - 0.5) * gray + 0.5;

    // HSL saturation: derived from the chroma at this sphere position
    double denominator = 1.0 - std::abs(2.0 * lHsl - 1.0);
    sHsl = (denominator > 1.0e-10) ? gray * (1.0 - std::abs(2.0 * th - 1.0)) / denominator : 0.0;
    sHsl = std::clamp(sHsl, 0.0, 1.0);
  };

  if(m_Sector.colorKeyMode() == "standard" || m_Sector.colorKeyMode() == "impossible")
  {
    // Standard: white center only
    // Radius convention: 1 at center, 0 at boundary
    // Map to sphere parameter: center(r=1) -> 1.0 (white), boundary(r=0) -> 0.5 (saturated)
    double r = 0.5 + radius / 2.0;
    computeColorFromSphere(r, k_GrayValueWhite);
  }
  else if(m_Sector.colorKeyMode() == "extended" && m_SupergroupSector)
  {
    bool inSupergroup = m_SupergroupSector->isInside(direction);

    if(inSupergroup)
    {
      auto [sgRadius, sgRho] = m_SupergroupSector->polarCoordinates(direction);
      double sgRhoCorrected = m_SupergroupSector->correctAzimuthalAngle(sgRho);
      hue = correctHue(sgRhoCorrected / k_TwoPi);
      // White center half: center(r=1)->1.0(white), boundary(r=0)->0.5(saturated)
      double r = 0.5 + sgRadius / 2.0;
      computeColorFromSphere(r, k_GrayValueWhite);
    }
    else
    {
      // Black center half: center(r=1)->0.0(black), boundary(r=0)->0.5(saturated)
      double rEff = radius;
      double r = rEff / 2.0;
      computeColorFromSphere(r, k_GrayValueBlack);
    }
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
