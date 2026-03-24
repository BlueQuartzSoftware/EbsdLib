#pragma once

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Utilities/FundamentalSectorGeometry.hpp"
#include "EbsdLib/Utilities/IColorKey.hpp"

#include <array>
#include <memory>
#include <string>

namespace ebsdlib
{

/**
 * @brief Nolze-Hielscher IPF color key.
 *
 * Implements the perceptually improved IPF coloring scheme described in:
 *   G. Nolze and R. Hielscher, "Orientations - perfectly colored",
 *   J. Appl. Cryst. (2016), 49, 1786-1802.
 *
 * Maps a crystal direction (already in the fundamental sector) to an RGB color
 * via HSL color space using polar coordinates within the sector.
 *
 * The algorithm:
 *   1. Compute polar coordinates (radius, rho) relative to the sector barycenter.
 *   2. Map the azimuthal angle rho to a hue H.
 *   3. Map the radial distance to lightness L via a nonlinear function.
 *   4. Compute saturation S from L.
 *   5. Convert (H, S, L) to RGB.
 *
 * The center of the sector maps to white and the boundary maps to fully
 * saturated colors at lightness 0.5.
 */
class EbsdLib_EXPORT NolzeHielscherColorKey : public IColorKey
{
public:
  /**
   * @brief Construct with a fundamental sector geometry.
   * @param sector The fundamental sector definition for the desired Laue group.
   * @param lambdaL Lightness nonlinearity parameter (default 0.25 per paper).
   * @param lambdaS Saturation desaturation parameter (default 0.25 per paper).
   */
  explicit NolzeHielscherColorKey(const FundamentalSectorGeometry& sector, double lambdaL = 0.25, double lambdaS = 0.25);
  ~NolzeHielscherColorKey() override = default;

  Vec3 direction2Color(const Vec3& direction) const override;
  std::string name() const override;

  /**
   * @brief Hue speed function v(rho) from paper Appendix A.1, Eq. 5.
   *
   * Controls how fast hue changes with azimuthal angle, producing perceptual
   * uniformity by slowing down near primary hues (0, 120, 240 degrees).
   *
   * @param rhoDeg Azimuthal angle in degrees
   * @param distance Boundary distance at this azimuth (scaling factor)
   * @return The hue speed value (always positive)
   */
  static double hueSpeedFunction(double rhoDeg, double distance);

  /**
   * @brief Raw lightness function from paper Appendix A.2.
   *
   * L(theta) = lambdaL * (theta / (pi/2)) + (1 - lambdaL) * sin^2(theta/2)
   *
   * @param theta Angle from center, in [0, pi/2]
   * @param lambdaL Nonlinearity parameter (0 = pure sin^2, 1 = linear)
   * @return Raw lightness value in [0, ~0.625] for lambdaL=0.25
   */
  static double lightness(double theta, double lambdaL);

  /**
   * @brief Saturation function from paper Appendix A.2.
   *
   * S = 1 - 2 * lambdaS * |L - 0.5|
   *
   * Produces maximum saturation (1.0) at L=0.5 and slightly desaturated
   * values near L=0 (black) and L=1 (white).
   *
   * @param L HSL lightness value in [0, 1]
   * @param lambdaS Desaturation parameter
   * @return Saturation in [0, 1]
   */
  static double saturation(double L, double lambdaS);

  /**
   * @brief Apply Gaussian-based hue correction to expand compressed yellow/cyan regions.
   *
   * Uses a precomputed CDF of the hue speed function to redistribute hue values
   * so that all six color sectors (R, Y, G, C, B, M) get proportional area.
   *
   * @param hueIn Raw hue in [0, 1)
   * @return Corrected hue in [0, 1)
   */
  double correctHue(double hueIn) const;

private:
  FundamentalSectorGeometry m_Sector;
  double m_LambdaL;
  double m_LambdaS;
  std::unique_ptr<FundamentalSectorGeometry> m_SupergroupSector; // null for standard/impossible

  // Precomputed hue correction CDF table (Gaussian-based redistribution)
  static constexpr size_t k_HueCdfSize = 1000;
  std::array<double, k_HueCdfSize> m_HueCdf = {};
  void precomputeHueCdf();
};

} // namespace ebsdlib
