#pragma once

#include "EbsdLib/EbsdLib.h"

#include <array>
#include <cmath>
#include <memory>
#include <string>

namespace ebsdlib
{

/**
 * @brief Abstract interface for IPF color key strategies.
 *
 * Maps a crystal direction (already projected into the fundamental sector)
 * to an RGB color. Implementations include TSL (traditional) and
 * Nolze-Hielscher (perceptually improved).
 *
 * Two overloads are provided:
 * - direction2Color(Vec3): takes a 3D unit direction vector (preferred for N-H)
 * - direction2Color(eta, chi, angleLimits): takes spherical coords (TSL compatibility)
 */
class EbsdLib_EXPORT IColorKey
{
public:
  using Pointer = std::shared_ptr<IColorKey>;
  using Vec3 = std::array<double, 3>;

  virtual ~IColorKey() = default;

  /**
   * @brief Map a unit crystal direction vector (in the fundamental sector) to an RGB color.
   * This is the primary interface. The direction must already be projected into the SST.
   * @param direction Unit direction vector {x, y, z} in the fundamental sector
   * @return {R, G, B} each in [0.0, 1.0]
   */
  virtual Vec3 direction2Color(const Vec3& direction) const = 0;

  /**
   * @brief Map a crystal direction via spherical coordinates to an RGB color.
   * Provided for backward compatibility with the TSL pipeline.
   * Default implementation converts to a direction vector and calls the Vec3 overload.
   * @param eta Azimuthal angle of the direction (radians)
   * @param chi Polar angle of the direction from z-axis (radians)
   * @param angleLimits {etaMin, etaMax, chiMax} from the LaueOps subclass
   * @return {R, G, B} each in [0.0, 1.0]
   */
  virtual Vec3 direction2Color(double eta, double chi, const Vec3& angleLimits) const
  {
    // Default: convert spherical to Cartesian and delegate
    double sinChi = std::sin(chi);
    Vec3 dir = {sinChi * std::cos(eta), sinChi * std::sin(eta), std::cos(chi)};
    return direction2Color(dir);
  }

  /**
   * @brief Human-readable name of this color key.
   */
  virtual std::string name() const = 0;
};

} // namespace ebsdlib
