#pragma once

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Utilities/IColorKey.hpp"

#include <memory>
#include <string>
#include <vector>

namespace ebsdlib
{

/**
 * @brief Decorator that wraps any IColorKey with grid-based flat shading.
 *
 * On construction, precomputes colors at a regular grid of (eta, chi) sample
 * points by calling the inner color key. When direction2Color() is called,
 * it snaps the direction to the nearest grid point and returns the precomputed
 * color (flat shading), producing smooth color patches that hide C1
 * discontinuities in the underlying color function.
 *
 * This replicates the MTEX rendering approach where colors are sampled at
 * ~1-degree intervals and rendered as flat-colored quadrilateral patches.
 *
 * Usage: pass `gridded = true` to LaueOps::generateIPFTriangleLegend(), which
 * will wrap the kind-selected key in a GriddedColorKey internally. Direct
 * construction is only needed for tests or custom pipelines.
 */
class EbsdLib_EXPORT GriddedColorKey : public IColorKey
{
public:
  /**
   * @brief Construct a grid-decorated color key.
   * @param innerKey The underlying color key to sample from
   * @param resolutionDeg Grid cell size in degrees (default 1.0, matching MTEX)
   */
  explicit GriddedColorKey(IColorKey::Pointer innerKey, double resolutionDeg = 1.0);
  ~GriddedColorKey() override = default;

  Vec3 direction2Color(const Vec3& direction) const override;
  Vec3 direction2Color(double eta, double chi, const Vec3& angleLimits) const override;
  std::string name() const override;

  /**
   * @brief Get the underlying (unwrapped) color key.
   */
  IColorKey::Pointer innerKey() const;

  /**
   * @brief Get the grid resolution in degrees.
   */
  double resolutionDeg() const;

private:
  IColorKey::Pointer m_InnerKey;
  double m_ResolutionRad; // grid cell size in radians

  // Precomputed color grid: m_Grid[etaIdx][chiIdx] = RGB color
  // Covers eta in [0, pi] and chi in [0, pi/2] to handle all Laue groups
  std::vector<std::vector<Vec3>> m_Grid;
  int m_EtaSteps;
  int m_ChiSteps;
  double m_ResolutionDeg;

  void precomputeGrid();
  Vec3 lookupGrid(double eta, double chi) const;
};

} // namespace ebsdlib
