#pragma once

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Utilities/IColorKey.hpp"

namespace ebsdlib
{

/**
 * @brief Traditional TSL/HKL IPF color key.
 * Refactored from LaueOps::computeIPFColor().
 * The spherical coordinate overload is the primary interface for this key.
 */
class EbsdLib_EXPORT TSLColorKey : public IColorKey
{
public:
  TSLColorKey() = default;
  ~TSLColorKey() override = default;

  Vec3 direction2Color(double eta, double chi, const Vec3& angleLimits) const override;
  Vec3 direction2Color(const Vec3& direction) const override;
  std::string name() const override;

  void setDefaultAngleLimits(const Vec3& limits);

private:
  Vec3 m_DefaultAngleLimits = {0.0, 0.7854, 0.6155};
};

} // namespace ebsdlib
