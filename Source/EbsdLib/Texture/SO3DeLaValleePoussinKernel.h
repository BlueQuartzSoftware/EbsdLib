#pragma once

#include "EbsdLib/EbsdLib.h"

namespace ebsdlib
{
/**
 * @brief De la Vallee Poussin kernel on SO(3). Port of MTEX SO3DeLaValleePoussinKernel.
 *
 * K(omega) = C * cos(omega/2)^(2*kappa) with
 *   kappa = ln(0.5) / (2 * ln(cos(halfwidth/2)))
 *   C     = B(1.5, 0.5) / B(1.5, kappa + 0.5)
 * The kernel integrates to 1 over SO(3) with normalized Haar measure, so a
 * weights-sum-to-one mixture of kernels is a normalized density (uniform == 1).
 */
class EbsdLib_EXPORT SO3DeLaValleePoussinKernel
{
public:
  explicit SO3DeLaValleePoussinKernel(double halfwidthRadians);

  double kappa() const;
  double constant() const;
  double halfwidth() const;

  /**
   * @brief Evaluate the kernel.
   * @param cosHalfOmega cos(omega/2); pass the absolute quaternion dot product.
   */
  double evaluate(double cosHalfOmega) const;

  /**
   * @brief Angle beyond which the kernel is treated as zero: min(pi, 3.5*halfwidth).
   */
  double cutoffAngle() const;

private:
  double m_Halfwidth = 0.0;
  double m_Kappa = 90.0;
  double m_C = 0.0;
};
} // namespace ebsdlib
