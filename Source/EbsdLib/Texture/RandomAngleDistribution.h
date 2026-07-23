#pragma once

#include "EbsdLib/EbsdLib.h"

#include <cstdint>
#include <vector>

namespace ebsdlib
{
namespace random_angle_distribution
{
/**
 * @brief Maximum rotation angle (radians) of the fundamental region for the Laue group.
 * Values generated from MTEX 6.1.0 fundamentalRegion(cs).maxAngle.
 */
EbsdLib_EXPORT double MaxMisorientationAngle(uint32_t crystalStructure);

/**
 * @brief Misorientation-angle distribution of the uniform (random) ODF.
 * Port of MTEX geometry/@symmetry/calcAngleDistribution.m. Result is normalized
 * to unit mean and zero-clamped. omega values beyond MaxMisorientationAngle get 0.
 * Throws std::invalid_argument for UnknownCrystalStructure.
 */
EbsdLib_EXPORT std::vector<double> Compute(uint32_t crystalStructure, const std::vector<double>& omega);
} // namespace random_angle_distribution
} // namespace ebsdlib
