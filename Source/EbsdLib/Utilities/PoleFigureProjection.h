#pragma once

#include <array>

#include "EbsdLib/EbsdLib.h"

namespace ebsdlib
{
/**
 * @brief Projects a unit direction vector to the upper-hemisphere stereographic disk.
 * Folds any pole with z<0 into the upper hemisphere (negates the vector), then applies
 * dx=x/(1+z), dy=y/(1+z). Returned coords lie within the unit disk (|d| <= 1).
 */
EbsdLib_EXPORT std::array<float, 2> StereographicProjectUpperHemisphere(float x, float y, float z);
} // namespace ebsdlib
