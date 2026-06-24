#pragma once

#include <array>

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Utilities/PoleFigureCompositor.h" // CompositePoleFigureConfiguration_t, CompositePoleFigureResult

namespace ebsdlib
{
/**
 * @brief Renders one opaque filled-circle marker into a square RGBA tile.
 * The tile is transparent outside the circle. outSize receives the tile edge length.
 */
EbsdLib_EXPORT UInt8ArrayType::Pointer RenderDiscreteMarkerSprite(const std::array<float, 3>& color, float radiusPx, int& outSize);
} // namespace ebsdlib
