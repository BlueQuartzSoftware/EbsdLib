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

/**
 * @brief Renders plain discrete pole figures as opaque filled-circle markers drawn
 * directly into a canvas_ity context (no intermediate RGBA raster). Crystallography
 * is delegated to the configured LaueOps. Use when config.discrete && !discreteHeatMap.
 */
class EbsdLib_EXPORT DiscretePoleFigureCompositor
{
public:
  DiscretePoleFigureCompositor() = default;
  ~DiscretePoleFigureCompositor() = default;
  DiscretePoleFigureCompositor(const DiscretePoleFigureCompositor&) = delete;
  DiscretePoleFigureCompositor(DiscretePoleFigureCompositor&&) = delete;
  DiscretePoleFigureCompositor& operator=(const DiscretePoleFigureCompositor&) = delete;
  DiscretePoleFigureCompositor& operator=(DiscretePoleFigureCompositor&&) = delete;

  CompositePoleFigureResult generateCompositeImage(CompositePoleFigureConfiguration_t& config);
};
} // namespace ebsdlib
