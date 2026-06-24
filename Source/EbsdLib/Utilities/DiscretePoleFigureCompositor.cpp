#include "EbsdLib/Utilities/DiscretePoleFigureCompositor.h"

#include "EbsdLib/Math/EbsdLibMath.h"

#include <canvas_ity.hpp>

#include <algorithm>
#include <cmath>

namespace ebsdlib
{
// -----------------------------------------------------------------------------
UInt8ArrayType::Pointer RenderDiscreteMarkerSprite(const std::array<float, 3>& color, float radiusPx, int& outSize)
{
  int diameter = static_cast<int>(std::ceil(2.0f * radiusPx + 2.0f)); // +2px AA padding
  diameter = std::max(diameter, 2);
  if(diameter % 2 != 0)
  {
    diameter++; // even => exact center for centered blits
  }
  outSize = diameter;

  // Fresh canvas starts fully transparent; draw only the circle.
  canvas_ity::canvas spriteCanvas(diameter, diameter);
  spriteCanvas.set_color(canvas_ity::fill_style, color[0], color[1], color[2], 1.0f);
  spriteCanvas.begin_path();
  spriteCanvas.arc(static_cast<float>(diameter) / 2.0f, static_cast<float>(diameter) / 2.0f, radiusPx, 0.0f, ebsdlib::constants::k_2PiF);
  spriteCanvas.fill();

  auto sprite = UInt8ArrayType::CreateArray(static_cast<size_t>(diameter) * diameter, {4ULL}, "MarkerSprite", true);
  spriteCanvas.get_image_data(sprite->getPointer(0), diameter, diameter, diameter * 4, 0, 0);
  return sprite;
}
} // namespace ebsdlib
