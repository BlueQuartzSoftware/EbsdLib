#include "EbsdLib/Utilities/DiscretePoleFigureCompositor.h"

#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Utilities/Fonts.hpp"
#include "EbsdLib/Utilities/MarkerOccupancyGrid.h"
#include "EbsdLib/Utilities/PoleFigureChrome.h"
#include "EbsdLib/Utilities/PoleFigureCompositor.h"
#include "EbsdLib/Utilities/PoleFigureProjection.h"

#include <canvas_ity.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

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

// -----------------------------------------------------------------------------
CompositePoleFigureResult DiscretePoleFigureCompositor::generateCompositeImage(CompositePoleFigureConfiguration_t& config)
{
  CompositePoleFigureResult result;

  std::vector<LaueOps::Pointer> ops = LaueOps::GetAllOrientationOps();
  if(config.laueOpsIndex >= ops.size())
  {
    return result;
  }
  LaueOps::Pointer op = ops[config.laueOpsIndex];

  // --- Crystallography: symmetric pole vectors for the 3 families ---
  const std::array<int32_t, 3> symSizes = op->getNumSymmetry();
  const size_t numOrientations = (config.eulers != nullptr) ? config.eulers->getNumberOfTuples() : 0;
  std::vector<size_t> dims(1, 3);
  std::array<FloatArrayType::Pointer, 3> families = {FloatArrayType::CreateArray(numOrientations * static_cast<size_t>(symSizes[0]), dims, "family0", true),
                                                     FloatArrayType::CreateArray(numOrientations * static_cast<size_t>(symSizes[1]), dims, "family1", true),
                                                     FloatArrayType::CreateArray(numOrientations * static_cast<size_t>(symSizes[2]), dims, "family2", true)};
  if(numOrientations > 0)
  {
    op->generateSphereCoordsFromEulers(config.eulers, families[0].get(), families[1].get(), families[2].get(), config.hexConvention);
  }

  // --- Layout (shared with the raster compositor) ---
  const LayoutMetrics layout = PoleFigureCompositor::computeLayoutMetrics(config);

  const std::array<std::string, 3> defaultNames = op->getDefaultPoleFigureNames(config.hexConvention);
  auto nameFor = [&](int familyIndex) -> std::string {
    if(config.poleFigureNames.size() > static_cast<size_t>(familyIndex))
    {
      return config.poleFigureNames[static_cast<size_t>(familyIndex)];
    }
    return defaultNames[static_cast<size_t>(familyIndex)];
  };

  // --- Fonts ---
  std::vector<unsigned char> latoBold = fonts::GetLatoBold();
  std::vector<unsigned char> latoRegular = fonts::GetLatoRegular();
  std::vector<unsigned char> firaSans = fonts::GetFiraSansRegular();

  // --- Marker sprite (single color => render once) ---
  const float halfSize = static_cast<float>(config.imageDim) / 2.0f;
  const float markerR = std::max(0.5f, config.markerStyle.radiusFraction * static_cast<float>(config.imageDim));
  const float cellSize = 1.0f; // std::max(1.0f, 2.0f * markerR);
  int spriteSize = 0;
  UInt8ArrayType::Pointer sprite = RenderDiscreteMarkerSprite(config.markerStyle.color, markerR, spriteSize);
  const float spriteHalf = static_cast<float>(spriteSize) / 2.0f;

  // --- Render ---
  canvas_ity::canvas context(layout.pageWidth, layout.pageHeight);
  DrawPoleFigureBackground(context, layout);
  DrawPoleFigureTitle(context, config.title, static_cast<float>(layout.pageWidth), layout.fontPtSize, layout.margins, latoBold);

  for(int familyIndex = 0; familyIndex < 3; familyIndex++)
  {
    const int slot = (config.order.size() == 3) ? static_cast<int>(config.order[static_cast<size_t>(familyIndex)]) : familyIndex;
    const std::array<float, 2> origin = layout.origins[static_cast<size_t>(slot)];
    const float x0 = origin[0] + layout.margins;
    const float y0 = origin[1] + layout.fontPtSize * 2.0f + layout.margins * 2.0f;

    MarkerOccupancyGrid grid(x0, y0, static_cast<float>(config.imageDim), static_cast<float>(config.imageDim), cellSize);

    const FloatArrayType::Pointer fam = families[static_cast<size_t>(familyIndex)];
    const size_t numPoles = fam->getNumberOfTuples();
    for(size_t t = 0; t < numPoles; t++)
    {
      // Do Not Draw Negative Hemisphere.
      if((*fam)[t * 3 + 2] < 0)
      {
        continue;
      }
      const std::array<float, 2> disk = StereographicProjectUpperHemisphere((*fam)[t * 3 + 0], (*fam)[t * 3 + 1], (*fam)[t * 3 + 2]);
      if(disk[0] * disk[0] + disk[1] * disk[1] > 1.0f)
      {
        continue; // outside the unit disk
      }
      const float cx = x0 + halfSize + disk[0] * halfSize;
      // flipFinalImage=true => +Y up (mirror of raw projection, matching the raster path's flip);
      // flipFinalImage=false => +Y down (raw projection orientation, matching the unflipped raster path).
      const float cy = config.flipFinalImage ? (y0 + halfSize - disk[1] * halfSize) : (y0 + halfSize + disk[1] * halfSize);
      if(!grid.shouldDraw(cx, cy))
      {
        continue; // decimated: a marker already occupies this cell
      }
      context.draw_image(sprite->getPointer(0), spriteSize, spriteSize, spriteSize * 4, cx - spriteHalf, cy - spriteHalf, static_cast<float>(spriteSize), static_cast<float>(spriteSize));
    }

    DrawPoleFigureFrame(context, config, origin, nameFor(familyIndex), layout.fontPtSize, layout.margins, latoBold, firaSans);
  }

  const float legendFontPtSize = static_cast<float>(config.imageDim) / 20.0f;
  DrawPoleFigureInfoBlock(context, config, layout.origins[3], layout.margins, legendFontPtSize, latoRegular);

  auto image = UInt8ArrayType::CreateArray(static_cast<size_t>(layout.pageWidth) * layout.pageHeight, {4ULL}, "CompositePoleFigure", true);
  context.get_image_data(image->getPointer(0), layout.pageWidth, layout.pageHeight, layout.pageWidth * 4, 0, 0);

  result.image = image;
  result.width = layout.pageWidth;
  result.height = layout.pageHeight;
  return result;
}

// -----------------------------------------------------------------------------
CompositePoleFigureResult GeneratePoleFigureComposite(CompositePoleFigureConfiguration_t& config)
{
  if(config.discrete && !config.discreteHeatMap)
  {
    DiscretePoleFigureCompositor compositor;
    return compositor.generateCompositeImage(config);
  }
  PoleFigureCompositor compositor;
  return compositor.generateCompositeImage(config);
}
} // namespace ebsdlib
