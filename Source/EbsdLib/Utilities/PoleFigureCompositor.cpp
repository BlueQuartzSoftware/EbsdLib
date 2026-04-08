#include "PoleFigureCompositor.h"

#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Utilities/CanvasUtilities.hpp"
#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"
#include "EbsdLib/Utilities/Fonts.hpp"
#include "EbsdLib/Utilities/PoleFigureUtilities.h"

#include <fmt/format.h>

#include <canvas_ity.hpp>

#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
#include <tbb/task_group.h>
#endif

namespace ebsdlib
{

// -----------------------------------------------------------------------------
CompositePoleFigureResult PoleFigureCompositor::generateCompositeImage(const CompositePoleFigureConfiguration_t& config)
{
  return {};
}

// -----------------------------------------------------------------------------
LayoutMetrics PoleFigureCompositor::computeLayoutMetrics(const CompositePoleFigureConfiguration_t& config)
{
  LayoutMetrics metrics;
  const auto imageWidth = static_cast<float>(config.imageDim);
  const auto imageHeight = static_cast<float>(config.imageDim);
  metrics.fontPtSize = imageHeight / 16.0f;
  metrics.margins = imageHeight / 32.0f;

  // Measure "X" character width using a temporary canvas
  float xCharWidth = 0.0f;
  {
    std::vector<unsigned char> latoBold = fonts::GetLatoBold();
    canvas_ity::canvas tempContext(config.imageDim, config.imageDim);
    tempContext.set_font(latoBold.data(), static_cast<int>(latoBold.size()), metrics.fontPtSize);
    const std::array<char, 2> buf = {'X', 0};
    xCharWidth = tempContext.measure_text(buf.data());
  }

  metrics.subCanvasWidth = metrics.margins + imageWidth + xCharWidth + metrics.margins;
  metrics.subCanvasHeight = metrics.margins + metrics.fontPtSize + imageHeight + metrics.fontPtSize * 2.0f + metrics.margins * 2.0f;

  switch(config.layoutType)
  {
  case PoleFigureLayoutType::Horizontal: {
    metrics.pageWidth = static_cast<int32_t>(metrics.subCanvasWidth) * 4;
    metrics.pageHeight = static_cast<int32_t>(metrics.margins + metrics.fontPtSize + metrics.subCanvasHeight);
    const float y = static_cast<float>(metrics.pageHeight) - metrics.subCanvasHeight;
    metrics.origins[0] = {0.0f, y};
    metrics.origins[1] = {metrics.subCanvasWidth, y};
    metrics.origins[2] = {metrics.subCanvasWidth * 2.0f, y};
    metrics.origins[3] = {metrics.subCanvasWidth * 3.0f, y};
    break;
  }
  case PoleFigureLayoutType::Vertical: {
    metrics.pageWidth = static_cast<int32_t>(metrics.subCanvasWidth);
    metrics.pageHeight = static_cast<int32_t>(metrics.margins + metrics.fontPtSize + metrics.subCanvasHeight * 4.0f);
    const float topY = metrics.margins + metrics.fontPtSize;
    metrics.origins[0] = {0.0f, topY};
    metrics.origins[1] = {0.0f, topY + metrics.subCanvasHeight};
    metrics.origins[2] = {0.0f, topY + metrics.subCanvasHeight * 2.0f};
    metrics.origins[3] = {0.0f, topY + metrics.subCanvasHeight * 3.0f};
    break;
  }
  case PoleFigureLayoutType::Square: {
    metrics.pageWidth = static_cast<int32_t>(metrics.subCanvasWidth) * 2;
    metrics.pageHeight = static_cast<int32_t>(metrics.margins + metrics.fontPtSize + metrics.subCanvasHeight * 2.0f);
    const float topY = static_cast<float>(metrics.pageHeight) - 2.0f * metrics.subCanvasHeight;
    const float bottomY = static_cast<float>(metrics.pageHeight) - metrics.subCanvasHeight;
    metrics.origins[0] = {0.0f, topY};
    metrics.origins[1] = {metrics.subCanvasWidth, topY};
    metrics.origins[2] = {0.0f, bottomY};
    metrics.origins[3] = {metrics.subCanvasWidth, bottomY};
    break;
  }
  }
  return metrics;
}

// -----------------------------------------------------------------------------
std::vector<UInt8ArrayType::Pointer> PoleFigureCompositor::generatePoleFigures(const CompositePoleFigureConfiguration_t& config)
{
  return {};
}

// -----------------------------------------------------------------------------
void PoleFigureCompositor::preprocessImages(std::vector<UInt8ArrayType::Pointer>& images, int imageDim, bool flipFinalImage)
{
}

// -----------------------------------------------------------------------------
UInt8ArrayType::Pointer PoleFigureCompositor::compositeToCanvas(const CompositePoleFigureConfiguration_t& config, const std::vector<UInt8ArrayType::Pointer>& images, const LayoutMetrics& layout)
{
  return nullptr;
}

// -----------------------------------------------------------------------------
void PoleFigureCompositor::drawPoleFigure(canvas_ity::canvas& context, const UInt8ArrayType& image, std::array<float, 2> origin, int imageDim, const std::string& directionLabel, float fontPtSize,
                                          float margins, const std::vector<unsigned char>& latoBold, const std::vector<unsigned char>& firaSans)
{
}

// -----------------------------------------------------------------------------
void PoleFigureCompositor::drawScalarBar(canvas_ity::canvas& context, const CompositePoleFigureConfiguration_t& config, std::array<float, 2> position, float margins, float fontPtSize,
                                         const std::vector<unsigned char>& latoRegular)
{
}

// -----------------------------------------------------------------------------
void PoleFigureCompositor::drawInfoBlock(canvas_ity::canvas& context, const CompositePoleFigureConfiguration_t& config, std::array<float, 2> position, float margins, float fontPtSize,
                                         const std::vector<unsigned char>& latoRegular)
{
}

// -----------------------------------------------------------------------------
void PoleFigureCompositor::drawTitle(canvas_ity::canvas& context, const std::string& title, float pageWidth, float fontPtSize, float margins, const std::vector<unsigned char>& latoBold)
{
}

// -----------------------------------------------------------------------------
UInt8ArrayType::Pointer PoleFigureCompositor::flipAndMirror(UInt8ArrayType* src, int imageDim)
{
  return nullptr;
}

// -----------------------------------------------------------------------------
UInt8ArrayType::Pointer PoleFigureCompositor::convertColorOrder(UInt8ArrayType* src, int imageDim)
{
  return nullptr;
}

} // namespace ebsdlib
