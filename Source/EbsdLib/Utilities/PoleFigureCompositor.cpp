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
  return {};
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
