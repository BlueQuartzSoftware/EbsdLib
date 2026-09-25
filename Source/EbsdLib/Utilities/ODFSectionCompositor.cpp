#include "EbsdLib/Utilities/ODFSectionCompositor.h"
#include "EbsdLib/Utilities/ODFSectionChrome.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <canvas_ity.hpp>
#include <fmt/format.h>

namespace ebsdlib
{
namespace
{
ODFSectionResult Failure(int32_t code, std::string message)
{
  ODFSectionResult result;
  result.errorCode = code;
  result.errorMessage = std::move(message);
  return result;
}

std::array<uint8_t, 4> MapColor(double fraction, const std::vector<float>& controls)
{
  fraction = std::clamp(fraction, 0.0, 1.0);
  size_t upper = 0;
  while(upper + 4 < controls.size() && controls[upper] < fraction)
  {
    upper += 4;
  }
  const size_t lower = upper == 0 ? 0 : upper - 4;
  const double weight = upper == lower ? 0.0 : std::clamp((fraction - controls[lower]) / (controls[upper] - controls[lower]), 0.0, 1.0);
  std::array<uint8_t, 4> color = {0, 0, 0, 255};
  for(size_t componentIndex = 0; componentIndex < 3; componentIndex++)
  {
    color[componentIndex] = static_cast<uint8_t>(std::lround(255.0 * ((1.0 - weight) * controls[lower + componentIndex + 1] + weight * controls[upper + componentIndex + 1])));
  }
  return color;
}
} // namespace

ODFSectionLayoutMetrics ComputeODFSectionLayout(const ODFSectionConfiguration& config)
{
  const auto limits = GetODFEulerPlotLimits(config.grid.laueOpsIndex);
  if(config.sectionWidth <= 0 || config.sectionsPerRow == 0 || config.sectionsPerRow > static_cast<size_t>(std::numeric_limits<int32_t>::max()) || config.sectionCount < 2 ||
     config.sectionCount > static_cast<size_t>(std::numeric_limits<int32_t>::max()) || limits.phi1MaxDeg <= 0.0)
  {
    return {};
  }
  ODFSectionLayoutMetrics metrics;
  metrics.panelWidth = config.sectionWidth;
  metrics.panelHeight = static_cast<int32_t>(std::lround(static_cast<double>(config.sectionWidth) * limits.phiMaxDeg / limits.phi1MaxDeg));
  metrics.columns = static_cast<int32_t>(config.sectionsPerRow);
  metrics.rows = static_cast<int32_t>((config.sectionCount + config.sectionsPerRow - 1) / config.sectionsPerRow);
  metrics.fontPtSize = std::max(10.0f, static_cast<float>(config.sectionWidth) / 24.0f);
  metrics.margin = std::max(8.0f, static_cast<float>(config.sectionWidth) / 32.0f);
  metrics.tickFontSize = std::min(metrics.fontPtSize * 0.7f, metrics.margin * 0.8f);
  metrics.leftAxisGutter = ComputeODFLeftAxisGutter(metrics.fontPtSize, metrics.tickFontSize);
  metrics.panelSlotWidth = static_cast<float>(metrics.panelWidth) + metrics.leftAxisGutter + 2.0f * metrics.margin;
  metrics.panelSlotHeight = static_cast<float>(metrics.panelHeight) + 3.0f * metrics.fontPtSize + 3.0f * metrics.margin;
  metrics.titleHeight = metrics.fontPtSize + 2.0f * metrics.margin;
  metrics.legendWidth = std::max(static_cast<float>(config.sectionWidth) / 3.0f, 180.0f);
  const float pageWidth = std::ceil(metrics.columns * metrics.panelSlotWidth + metrics.legendWidth + 2.0f * metrics.margin);
  const float pageHeight = std::ceil(metrics.titleHeight + metrics.rows * metrics.panelSlotHeight + metrics.margin);
  // canvas_ity stores coordinates in unsigned shorts and multiplies page dimensions as signed integers.
  if(metrics.panelHeight < 1 || pageWidth >= 65535.0f || pageHeight >= 65535.0f || static_cast<double>(pageWidth) * pageHeight > std::numeric_limits<int32_t>::max())
  {
    return {};
  }
  metrics.pageWidth = static_cast<int32_t>(pageWidth);
  metrics.pageHeight = static_cast<int32_t>(pageHeight);
  return metrics;
}

ODFSectionResult ODFSectionCompositor::generateCompositeImage(const ODFSectionConfiguration& config) const
{
  if(config.sectionWidth <= 0)
  {
    return Failure(-7510, fmt::format("Section width must be positive; received {}.", config.sectionWidth));
  }
  if(config.sectionsPerRow == 0 || config.sectionsPerRow > static_cast<size_t>(std::numeric_limits<int32_t>::max()))
  {
    return Failure(-7511, fmt::format("Sections per row must be in [1, {}]; received {}.", std::numeric_limits<int32_t>::max(), config.sectionsPerRow));
  }
  if(config.scaleMode != ODFScaleMode::Automatic && config.scaleMode != ODFScaleMode::Manual)
  {
    return Failure(-7512, fmt::format("Unknown ODF scale mode {}.", static_cast<uint8_t>(config.scaleMode)));
  }
  if(config.scaleMode == ODFScaleMode::Manual &&
     (!std::isfinite(config.manualMinimumMUD) || !std::isfinite(config.manualMaximumMUD) || config.manualMinimumMUD < 0.0 || config.manualMinimumMUD >= config.manualMaximumMUD))
  {
    return Failure(-7512, fmt::format("Manual MUD range must satisfy 0 <= minimum < maximum with finite endpoints; received [{}, {}].", config.manualMinimumMUD, config.manualMaximumMUD));
  }
  const auto& controls = config.colorControlPoints;
  if(controls.size() < 8 || controls.size() % 4 != 0)
  {
    return Failure(-7513, fmt::format("At least two complete [position, r, g, b] controls are required; received {} components.", controls.size()));
  }
  for(size_t componentIndex = 0; componentIndex < controls.size(); componentIndex++)
  {
    const float value = controls[componentIndex];
    if(!std::isfinite(value) || value < 0.0f || value > 1.0f || (componentIndex >= 4 && componentIndex % 4 == 0 && value <= controls[componentIndex - 4]))
    {
      return Failure(-7513, fmt::format("Color control component {} is invalid ({}): components must be finite in [0, 1] and positions must increase strictly.", componentIndex, value));
    }
  }
  const auto layout = ComputeODFSectionLayout(config);
  // Reject excessive page requests before preparation allocates the section buffers.
  if(layout.pageWidth == 0 && config.sectionCount >= 2 && GetODFEulerPlotLimits(config.grid.laueOpsIndex).phi1MaxDeg > 0)
  {
    return Failure(-7510, fmt::format("ODF layout exceeds canvas limits or has an empty panel: width {}, sections {}, columns {}.", config.sectionWidth, config.sectionCount, config.sectionsPerRow));
  }
  auto prepared = PrepareODFSections(config.grid, config.sectionCount);
  if(!prepared)
  {
    return Failure(prepared.errorCode, prepared.errorMessage);
  }
  for(size_t valueIndex = 0; valueIndex < prepared.sections.mudValues.size(); valueIndex++)
  {
    if(!std::isfinite(prepared.sections.mudValues[valueIndex]))
    {
      return Failure(-7512,
                     fmt::format("Prepared MUD value {} at section-buffer index {} is not finite; the color scale requires finite values.", prepared.sections.mudValues[valueIndex], valueIndex));
    }
  }
  ODFSectionResult result;
  result.appliedMinimumMUD = config.scaleMode == ODFScaleMode::Automatic ? 0.0 : config.manualMinimumMUD;
  result.appliedMaximumMUD = config.scaleMode == ODFScaleMode::Automatic ? prepared.sections.maximumDisplayedMUD : config.manualMaximumMUD;
  if(result.appliedMaximumMUD <= result.appliedMinimumMUD)
  {
    return Failure(-7512, fmt::format("Automatic MUD maximum must be positive; displayed maximum is {}.", result.appliedMaximumMUD));
  }
  result.width = layout.pageWidth;
  result.height = layout.pageHeight;
  canvas_ity::canvas context(layout.pageWidth, layout.pageHeight);
  context.set_color(canvas_ity::fill_style, 1, 1, 1, 1);
  context.fill_rectangle(0, 0, static_cast<float>(layout.pageWidth), static_cast<float>(layout.pageHeight));
  const auto& sections = prepared.sections;
  std::vector<uint8_t> panel(static_cast<size_t>(layout.panelWidth) * layout.panelHeight * 4);
  const double range = result.appliedMaximumMUD - result.appliedMinimumMUD;
  for(size_t sectionIndex = 0; sectionIndex < config.sectionCount; sectionIndex++)
  {
    // Nearest-cell expansion preserves the prepared values without adding spatial interpolation.
    for(int32_t row = 0; row < layout.panelHeight; row++)
    {
      const size_t phiIndex = static_cast<size_t>(row) * sections.phiCount / static_cast<size_t>(layout.panelHeight);
      for(int32_t column = 0; column < layout.panelWidth; column++)
      {
        const size_t phi1Index = static_cast<size_t>(column) * sections.phi1Count / static_cast<size_t>(layout.panelWidth);
        const double value = sections.mudValues[(sectionIndex * sections.phiCount + phiIndex) * sections.phi1Count + phi1Index];
        const auto color = MapColor((value - result.appliedMinimumMUD) / range, controls);
        const size_t pixelOffset = (static_cast<size_t>(row) * layout.panelWidth + column) * 4;
        std::copy(color.begin(), color.end(), panel.begin() + pixelOffset);
      }
    }
    const auto origin = GetODFSectionPanelOrigin(layout, sectionIndex);
    context.draw_image(panel.data(), layout.panelWidth, layout.panelHeight, layout.panelWidth * 4, origin[0], origin[1], static_cast<float>(layout.panelWidth), static_cast<float>(layout.panelHeight));
  }
  std::vector<uint8_t> colorBar(static_cast<size_t>(layout.panelHeight) * 4);
  for(int32_t row = 0; row < layout.panelHeight; row++)
  {
    const auto color = MapColor(layout.panelHeight == 1 ? 1.0 : 1.0 - static_cast<double>(row) / (layout.panelHeight - 1), controls);
    std::copy(color.begin(), color.end(), colorBar.begin() + static_cast<size_t>(row) * 4);
  }
  DrawODFSectionChrome(context, config, sections, layout, result.appliedMinimumMUD, result.appliedMaximumMUD, colorBar);
  result.image = UInt8ArrayType::CreateArray(static_cast<size_t>(layout.pageWidth) * layout.pageHeight, {4ULL}, "ODFSectionsComposite", true);
  context.get_image_data(result.image->getPointer(0), layout.pageWidth, layout.pageHeight, layout.pageWidth * 4, 0, 0);
  result.sectionAnglesDeg = std::move(prepared.sections.sectionAnglesDeg);
  return result;
}
} // namespace ebsdlib
