#include "EbsdLib/Utilities/ODFSectionChrome.h"

#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Utilities/Fonts.hpp"

#include <algorithm>
#include <cmath>

#include <canvas_ity.hpp>
#include <fmt/format.h>

namespace ebsdlib
{
std::string FormatODFSectionTitle(double angleDeg)
{
  return fmt::format("\xCF\x86\xE2\x82\x82 = {:g}\xC2\xB0", angleDeg);
}

std::string GetODFHorizontalAxisTitle()
{
  return "\xCF\x86\xE2\x82\x81";
}

std::string GetODFVerticalAxisTitle()
{
  return "\xCE\xA6";
}

double SelectODFAxisTickInterval(double maximumDeg, int32_t pixelLength)
{
  const double tenDegreePixelSpacing = static_cast<double>(pixelLength) / (maximumDeg / 10.0);
  return tenDegreePixelSpacing >= 24.0 ? 10.0 : 20.0;
}

std::vector<double> GenerateODFAxisTicks(double maximumDeg, int32_t pixelLength)
{
  const double intervalDeg = SelectODFAxisTickInterval(maximumDeg, pixelLength);
  std::vector<double> ticks;
  for(double angleDeg = 0.0; angleDeg < maximumDeg; angleDeg += intervalDeg)
  {
    ticks.push_back(angleDeg);
  }
  if(ticks.empty() || ticks.back() != maximumDeg)
  {
    ticks.push_back(maximumDeg);
  }
  return ticks;
}

std::vector<double> GenerateODFAxisLabelTicks(double maximumDeg, int32_t pixelLength, float fontSize)
{
  if(fontSize <= 0.0f)
  {
    fontSize = std::min(std::max(10.0f, pixelLength / 24.0f) * 0.7f, std::max(8.0f, pixelLength / 32.0f) * 0.8f);
  }
  const auto font = fonts::GetFiraSansRegular();
  canvas_ity::canvas context(1, 1);
  context.set_font(font.data(), static_cast<int>(font.size()), fontSize);
  const auto ticks = GenerateODFAxisTicks(maximumDeg, pixelLength);
  std::vector<double> labels = {ticks.front()};
  float previousRight = context.measure_text("0");
  const float endpointLeft = pixelLength - context.measure_text(fmt::format("{:g}", maximumDeg).c_str());
  constexpr float padding = 4.0f;
  if(endpointLeft < previousRight + padding)
  {
    return labels;
  }
  for(size_t tickIndex = 1; tickIndex + 1 < ticks.size(); tickIndex++)
  {
    const double angleDeg = ticks[tickIndex];
    // Fira's numeric advance widths include the side bearings of each rendered digit.
    const float width = context.measure_text(fmt::format("{:g}", angleDeg).c_str());
    const float center = static_cast<float>(angleDeg / maximumDeg * pixelLength);
    const float left = center - width / 2;
    const float right = center + width / 2;
    if(left >= previousRight + padding && right + padding <= endpointLeft)
    {
      labels.push_back(angleDeg);
      previousRight = right;
    }
  }
  if(ticks.back() != labels.back())
  {
    labels.push_back(ticks.back());
  }
  return labels;
}

float ComputeODFLeftAxisGutter(float fontSize, float tickSize)
{
  // Measure ink at a fixed font size so layout checks cannot allocate an oversized canvas.
  const auto font = fonts::GetFiraSansRegular();
  canvas_ity::canvas context(640, 192);
  context.set_font(font.data(), static_cast<int>(font.size()), 64.0f);
  context.set_color(canvas_ity::fill_style, 0, 0, 0, 1);
  context.fill_text("\xCE\xA6", 16, 96);
  context.fill_text("0123456789", 128, 96);
  std::vector<uint8_t> pixels(640 * 192 * 4);
  context.get_image_data(pixels.data(), 640, 192, 640 * 4, 0, 0);
  int32_t titleBottom = 96;
  int32_t tickTop = 96;
  for(int32_t y = 0; y < 192; ++y)
  {
    for(int32_t x = 0; x < 640; ++x)
    {
      if(pixels[(y * 640 + x) * 4 + 3] != 0)
      {
        if(x < 128)
        {
          titleBottom = std::max(titleBottom, y + 1);
        }
        else
        {
          tickTop = std::min(tickTop, y);
        }
      }
    }
  }
  // Rotation maps font ascent to the left of each baseline. Padding also covers pixel rounding.
  return std::ceil(fontSize + (titleBottom - 96) * fontSize / 64.0f + (96 - tickTop) * tickSize / 64.0f + 10.0f);
}

std::string GetODFColorBarTitle(ODFValueUnits sourceUnits)
{
  return sourceUnits == ODFValueUnits::MUD ? "MUD" : "MUD (from Count-Density)";
}

std::array<float, 2> GetODFSectionPanelOrigin(const ODFSectionLayoutMetrics& layout, size_t sectionIndex)
{
  return {static_cast<float>(sectionIndex % static_cast<size_t>(layout.columns)) * layout.panelSlotWidth + layout.margin + layout.leftAxisGutter,
          layout.titleHeight + static_cast<float>(sectionIndex / static_cast<size_t>(layout.columns)) * layout.panelSlotHeight + layout.fontPtSize + layout.margin};
}

void DrawODFSectionChrome(canvas_ity::canvas& context, const ODFSectionConfiguration& config, const PreparedODFSections& sections, const ODFSectionLayoutMetrics& layout, double minimumMUD,
                          double maximumMUD, const std::vector<uint8_t>& colorBar)
{
  auto boldFont = fonts::GetLatoBold();
  auto regularFont = fonts::GetLatoRegular();
  auto tickFont = fonts::GetFiraSansRegular();
  const float fontSize = layout.fontPtSize;
  const float margin = layout.margin;
  const float panelWidth = static_cast<float>(layout.panelWidth);
  const float panelHeight = static_cast<float>(layout.panelHeight);
  context.set_color(canvas_ity::fill_style, 0, 0, 0, 1);
  context.text_baseline = canvas_ity::alphabetic;
  context.set_font(boldFont.data(), static_cast<int>(boldFont.size()), fontSize);
  context.fill_text(config.title.c_str(), margin, margin + fontSize, static_cast<float>(layout.pageWidth) - 2 * margin);

  // Fira Sans contains the Greek letters and subscripts that the embedded Lato fonts lack.
  for(size_t sectionIndex = 0; sectionIndex < sections.sectionAnglesDeg.size(); sectionIndex++)
  {
    const auto origin = GetODFSectionPanelOrigin(layout, sectionIndex);
    const float x = origin[0];
    const float y = origin[1];
    context.set_font(tickFont.data(), static_cast<int>(tickFont.size()), fontSize);
    context.text_align = canvas_ity::start;
    context.fill_text(FormatODFSectionTitle(sections.sectionAnglesDeg[sectionIndex]).c_str(), x, y - margin, panelWidth);
    context.fill_rectangle(x - 1, y, 1, panelHeight + 1);
    context.fill_rectangle(x, y + panelHeight, panelWidth, 1);

    // PHI increases down the page, matching the prepared section row order.
    const float tickSize = layout.tickFontSize;
    context.set_font(tickFont.data(), static_cast<int>(tickFont.size()), tickSize);
    const auto phi1Ticks = GenerateODFAxisTicks(sections.limits.phi1MaxDeg, layout.panelWidth);
    const auto phi1LabelTicks = GenerateODFAxisLabelTicks(sections.limits.phi1MaxDeg, layout.panelWidth, tickSize);
    for(const double tickDeg : phi1Ticks)
    {
      const float fraction = static_cast<float>(tickDeg / sections.limits.phi1MaxDeg);
      const float tickX = x + panelWidth * fraction;
      context.fill_rectangle(tickX - 1, y + panelHeight, 1, 3);
      if(std::binary_search(phi1LabelTicks.begin(), phi1LabelTicks.end(), tickDeg))
      {
        context.text_align = tickDeg == 0.0 ? canvas_ity::start : (tickDeg == sections.limits.phi1MaxDeg ? canvas_ity::rightward : canvas_ity::center);
        context.fill_text(fmt::format("{:g}", tickDeg).c_str(), tickX, y + panelHeight + tickSize + 3, panelWidth / 3);
      }
    }
    const auto phiTicks = GenerateODFAxisTicks(sections.limits.phiMaxDeg, layout.panelHeight);
    const auto phiLabelTicks = GenerateODFAxisLabelTicks(sections.limits.phiMaxDeg, layout.panelHeight, tickSize);
    for(const double tickDeg : phiTicks)
    {
      const float fraction = static_cast<float>(tickDeg / sections.limits.phiMaxDeg);
      const float tickY = y + panelHeight * fraction;
      context.fill_rectangle(x - 3, tickY, 3, 1);
      if(!std::binary_search(phiLabelTicks.begin(), phiLabelTicks.end(), tickDeg))
      {
        continue;
      }
      context.save();
      context.translate(x - 4, tickY);
      context.rotate(-1.5707963267948966f);
      context.text_align = tickDeg == 0.0 ? canvas_ity::rightward : (tickDeg == sections.limits.phiMaxDeg ? canvas_ity::start : canvas_ity::center);
      context.fill_text(fmt::format("{:g}", tickDeg).c_str(), 0, 0, panelHeight / 3);
      context.restore();
    }
    context.set_font(tickFont.data(), static_cast<int>(tickFont.size()), fontSize);
    context.text_align = canvas_ity::center;
    context.fill_text(GetODFHorizontalAxisTitle().c_str(), x + panelWidth / 2, y + panelHeight + 2 * fontSize + margin / 2, panelWidth);
    context.save();
    context.translate(std::round(x - layout.leftAxisGutter + fontSize), std::round(y + panelHeight / 2));
    context.rotate(-1.5707963267948966f);
    context.fill_text(GetODFVerticalAxisTitle().c_str(), 0, 0, panelHeight);
    context.restore();
  }

  const float legendX = static_cast<float>(layout.columns) * layout.panelSlotWidth + 2 * margin;
  const float legendY = layout.titleHeight + fontSize + margin;
  const float barWidth = 2 * margin;
  context.draw_image(colorBar.data(), 1, layout.panelHeight, 4, legendX, legendY, barWidth, panelHeight);
  context.set_font(boldFont.data(), static_cast<int>(boldFont.size()), fontSize);
  context.text_align = canvas_ity::start;
  context.fill_text(GetODFColorBarTitle(config.grid.units).c_str(), legendX, legendY - margin, layout.legendWidth - margin);
  const float textX = legendX + barWidth + margin;
  const float textWidth = static_cast<float>(layout.pageWidth) - textX - margin;
  context.set_font(regularFont.data(), static_cast<int>(regularFont.size()), fontSize);
  context.fill_text(fmt::format("{:g}", maximumMUD).c_str(), textX, legendY + fontSize / 2, textWidth);
  context.fill_text(fmt::format("{:g}", minimumMUD).c_str(), legendX, legendY + panelHeight + fontSize + margin / 2, barWidth);

  const auto laueNames = LaueOps::GetLaueNames();
  const std::array<std::string, 7> labels = {fmt::format("Phase: {}", config.phaseNumber),
                                             fmt::format("Material: {}", config.materialName),
                                             fmt::format("Laue: {}", laueNames[config.grid.laueOpsIndex]),
                                             fmt::format("Input: {}", config.grid.units == ODFValueUnits::MUD ? "MUD" : "CountDensity"),
                                             "Rendered: MUD",
                                             fmt::format("Bins: {:g}, {:g}, {:g} deg", config.grid.spacingDeg[0], config.grid.spacingDeg[1], config.grid.spacingDeg[2]),
                                             fmt::format("Sections: {}", sections.sectionAnglesDeg.size())};
  const float infoSize = std::min(fontSize, (static_cast<float>(layout.pageHeight) - margin - legendY - fontSize) / 8);
  context.set_font(regularFont.data(), static_cast<int>(regularFont.size()), infoSize);
  for(size_t labelIndex = 0; labelIndex < labels.size(); labelIndex++)
  {
    context.fill_text(labels[labelIndex].c_str(), textX, legendY + fontSize + static_cast<float>(labelIndex + 1) * infoSize, textWidth);
  }
}
} // namespace ebsdlib
