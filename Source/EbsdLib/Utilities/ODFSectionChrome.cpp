#include "EbsdLib/Utilities/ODFSectionChrome.h"

#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Utilities/Fonts.hpp"

#include <algorithm>

#include <canvas_ity.hpp>
#include <fmt/format.h>

namespace ebsdlib
{
std::array<float, 2> GetODFSectionPanelOrigin(const ODFSectionLayoutMetrics& layout, size_t sectionIndex)
{
  return {static_cast<float>(sectionIndex % static_cast<size_t>(layout.columns)) * layout.panelSlotWidth + layout.margin,
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

  for(size_t sectionIndex = 0; sectionIndex < sections.sectionAnglesDeg.size(); sectionIndex++)
  {
    const auto origin = GetODFSectionPanelOrigin(layout, sectionIndex);
    const float x = origin[0];
    const float y = origin[1];
    context.set_font(boldFont.data(), static_cast<int>(boldFont.size()), fontSize);
    context.text_align = canvas_ity::start;
    context.fill_text(fmt::format("phi2 = {:g} degrees", sections.sectionAnglesDeg[sectionIndex]).c_str(), x, y - margin, panelWidth);
    context.fill_rectangle(x - 1, y, 1, panelHeight + 1);
    context.fill_rectangle(x, y + panelHeight, panelWidth, 1);

    // PHI increases down the page, matching the prepared section row order.
    const float tickSize = std::min(fontSize * 0.7f, margin * 0.8f);
    context.set_font(tickFont.data(), static_cast<int>(tickFont.size()), tickSize);
    for(int tick = 0; tick <= 2; tick++)
    {
      const float fraction = static_cast<float>(tick) / 2;
      const float tickX = x + panelWidth * fraction;
      const float tickY = y + panelHeight * fraction;
      context.fill_rectangle(tickX - 1, y + panelHeight, 1, 3);
      context.text_align = tick == 0 ? canvas_ity::start : (tick == 2 ? canvas_ity::rightward : canvas_ity::center);
      context.fill_text(fmt::format("{:g} deg", sections.limits.phi1MaxDeg * fraction).c_str(), tickX, y + panelHeight + tickSize + 3, panelWidth / 3);
      context.fill_rectangle(x - 3, tickY, 3, 1);
      context.save();
      context.translate(x - 1, tickY);
      context.rotate(-1.5707963267948966f);
      context.text_align = tick == 0 ? canvas_ity::rightward : (tick == 2 ? canvas_ity::start : canvas_ity::center);
      context.fill_text(fmt::format("{:g} deg", sections.limits.phiMaxDeg * fraction).c_str(), 0, 0, panelHeight / 3);
      context.restore();
    }
    context.set_font(regularFont.data(), static_cast<int>(regularFont.size()), fontSize);
    context.text_align = canvas_ity::center;
    context.fill_text("phi1", x + panelWidth / 2, y + panelHeight + 2 * fontSize + margin / 2, panelWidth);
    context.text_align = canvas_ity::start;
    context.fill_text("PHI", x, y + panelHeight + 2 * fontSize + margin / 2, panelWidth / 4);
  }

  const float legendX = static_cast<float>(layout.columns) * layout.panelSlotWidth + 2 * margin;
  const float legendY = layout.titleHeight + fontSize + margin;
  const float barWidth = 2 * margin;
  context.draw_image(colorBar.data(), 1, layout.panelHeight, 4, legendX, legendY, barWidth, panelHeight);
  context.set_font(boldFont.data(), static_cast<int>(boldFont.size()), fontSize);
  context.fill_text("MUD", legendX, legendY - margin, layout.legendWidth - margin);
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
