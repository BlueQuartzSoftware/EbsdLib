/* ============================================================================
 * Copyright (c) 2009-2025 BlueQuartz Software, LLC
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of BlueQuartz Software, the US Air Force, nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include "EbsdLib/Utilities/PoleFigureChrome.h"

#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"

#include <canvas_ity.hpp>

#include <fmt/format.h>

#include "EbsdLib/LaueOps/LaueOps.h"

namespace ebsdlib
{

// -----------------------------------------------------------------------------
void DrawPoleFigureBackground(canvas_ity::canvas& context, const LayoutMetrics& layout)
{
  context.move_to(0.0f, 0.0f);
  context.line_to(static_cast<float>(layout.pageWidth), 0.0f);
  context.line_to(static_cast<float>(layout.pageWidth), static_cast<float>(layout.pageHeight));
  context.line_to(0.0f, static_cast<float>(layout.pageHeight));
  context.line_to(0.0f, 0.0f);
  context.close_path();
  context.set_color(canvas_ity::fill_style, 1.0f, 1.0f, 1.0f, 1.0f);
  context.fill();
}

// -----------------------------------------------------------------------------
void DrawPoleFigureTitle(canvas_ity::canvas& context, const std::string& title, float pageWidth, float fontPtSize, float margins, const std::vector<unsigned char>& latoBold)
{
  if(title.empty())
  {
    return;
  }
  context.begin_path();
  context.set_font(const_cast<unsigned char*>(latoBold.data()), static_cast<int>(latoBold.size()), fontPtSize);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.text_baseline = canvas_ity::alphabetic;
  context.fill_text(title.c_str(), margins, margins + fontPtSize);
  context.close_path();
}

// -----------------------------------------------------------------------------
void DrawPoleFigureInfoBlock(canvas_ity::canvas& context, const CompositePoleFigureConfiguration_t& config, std::array<float, 2> position, float margins, float fontPtSize,
                             const std::vector<unsigned char>& latoRegular)
{
  const float scaleBarRelativeWidth = 0.10f;
  const auto imageWidth = static_cast<float>(config.imageDim);
  const float rectWidth = imageWidth * scaleBarRelativeWidth;

  std::vector<std::string> laueNames = LaueOps::GetLaueNames();
  std::string laueGroupName;
  if(config.laueOpsIndex < laueNames.size())
  {
    laueGroupName = laueNames[config.laueOpsIndex];
  }

  std::vector<std::string> labels = {
      fmt::format("Phase Num: {}", config.phaseNumber),
      fmt::format("Material Name: {}", config.phaseName),
      fmt::format("Laue Group: {}", laueGroupName),
      fmt::format("Samples: {}", config.eulers != nullptr ? config.eulers->getNumberOfTuples() : 0),
      fmt::format("Hex/Trig Convention: {}", config.hexConvention == ebsdlib::HexConvention::XParallelAStar ? "x||a*" : "x||a"),
      // fmt::format("{} Right, {} Up", config.axisNames[0], config.axisNames[1])
  };

  if(!config.discrete)
  {
    labels.push_back(fmt::format("Lambert Sq. Dim: {}", config.lambertDim));
  }

  float heightInc = 1.0f;
  for(const auto& label : labels)
  {
    context.begin_path();
    context.set_font(const_cast<unsigned char*>(latoRegular.data()), static_cast<int>(latoRegular.size()), fontPtSize);
    context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
    context.text_baseline = canvas_ity::alphabetic;
    context.fill_text(label.c_str(), position[0] + margins + rectWidth + margins, position[1] + margins + (static_cast<float>(config.imageDim) / 3.0f) + (heightInc * fontPtSize));
    context.close_path();
    heightInc++;
  }
}

// -----------------------------------------------------------------------------
void DrawPoleFigureFrame(canvas_ity::canvas& context, const CompositePoleFigureConfiguration_t& config, std::array<float, 2> origin, const std::string& poleFigureName, float fontPtSize, float margins,
                         const std::vector<unsigned char>& latoBold, const std::vector<unsigned char>& firaSans)
{
  const auto imageSize = static_cast<float>(config.imageDim);

  // Circle outline
  context.begin_path();
  context.line_cap = canvas_ity::circle;
  context.set_line_width(1.0f);
  context.set_color(canvas_ity::stroke_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.arc(origin[0] + margins + imageSize / 2.0f, origin[1] + fontPtSize * 2.0f + margins * 2.0f + imageSize / 2.0f, imageSize / 2.0f, 0, ebsdlib::constants::k_2PiF);
  context.stroke();
  context.close_path();

  // X axis line
  context.begin_path();
  context.line_cap = canvas_ity::square;
  context.set_line_width(1.0f);
  context.set_color(canvas_ity::stroke_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.move_to(origin[0] + margins, origin[1] + fontPtSize * 2.0f + margins * 2.0f + imageSize / 2.0f);
  context.line_to(origin[0] + margins + imageSize, origin[1] + fontPtSize * 2.0f + margins * 2.0f + imageSize / 2.0f);
  context.stroke();
  context.close_path();

  // Y axis line
  context.begin_path();
  context.line_cap = canvas_ity::square;
  context.set_line_width(1.0f);
  context.set_color(canvas_ity::stroke_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.move_to(origin[0] + margins + imageSize / 2.0f, origin[1] + fontPtSize * 2.0f + margins * 2.0f);
  context.line_to(origin[0] + margins + imageSize / 2.0f, origin[1] + fontPtSize * 2.0f + margins * 2.0f + imageSize);
  context.stroke();
  context.close_path();

  // "X" axis label
  context.begin_path();
  context.set_font(const_cast<unsigned char*>(latoBold.data()), static_cast<int>(latoBold.size()), fontPtSize);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.text_baseline = canvas_ity::alphabetic;
  context.fill_text(config.axisNames[0].c_str(), origin[0] + margins * 1.5f + imageSize, origin[1] + fontPtSize * 2.25f + margins * 2.0f + imageSize / 2.0f);
  context.close_path();

  // "Y" axis label
  context.begin_path();
  context.set_font(const_cast<unsigned char*>(latoBold.data()), static_cast<int>(latoBold.size()), fontPtSize);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.text_baseline = canvas_ity::alphabetic;
  const float yFontWidth = context.measure_text(config.axisNames[1].c_str());
  if(config.flipFinalImage)
  {
    context.fill_text(config.axisNames[1].c_str(), origin[0] + margins - (0.5f * yFontWidth) + imageSize / 2.0f, origin[1] + fontPtSize * 1.9f + margins * 2.0f);
  }
  else
  {
    context.fill_text(config.axisNames[1].c_str(), origin[0] + margins - (0.5f * yFontWidth) + imageSize / 2.0f, origin[1] + fontPtSize * 3.0f + margins * 2.0f + imageSize);
  }

  context.close_path();

  // Family label rendered as a parenthesized title, e.g. brace-notation
  // "{0001}" (and any legacy angle-bracket "<0001>") is shown as "(0001)".
  std::string subtitle = EbsdStringUtils::replace(poleFigureName, "{", "(");
  subtitle = EbsdStringUtils::replace(subtitle, "}", ")");
  subtitle = EbsdStringUtils::replace(subtitle, "<", "(");
  subtitle = EbsdStringUtils::replace(subtitle, ">", ")");

  std::string bottomPart;
  std::array<float, 2> textOrigin = {origin[0] + margins, origin[1] + fontPtSize + 2.0f * margins};

  // Handle overbar notation: "-" before a digit draws a line above the digit
  for(size_t idx = 0; idx < subtitle.size(); idx++)
  {
    if(subtitle.at(idx) == '-' && idx + 1 < subtitle.size())
    {
      const char charBuf[] = {subtitle[idx + 1], 0};
      context.set_font(const_cast<unsigned char*>(firaSans.data()), static_cast<int>(firaSans.size()), fontPtSize);
      float tw = 0.0f;
      if(!bottomPart.empty())
      {
        tw = context.measure_text(bottomPart.c_str());
      }
      const float charWidth = context.measure_text(charBuf);
      const float dashWidth = charWidth * 0.5f;
      const float dashOffset = charWidth * 0.25f;

      context.begin_path();
      context.line_cap = canvas_ity::square;
      context.set_line_width(2.0f);
      context.set_color(canvas_ity::stroke_style, 0.0f, 0.0f, 0.0f, 1.0f);
      context.move_to(textOrigin[0] + tw + dashOffset, textOrigin[1] - (0.8f * fontPtSize));
      context.line_to(textOrigin[0] + tw + dashOffset + dashWidth, textOrigin[1] - (0.8f * fontPtSize));
      context.stroke();
      context.close_path();
    }
    else
    {
      bottomPart.push_back(subtitle.at(idx));
    }
  }

  // Draw the direction subtitle text
  context.begin_path();
  context.set_font(const_cast<unsigned char*>(firaSans.data()), static_cast<int>(firaSans.size()), fontPtSize);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.text_baseline = canvas_ity::alphabetic;
  context.fill_text(bottomPart.c_str(), textOrigin[0], textOrigin[1]);
  context.close_path();
}

} // namespace ebsdlib
