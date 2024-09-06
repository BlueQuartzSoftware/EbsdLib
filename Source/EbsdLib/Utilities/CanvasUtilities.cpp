
#include "CanvasUtilities.hpp"


#define CANVAS_ITY_IMPLEMENTATION
#include <canvas_ity.hpp>

namespace EbsdLib
{

// -----------------------------------------------------------------------------
void WriteText(canvas_ity::canvas& context, const std::string& figureSubtitle, std::array<float, 2> textOrigin, int fontPtSize)
{
  std::string bottomPart;
  //std::array<float, 2> textOrigin = {figureOrigin[0] + margins, figureOrigin[1] + fontPtSize + 2 * margins};
  for(size_t idx = 0; idx < figureSubtitle.size(); idx++)
  {
    if(figureSubtitle.at(idx) == '-')
    {
      const char charBuf[] = {figureSubtitle[idx + 1], 0};
      //      context.set_font(m_FiraSansRegular.data(), static_cast<int>(m_FiraSansRegular.size()), fontPtSize);
      float tw = 0.0f;
      if(!bottomPart.empty())
      {
        tw = context.measure_text(bottomPart.c_str());
      }
      const float charWidth = context.measure_text(charBuf);
      const float dashWidth = charWidth * 0.6f;
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
      bottomPart.push_back(figureSubtitle.at(idx));
    }
  }
  // Draw the Direction subtitle text
  context.begin_path();
  //context.set_font(m_FiraSansRegular.data(), static_cast<int>(m_FiraSansRegular.size()), fontPtSize);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
  //context.text_baseline = baselines[0];
  context.fill_text(bottomPart.c_str(), textOrigin[0], textOrigin[1]);
  context.close_path();
}


// -----------------------------------------------------------------------------
void DrawLine(canvas_ity::canvas& context, float xStart, float yStart, float xEnd, float yEnd)
{
  context.begin_path();
  context.line_cap = canvas_ity::square;
  //context.set_line_width(1.0f);
  context.set_color(canvas_ity::stroke_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.move_to(xStart, yStart);
  context.line_to(xEnd, yEnd);
  context.stroke();
  context.close_path();
}


}
