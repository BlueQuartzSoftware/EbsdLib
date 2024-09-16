
#include "CanvasUtilities.hpp"

#include "EbsdLib/Utilities/FiraSansRegular.hpp"
#include "EbsdLib/Utilities/Fonts.hpp"
#include "EbsdLib/Utilities/LatoBold.hpp"
#include "EbsdLib/Utilities/LatoRegular.hpp"
#include "EbsdLib/Math/EbsdLibMath.h"
#include "EbsdLib/Utilities/ComputeStereographicProjection.h"
#include "EbsdLib/Utilities/EbsdStringUtils.hpp"


#define CANVAS_ITY_IMPLEMENTATION
#include <canvas_ity.hpp>

namespace EbsdLib
{

// -----------------------------------------------------------------------------
void WriteText(canvas_ity::canvas& context, const std::string& figureSubtitle,
               std::array<float, 2> textOrigin, int fontPtSize)
{
  std::string bottomPart;
  // std::array<float, 2> textOrigin = {figureOrigin[0] + margins, figureOrigin[1] + fontPtSize + 2 * margins};
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
  // context.set_font(m_FiraSansRegular.data(), static_cast<int>(m_FiraSansRegular.size()), fontPtSize);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
  // context.text_baseline = baselines[0];
  context.fill_text(bottomPart.c_str(), textOrigin[0], textOrigin[1]);
  context.close_path();
}

// -----------------------------------------------------------------------------
void DrawLine(canvas_ity::canvas& context, float xStart, float yStart, float xEnd, float yEnd)
{
  context.begin_path();
  context.line_cap = canvas_ity::square;
  // context.set_line_width(1.0f);
  // context.set_color(canvas_ity::stroke_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.move_to(xStart, yStart);
  context.line_to(xEnd, yEnd);
  context.stroke();
  context.close_path();
}

// -----------------------------------------------------------------------------
// Function to generate points on a unit circle on the plane
std::vector<Point3DType> GeneratePointsOnUnitCircle(const Point3DType& direction, int num_points)
{
  std::vector<Point3DType> points;
  // Normalize the normal vector (A, B, C)
  Point3DType dirNormalized = direction.normalize();

  // Find a basis vector (v1) on the plane by choosing an arbitrary perpendicular vector
  Point3DType v1;
  if(dirNormalized[0] != 0 || dirNormalized[1] != 0)
  {
    v1[0] = -dirNormalized[1];
    v1[1] = dirNormalized[0];
    v1[2] = 0;
  }
  else
  {
    v1[0] = 1;
    v1[1] = 0;
    v1[2] = 0;
  }

  // Normalize v1
  v1 = v1.normalize();

  // Find another vector (v2) that is perpendicular to both the normal and v1 using the cross product
  Point3DType v2 = dirNormalized.cross(v1).normalize();
  double angleStart = 0.0 * EbsdLib::Constants::k_PiOver180D;
  double arc = 360.0 * EbsdLib::Constants::k_PiOver180D;

  // Generate points on the unit circle that has been rotated according to the direction
  for(int i = 0; i < num_points + 1; ++i)
  {
    float theta = angleStart + arc * i / num_points;

    Point3DType point = {
        std::cos(theta) * v1[0] + std::sin(theta) * v2[0],
        std::cos(theta) * v1[1] + std::sin(theta) * v2[1],
        std::cos(theta) * v1[2] + std::sin(theta) * v2[2],
    };

    points.push_back(point);
  }

  return points;
}


// -----------------------------------------------------------------------------
EbsdLib::UInt8ArrayType::Pointer DrawStandardCubicProjection(EbsdLib::UInt8ArrayType::Pointer image, int pageWidth, int pageHeight)
{
  std::vector<unsigned char> m_FiraSansRegular;
  std::vector<unsigned char> m_LatoRegular;
  std::vector<unsigned char> m_LatoBold;
  // Initialize our fonts
  fonts::Base64Decode(fonts::k_FiraSansRegularBase64, m_FiraSansRegular);
  fonts::Base64Decode(fonts::k_LatoRegularBase64, m_LatoRegular);
  fonts::Base64Decode(fonts::k_LatoBoldBase64, m_LatoBold);
  const float fontPtSize = static_cast<float>(pageWidth) / 24.0f;

  // Create a Canvas to draw into
  canvas_ity::canvas context(pageWidth, pageHeight);

  context.set_font(m_LatoBold.data(), static_cast<int>(m_LatoBold.size()), fontPtSize);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
  canvas_ity::baseline_style const baselines[] = {canvas_ity::alphabetic, canvas_ity::top, canvas_ity::middle, canvas_ity::bottom, canvas_ity::hanging, canvas_ity::ideographic};
  context.text_baseline = baselines[0];

  // Fill the whole background with white
  context.move_to(0.0f, 0.0f);
  context.line_to(static_cast<float>(pageWidth), 0.0f);
  context.line_to(static_cast<float>(pageWidth), static_cast<float>(pageHeight));
  context.line_to(0.0f, static_cast<float>(pageHeight));
  context.line_to(0.0f, 0.0f);
  context.close_path();
  context.set_color(canvas_ity::fill_style, 1.0f, 1.0f, 1.0f, 1.0f);
  context.fill();

  context.draw_image(image->getPointer(0), pageWidth, pageHeight, pageWidth * image->getNumberOfComponents(), 0, 0, static_cast<float>(pageWidth), static_cast<float>(pageHeight));

  float penWidth = 1.0f;
  context.set_color(canvas_ity::stroke_style, 0.25f, 0.25f, 0.25f, 1.0f);
  context.set_line_width(penWidth);

  context.begin_path();
  context.line_cap = canvas_ity::square;
  context.move_to(pageWidth * 0.5F, pageHeight * 0.5F);
  context.arc(pageWidth * 0.5F, pageHeight * 0.5F, pageWidth * 0.5F, 0.0, 2.0 * M_PI);
  context.stroke();
  context.close_path();


  int num_points = 50;
  std::vector<EbsdLib::Point3DType> directions = {
      {1.0, 0.0, 1.0},  // Horizontal Meridian Line
//      {2.0, 0.0, 1.0},
//      {1.0, 0.0, 2.0},
      {0.0, 1.0, 1.0},  // Vertical Meridian Line
//      {0.0, 2.0, 1.0},
//      {0.0, 1.0, 2.0},
      {-1.0, 1.0, 0.0}, // Upper Left to Lower Right
      {1.0, 1.0, 0.0},  // Upper Right to Lower Left
      {1.0, 0.0, 0.0},  // Vertical Axis
      {0.0, 1.0, 0.0},  // Horizontal Axis
  };

  int halfWidth = pageWidth * 0.5F;
  int halfHeight = pageHeight * 0.5F;
  std::array<float, 2> figureOrigin = {0.0F, 0.0F};
  for(const auto& direction : directions)
  {
    std::vector<EbsdLib::Point3DType> stereoPoints = Stereographic::Utils::TransformUnitSphereToStereographicCoords(EbsdLib::GeneratePointsOnUnitCircle(direction, num_points));

    for(size_t i = 1; i < stereoPoints.size(); i++)
    {
      EbsdLib::Point3DType p0 = stereoPoints[i - 1];
      EbsdLib::Point3DType p1 = stereoPoints[i];
      p0 = (p0 * static_cast<float>(halfWidth)) + halfWidth;
      p1 = (p1 * static_cast<float>(halfWidth)) + halfWidth;

      p0[0] = p0[0] + figureOrigin[0];
      p0[1] = p0[1] + figureOrigin[1];

      p1[0] = p1[0] + figureOrigin[0];
      p1[1] = p1[1] + figureOrigin[1];

      EbsdLib::DrawLine(context, p0[0], p0[1], p1[0], p1[1]);
    }
  }

  // This is PRE ROTATION of the entire image, so +X is to the RIGHT, +Y is UP
  std::string fontWidthString = "[100]";
  float fontWidth = context.measure_text(fontWidthString.c_str());
  EbsdLib::WriteText(context, fontWidthString, {pageWidth - fontWidth, static_cast<float>(halfHeight)}, fontPtSize );
  EbsdLib::WriteText(context, "[010]", {pageWidth * 0.5F, static_cast<float>(fontPtSize * 1.2F)}, fontPtSize );
  EbsdLib::WriteText(context, "[110]", {pageWidth * 0.85F, pageHeight * 0.15F}, fontPtSize );
  EbsdLib::WriteText(context, "[-110]", {pageWidth * 0.15F, pageHeight * 0.15F}, fontPtSize );
  EbsdLib::WriteText(context, "[-100]", {pageWidth * 0.0F, pageHeight * 0.5F}, fontPtSize );
  EbsdLib::WriteText(context, "[-1-10]", {pageWidth * 0.15F, pageHeight * 0.85F}, fontPtSize );
  EbsdLib::WriteText(context, "[0-10]", {pageWidth * 0.5F, pageHeight - fontPtSize * 1.1F}, fontPtSize );
  EbsdLib::WriteText(context, "[1-10]", {pageWidth * 0.85F, pageHeight * 0.85F}, fontPtSize );

  // Fetch the rendered RGBA pixels from the entire canvas.
  EbsdLib::UInt8ArrayType::Pointer rgbaCanvasImage = EbsdLib::UInt8ArrayType::CreateArray(pageHeight * pageWidth, {4ULL}, "Triangle Legend", true);
  // std::vector<unsigned char> rgbaCanvasImage(static_cast<size_t>(pageHeight * pageWidth * 4));
  context.get_image_data(rgbaCanvasImage->getPointer(0), pageWidth, pageHeight, pageWidth * 4, 0, 0);

  rgbaCanvasImage = EbsdLib::RemoveAlphaChannel(rgbaCanvasImage.get());
  return rgbaCanvasImage;
}


// -----------------------------------------------------------------------------
EbsdLib::UInt8ArrayType::Pointer DrawStandardHexagonalProjection(EbsdLib::UInt8ArrayType::Pointer image, int pageWidth, int pageHeight)
{
  int legendHeight = pageHeight;
  int legendWidth = pageWidth;

  if(legendHeight > legendWidth)
  {
    legendHeight = legendWidth;
  }
  else
  {
    legendWidth = legendHeight;
  }

  int halfWidth = legendWidth / 2;
  int halfHeight = legendHeight / 2;

  std::array<float, 2> figureOrigin = {0.0F, 0.0F};
      std::array<float, 2> figureCenter = {pageWidth * 0.5F, pageHeight * 0.5F};


  std::vector<unsigned char> m_FiraSansRegular;
  std::vector<unsigned char> m_LatoRegular;
  std::vector<unsigned char> m_LatoBold;
  // Initialize our fonts
  fonts::Base64Decode(fonts::k_FiraSansRegularBase64, m_FiraSansRegular);
  fonts::Base64Decode(fonts::k_LatoRegularBase64, m_LatoRegular);
  fonts::Base64Decode(fonts::k_LatoBoldBase64, m_LatoBold);
  const float fontPtSize = static_cast<float>(pageWidth) / 24.0f;

  // Create a Canvas to draw into
  canvas_ity::canvas context(pageWidth, pageHeight);

  context.set_font(m_LatoBold.data(), static_cast<int>(m_LatoBold.size()), fontPtSize);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
  canvas_ity::baseline_style const baselines[] = {canvas_ity::alphabetic, canvas_ity::top, canvas_ity::middle, canvas_ity::bottom, canvas_ity::hanging, canvas_ity::ideographic};
  context.text_baseline = baselines[0];

  // Fill the whole background with white
  context.move_to(0.0f, 0.0f);
  context.line_to(static_cast<float>(pageWidth), 0.0f);
  context.line_to(static_cast<float>(pageWidth), static_cast<float>(pageHeight));
  context.line_to(0.0f, static_cast<float>(pageHeight));
  context.line_to(0.0f, 0.0f);
  context.close_path();
  context.set_color(canvas_ity::fill_style, 1.0f, 1.0f, 1.0f, 1.0f);
  context.fill();

  context.draw_image(image->getPointer(0), pageWidth, pageHeight, pageWidth * image->getNumberOfComponents(), 0, 0, static_cast<float>(pageWidth), static_cast<float>(pageHeight));

  float penWidth = 1.0f;
  context.set_color(canvas_ity::stroke_style, 0.25f, 0.25f, 0.25f, 1.0f);
  context.set_line_width(penWidth);

  context.begin_path();
  context.line_cap = canvas_ity::square;
  context.move_to(pageWidth * 0.5F, pageHeight * 0.5F);
  context.arc(pageWidth * 0.5F, pageHeight * 0.5F, pageWidth * 0.5F, 0.0, 2.0 * M_PI);
  context.stroke();
  context.close_path();

  std::vector<float> angles = {0.0f, 30.0f, 60.0f, 90.0f, 120.0f, 150.0f, 180.0f, 210.0f, 240.0f, 270.0f, 300.0f, 330.0f};
  std::vector<std::string> labels2 = {"[2-1-10]", "[10-10]", "[11-20]", "[01-10]", "[-12-10]", "[-1100]", "[-2110]", "[-1010]", "[-1-120]", "[0-110]", "[1-210]", "[1-100]"};

  std::vector<float> xAdj = {
     -1.0F,  -0.5F, 0.0F, -0.5F, -1.0F, -0.5F, 0.0F, -0.5F, -1.0F, -0.5F, 0.0F, -0.5F,
  };
  std::vector<float> yAdj = {
      0.25F, 0.0F, 0.0F, -1.0F,  0.0F,  0.0F, 0.25F, 0.5F, 1.0F, -1.0F, 1.0F, 1.0F,
  };
  std::vector<bool> drawAngle = {true, false, false, false, false, false, false, false, true, true, true, true};
  float radius = 1.0; // Work with a Unit Circle.
  for(size_t idx = 0; idx < angles.size(); idx++)
  {
    radius = 1.0F;
    float angle = angles[idx];
    float rads = angle * M_PI / 180.0f;
    float x = radius * (cos(rads));
    float y = radius * (sin(rads));

    // Transform from Unit Circle to our flipped Screen Pixel Coordinates
    // First Scale up to our image dimensions
    x = x * halfWidth;
    y = y * halfHeight;

    // Next, translate to the center of the image
    x = x + halfWidth;
    y = y + halfHeight;

    // Now mirror across the x-axis (vertically) because this is the transformation from
    // cartesian coords to screen coords
    y = legendHeight - y;

    x = x + figureOrigin[0];
    y = y + figureOrigin[1];

    // Draw the line from the center point to the point on the circle

      context.set_color(canvas_ity::stroke_style, 0.25f, 0.25f, 0.25f, 1.0f);
      context.set_line_width(penWidth);
      EbsdLib::DrawLine(context, figureCenter[0], figureCenter[1], x, y);

    std::string label = labels2[idx];
    std::string fontWidthString = EbsdStringUtils::replace(label, "-", "");
    float fontWidth = context.measure_text(fontWidthString.c_str());

    x = x + (xAdj[idx] * fontWidth);
    y = y + (yAdj[idx] * fontPtSize);

    context.set_color(canvas_ity::stroke_style, 0.0f, 0.0f, 0.0f, 1.0f);

      EbsdLib::WriteText(context, label, {x, y}, fontPtSize);

  }

  // Draw the [0001] in the center of the image
  {
    float x = figureCenter[0];
    float y = figureCenter[1] - fontPtSize * 0.2F;

    std::string label("[0001]");
    EbsdLib::WriteText(context, label, {x, y}, fontPtSize);
  }

  // Fetch the rendered RGBA pixels from the entire canvas.
  EbsdLib::UInt8ArrayType::Pointer rgbaCanvasImage = EbsdLib::UInt8ArrayType::CreateArray(pageHeight * pageWidth, {4ULL}, "Triangle Legend", true);
  // std::vector<unsigned char> rgbaCanvasImage(static_cast<size_t>(pageHeight * pageWidth * 4));
  context.get_image_data(rgbaCanvasImage->getPointer(0), pageWidth, pageHeight, pageWidth * 4, 0, 0);

  rgbaCanvasImage = EbsdLib::RemoveAlphaChannel(rgbaCanvasImage.get());
  return rgbaCanvasImage;
}



// -----------------------------------------------------------------------------
void DrawStereographicLines(canvas_ity::canvas& context, const std::vector<EbsdLib::Point3DType>& directions, int numPoints, int halfWidth,
                            std::array<float, 2> figureOrigin)
{


  for(const auto& direction : directions)
  {
    std::vector<EbsdLib::Point3DType> stereoPoints = Stereographic::Utils::TransformUnitSphereToStereographicCoords(EbsdLib::GeneratePointsOnUnitCircle(direction, numPoints));
    for(size_t i = 1; i < stereoPoints.size(); i++)
    {
      EbsdLib::Point3DType p0 = stereoPoints[i - 1];
      EbsdLib::Point3DType p1 = stereoPoints[i];
      p0 = (p0 * static_cast<float>(halfWidth)) + halfWidth;
      p1 = (p1 * static_cast<float>(halfWidth)) + halfWidth;

      p0[0] = p0[0] + figureOrigin[0];
      p0[1] = p0[1] + figureOrigin[1];

      p1[0] = p1[0] + figureOrigin[0];
      p1[1] = p1[1] + figureOrigin[1];

      EbsdLib::DrawLine(context, p0[0], p0[1], p1[0], p1[1]);
    }
  }
}

} // namespace EbsdLib
