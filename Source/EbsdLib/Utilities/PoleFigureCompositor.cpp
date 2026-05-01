#include "PoleFigureCompositor.h"

#include "EbsdLib/LaueOps/LaueOps.h"
#include "EbsdLib/Math/EbsdLibMath.h"
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
CompositePoleFigureResult PoleFigureCompositor::generateCompositeImage(CompositePoleFigureConfiguration_t& config)
{
  CompositePoleFigureResult result;

  // Stage 1: Generate 3 individual pole figure RGBA images
  std::vector<UInt8ArrayType::Pointer> figures = generatePoleFigures(config);
  if(figures.size() != 3)
  {
    return result;
  }

  // Stage 2: Preprocess (flip + color conversion, optionally parallel)
  preprocessImages(figures, config.imageDim, config.flipFinalImage);

  // Stage 3: Compute layout
  LayoutMetrics layout = computeLayoutMetrics(config);

  // Stage 4: Composite onto canvas
  result.image = compositeToCanvas(config, figures, layout);
  result.width = layout.pageWidth;
  result.height = layout.pageHeight;

  return result;
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

  // Extra margin on the right to ensure the "X" axis label is not clipped
  metrics.subCanvasWidth = metrics.margins + imageWidth + metrics.margins + xCharWidth + metrics.margins;
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
std::vector<UInt8ArrayType::Pointer> PoleFigureCompositor::generatePoleFigures(CompositePoleFigureConfiguration_t& config)
{
  PoleFigureConfiguration_t pfConfig;
  pfConfig.eulers = config.eulers;
  pfConfig.imageDim = config.imageDim;
  pfConfig.lambertDim = config.lambertDim;
  pfConfig.numColors = config.numColors;
  pfConfig.minScale = config.minScale;
  pfConfig.maxScale = config.maxScale;
  pfConfig.sphereRadius = config.sphereRadius;
  pfConfig.discrete = config.discrete;
  pfConfig.discreteHeatMap = config.discreteHeatMap;
  pfConfig.colorMap = config.colorMap;
  pfConfig.labels = config.labels;
  pfConfig.order = config.order;
  pfConfig.phaseName = config.phaseName;
  pfConfig.FlipFinalImage = config.flipFinalImage;
  pfConfig.hexConvention = config.hexConvention;

  std::vector<LaueOps::Pointer> orientationOps = LaueOps::GetAllOrientationOps();
  if(config.laueOpsIndex >= orientationOps.size())
  {
    return {};
  }

  auto result = orientationOps[config.laueOpsIndex]->generatePoleFigure(pfConfig);

  // LaueOps::generatePoleFigure updates minScale/maxScale to reflect the actual
  // data range. Propagate these back so the scalar bar shows correct values.
  config.minScale = pfConfig.minScale;
  config.maxScale = pfConfig.maxScale;

  return result;
}

// -----------------------------------------------------------------------------
void PoleFigureCompositor::preprocessImages(std::vector<UInt8ArrayType::Pointer>& images, int imageDim, bool flip)
{
#ifdef EbsdLib_USE_PARALLEL_ALGORITHMS
  tbb::task_group g;
  for(auto& image : images)
  {
    g.run([&image, imageDim, flip]() {
      if(flip)
      {
        image = flipAndMirror(image.get(), imageDim);
      }
      image = convertColorOrder(image.get(), imageDim);
    });
  }
  g.wait();
#else
  for(auto& image : images)
  {
    if(flip)
    {
      image = flipAndMirror(image.get(), imageDim);
    }
    image = convertColorOrder(image.get(), imageDim);
  }
#endif
}

// -----------------------------------------------------------------------------
UInt8ArrayType::Pointer PoleFigureCompositor::compositeToCanvas(const CompositePoleFigureConfiguration_t& config, const std::vector<UInt8ArrayType::Pointer>& images, const LayoutMetrics& layout)
{
  std::vector<unsigned char> latoBold = fonts::GetLatoBold();
  std::vector<unsigned char> latoRegular = fonts::GetLatoRegular();
  std::vector<unsigned char> firaSans = fonts::GetFiraSansRegular();

  canvas_ity::canvas context(layout.pageWidth, layout.pageHeight);

  context.set_font(latoBold.data(), static_cast<int>(latoBold.size()), layout.fontPtSize);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.text_baseline = canvas_ity::alphabetic;

  // White background
  context.move_to(0.0f, 0.0f);
  context.line_to(static_cast<float>(layout.pageWidth), 0.0f);
  context.line_to(static_cast<float>(layout.pageWidth), static_cast<float>(layout.pageHeight));
  context.line_to(0.0f, static_cast<float>(layout.pageHeight));
  context.line_to(0.0f, 0.0f);
  context.close_path();
  context.set_color(canvas_ity::fill_style, 1.0f, 1.0f, 1.0f, 1.0f);
  context.fill();

  // Draw each of the 3 pole figures
  for(int i = 0; i < 3 && i < static_cast<int>(images.size()); i++)
  {
    std::string directionLabel = images[i]->getName();
    drawPoleFigure(context, *images[i], layout.origins[i], config.imageDim, directionLabel, layout.fontPtSize, layout.margins, latoBold, firaSans);
  }

  // Title
  drawTitle(context, config.title, static_cast<float>(layout.pageWidth), layout.fontPtSize, layout.margins, latoBold);

  // Legend at 4th position
  const float legendFontPtSize = static_cast<float>(config.imageDim) / 20.0f;
  if(config.discrete)
  {
    drawInfoBlock(context, config, layout.origins[3], layout.margins, legendFontPtSize, latoRegular);
  }
  else
  {
    drawScalarBar(context, config, layout.origins[3], layout.margins, legendFontPtSize, latoRegular);
  }

  // Extract RGBA pixels
  auto result = UInt8ArrayType::CreateArray(static_cast<size_t>(layout.pageWidth) * layout.pageHeight, {4ULL}, "CompositePoleFigure", true);
  context.get_image_data(result->getPointer(0), layout.pageWidth, layout.pageHeight, layout.pageWidth * 4, 0, 0);

  return result;
}

// -----------------------------------------------------------------------------
void PoleFigureCompositor::drawPoleFigure(canvas_ity::canvas& context, const UInt8ArrayType& image, std::array<float, 2> origin, int imageDim, const std::string& directionLabel, float fontPtSize,
                                          float margins, const std::vector<unsigned char>& latoBold, const std::vector<unsigned char>& firaSans)
{
  const auto imageSize = static_cast<float>(imageDim);

  // Draw the pole figure image
  context.draw_image(const_cast<uint8_t*>(image.getPointer(0)), imageDim, imageDim, imageDim * static_cast<int>(image.getNumberOfComponents()), origin[0] + margins,
                     origin[1] + fontPtSize * 2.0f + margins * 2.0f, imageSize, imageSize);

  // Circle outline
  context.begin_path();
  context.line_cap = canvas_ity::circle;
  context.set_line_width(3.0f);
  context.set_color(canvas_ity::stroke_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.arc(origin[0] + margins + imageSize / 2.0f, origin[1] + fontPtSize * 2.0f + margins * 2.0f + imageSize / 2.0f, imageSize / 2.0f, 0, ebsdlib::constants::k_2PiF);
  context.stroke();
  context.close_path();

  // X axis line
  context.begin_path();
  context.line_cap = canvas_ity::square;
  context.set_line_width(2.0f);
  context.set_color(canvas_ity::stroke_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.move_to(origin[0] + margins, origin[1] + fontPtSize * 2.0f + margins * 2.0f + imageSize / 2.0f);
  context.line_to(origin[0] + margins + imageSize, origin[1] + fontPtSize * 2.0f + margins * 2.0f + imageSize / 2.0f);
  context.stroke();
  context.close_path();

  // Y axis line
  context.begin_path();
  context.line_cap = canvas_ity::square;
  context.set_line_width(2.0f);
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
  context.fill_text("TD", origin[0] + margins * 1.5f + imageSize, origin[1] + fontPtSize * 2.25f + margins * 2.0f + imageSize / 2.0f);
  context.close_path();

  // "Y" axis label
  context.begin_path();
  context.set_font(const_cast<unsigned char*>(latoBold.data()), static_cast<int>(latoBold.size()), fontPtSize);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.text_baseline = canvas_ity::alphabetic;
  const float yFontWidth = context.measure_text("RD");
  context.fill_text("RD", origin[0] + margins - (0.5f * yFontWidth) + imageSize / 2.0f, origin[1] + fontPtSize * 2.0f + margins);
  context.close_path();

  // Direction label (e.g., "<001>" displayed as "(001)")
  std::string subtitle = EbsdStringUtils::replace(directionLabel, "<", "(");
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

// -----------------------------------------------------------------------------
void PoleFigureCompositor::drawScalarBar(canvas_ity::canvas& context, const CompositePoleFigureConfiguration_t& config, std::array<float, 2> position, float margins, float fontPtSize,
                                         const std::vector<unsigned char>& latoRegular)
{
  const int numColors = config.numColors;

  std::vector<ebsdlib::Rgb> colorTable(numColors);
  std::vector<float> colors(3 * numColors, 0.0f);
  EbsdColorTable::GetColorTable(numColors, colors);
  for(int i = 0; i < numColors; i++)
  {
    float r = colors[3 * i];
    float g = colors[3 * i + 1];
    float b = colors[3 * i + 2];
    colorTable[i] = ebsdlib::RgbColor::dRgb(static_cast<int>(r * 255.0f), static_cast<int>(g * 255.0f), static_cast<int>(b * 255.0f), 255);
  }

  const float scaleBarRelativeWidth = 0.10f;
  const float colorHeight = static_cast<float>(config.imageDim) / static_cast<float>(numColors);
  const float rectWidth = static_cast<float>(config.imageDim) * scaleBarRelativeWidth;

  // Max value text
  context.begin_path();
  const std::string maxStr = fmt::format("{:#.6}", config.maxScale);
  context.set_font(const_cast<unsigned char*>(latoRegular.data()), static_cast<int>(latoRegular.size()), fontPtSize);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.text_baseline = canvas_ity::alphabetic;
  context.fill_text(maxStr.c_str(), position[0] + 2.0f * margins + rectWidth, position[1] + 2.0f * margins + 2.0f * fontPtSize + colorHeight);
  context.close_path();

  // Min value text
  context.begin_path();
  const std::string minStr = fmt::format("{:#.6}", config.minScale);
  context.set_font(const_cast<unsigned char*>(latoRegular.data()), static_cast<int>(latoRegular.size()), fontPtSize);
  context.set_color(canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
  context.text_baseline = canvas_ity::alphabetic;
  context.fill_text(minStr.c_str(), position[0] + 2.0f * margins + rectWidth, position[1] + 2.0f * margins + 2.0f * fontPtSize + static_cast<float>(numColors) * colorHeight);
  context.close_path();

  // Color bar rectangles
  for(int i = 0; i < numColors; i++)
  {
    const ebsdlib::Rgb c = colorTable[numColors - i - 1];
    auto [r, g, b] = ebsdlib::RgbColor::fRgb(c);

    const float x = position[0] + margins;
    const float y = position[1] + 2.0f * margins + 2.0f * fontPtSize + static_cast<float>(i) * colorHeight;

    context.begin_path();
    context.set_color(canvas_ity::fill_style, r, g, b, 1.0f);
    context.fill_rectangle(x, y, rectWidth, colorHeight);
    context.set_color(canvas_ity::stroke_style, r, g, b, 1.0f);
    context.set_line_width(1.0f);
    context.stroke_rectangle(x, y, rectWidth, colorHeight);
  }

  drawInfoBlock(context, config, position, margins, fontPtSize, latoRegular);
}

// -----------------------------------------------------------------------------
void PoleFigureCompositor::drawInfoBlock(canvas_ity::canvas& context, const CompositePoleFigureConfiguration_t& config, std::array<float, 2> position, float margins, float fontPtSize,
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

  const std::vector<std::string> labels = {fmt::format("Phase Num: {}", config.phaseNumber),
                                           fmt::format("Material Name: {}", config.phaseName),
                                           fmt::format("Laue Group: {}", laueGroupName),
                                           fmt::format("Upper & Lower:"),
                                           fmt::format("Samples: {}", config.eulers != nullptr ? config.eulers->getNumberOfTuples() : 0),
                                           fmt::format("Lambert Sq. Dim: {}", config.lambertDim),
                                           fmt::format("Hex/Trig Convention: {}", config.hexConvention == ebsdlib::HexConvention::XParallelAStar ? "x||a*" : "x||a")};

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
void PoleFigureCompositor::drawTitle(canvas_ity::canvas& context, const std::string& title, float pageWidth, float fontPtSize, float margins, const std::vector<unsigned char>& latoBold)
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
UInt8ArrayType::Pointer PoleFigureCompositor::flipAndMirror(UInt8ArrayType* src, int imageDim)
{
  UInt8ArrayType::Pointer converted = UInt8ArrayType::CreateArray(static_cast<size_t>(imageDim) * imageDim, src->getComponentDimensions(), src->getName(), true);
  for(int y = 0; y < imageDim; y++)
  {
    const int destY = imageDim - 1 - y;
    for(int x = 0; x < imageDim; x++)
    {
      const size_t indexSrc = static_cast<size_t>(y) * imageDim + x;
      const size_t indexDest = static_cast<size_t>(destY) * imageDim + x;
      uint8_t* srcPtr = src->getTuplePointer(indexSrc);
      converted->setTuple(indexDest, srcPtr);
    }
  }
  return converted;
}

// -----------------------------------------------------------------------------
UInt8ArrayType::Pointer PoleFigureCompositor::convertColorOrder(UInt8ArrayType* src, int imageDim)
{
  UInt8ArrayType::Pointer converted = UInt8ArrayType::CreateArray(src->getNumberOfTuples(), src->getComponentDimensions(), src->getName(), true);
  for(size_t tIdx = 0; tIdx < src->getNumberOfTuples(); tIdx++)
  {
    uint8_t* argbPtr = src->getTuplePointer(tIdx);
    uint8_t* destPtr = converted->getTuplePointer(tIdx);
    destPtr[0] = argbPtr[2];
    destPtr[1] = argbPtr[1];
    destPtr[2] = argbPtr[0];
    destPtr[3] = argbPtr[3];
  }
  return converted;
}

} // namespace ebsdlib
