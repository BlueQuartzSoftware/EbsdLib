#include <catch2/catch.hpp>

#include "EbsdLib/Utilities/Fonts.hpp"
#include "EbsdLib/Utilities/ODFSectionChrome.h"
#include "EbsdLib/Utilities/ODFSectionCompositor.h"

#include <canvas_ity.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fmt/format.h>
#include <limits>

using namespace ebsdlib;

namespace
{
ODFSectionConfiguration RenderConfiguration(DoubleArrayType* values)
{
  ODFSectionConfiguration config;
  config.grid = {values, {4, 2, 4}, {0, 0, 0}, {90, 90, 90}, ODFValueUnits::MUD, CrystalStructure::Triclinic};
  config.sectionCount = 4;
  config.sectionsPerRow = 2;
  config.sectionWidth = 128;
  config.scaleMode = ODFScaleMode::Manual;
  config.manualMaximumMUD = 4;
  config.colorControlPoints = {0, 0, 0, 1, 1, 1, 0, 0};
  config.title = "Standard ODF sections";
  config.materialName = "Titanium";
  return config;
}

void RequirePixel(const ODFSectionResult& result, int32_t x, int32_t y, std::array<int, 4> expected)
{
  const size_t offset = (static_cast<size_t>(y) * result.width + x) * 4;
  for(size_t componentIndex = 0; componentIndex < 4; componentIndex++)
  {
    INFO("Pixel " << x << ", " << y << " component " << componentIndex);
    REQUIRE(std::abs(static_cast<int>(result.image->getValue(offset + componentIndex)) - expected[componentIndex]) <= 1);
  }
}

bool HasInk(const ODFSectionResult& result, int32_t x, int32_t y, int32_t width, int32_t height)
{
  for(int32_t row = y; row < y + height; row++)
  {
    for(int32_t column = x; column < x + width; column++)
    {
      if(result.image->getValue((static_cast<size_t>(row) * result.width + column) * 4) < 128)
      {
        return true;
      }
    }
  }
  return false;
}

bool HasPixelDifference(const ODFSectionResult& first, const ODFSectionResult& second, int32_t x, int32_t y, int32_t width, int32_t height)
{
  for(int32_t row = y; row < y + height; row++)
  {
    for(int32_t column = x; column < x + width; column++)
    {
      const size_t pixelOffset = (static_cast<size_t>(row) * first.width + column) * 4;
      for(size_t componentIndex = 0; componentIndex < 4; componentIndex++)
      {
        if(first.image->getValue(pixelOffset + componentIndex) != second.image->getValue(pixelOffset + componentIndex))
        {
          return true;
        }
      }
    }
  }
  return false;
}

std::vector<uint8_t> RenderFiraGlyph(const std::vector<unsigned char>& font, const std::string& glyph)
{
  canvas_ity::canvas context(48, 48);
  context.set_color(canvas_ity::fill_style, 1, 1, 1, 1);
  context.fill_rectangle(0, 0, 48, 48);
  context.set_color(canvas_ity::fill_style, 0, 0, 0, 1);
  context.set_font(font.data(), static_cast<int>(font.size()), 24);
  context.fill_text(glyph.c_str(), 4, 32);
  std::vector<uint8_t> pixels(48 * 48 * 4);
  context.get_image_data(pixels.data(), 48, 48, 48 * 4, 0, 0);
  return pixels;
}
} // namespace

TEST_CASE("ebsdlib::ODFSectionCompositor::ChromeTextAndTicks", "[EbsdLib][ODFSectionCompositor]")
{
  const std::string title = FormatODFSectionTitle(10.0);
  REQUIRE(std::vector<uint8_t>(title.begin(), title.end()) == std::vector<uint8_t>{0xCF, 0x86, 0xE2, 0x82, 0x82, 0x20, 0x3D, 0x20, 0x31, 0x30, 0xC2, 0xB0});
  const std::string horizontalAxisTitle = GetODFHorizontalAxisTitle();
  REQUIRE(std::vector<uint8_t>(horizontalAxisTitle.begin(), horizontalAxisTitle.end()) == std::vector<uint8_t>{0xCF, 0x86, 0xE2, 0x82, 0x81});
  const std::string verticalAxisTitle = GetODFVerticalAxisTitle();
  REQUIRE(std::vector<uint8_t>(verticalAxisTitle.begin(), verticalAxisTitle.end()) == std::vector<uint8_t>{0xCE, 0xA6});
  REQUIRE(SelectODFAxisTickInterval(360.0, 900) == Approx(10.0));
  REQUIRE(SelectODFAxisTickInterval(360.0, 864) == Approx(10.0));
  REQUIRE(SelectODFAxisTickInterval(360.0, 512) == Approx(20.0));
  REQUIRE(GenerateODFAxisTicks(90.0, 128) == std::vector<double>{0.0, 20.0, 40.0, 60.0, 80.0, 90.0});
  REQUIRE(GenerateODFAxisTicks(360.0, 128) == std::vector<double>{0.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0, 160.0, 180.0, 200.0, 220.0, 240.0, 260.0, 280.0, 300.0, 320.0, 340.0, 360.0});
  REQUIRE(GenerateODFAxisLabelTicks(360.0, 128).size() < GenerateODFAxisTicks(360.0, 128).size());
  REQUIRE(GenerateODFAxisTicks(360.0, 864).size() == 37);
  REQUIRE(GetODFColorBarTitle(ODFValueUnits::MUD) == "MUD");
  REQUIRE(GetODFColorBarTitle(ODFValueUnits::CountDensity) == "MUD (from Count-Density)");
  for(const double tick : GenerateODFAxisTicks(90.0, 128))
  {
    const std::string label = fmt::format("{:g}", tick);
    REQUIRE(label.find("deg") == std::string::npos);
    for(const unsigned char value : label)
    {
      REQUIRE(value < 0x80);
    }
  }
}

TEST_CASE("ebsdlib::ODFSectionCompositor::FiraGreekGlyphs", "[EbsdLib][ODFSectionCompositor]")
{
  const auto font = fonts::GetFiraSansRegular();
  const auto missingGlyph = RenderFiraGlyph(font, "\xF4\x8F\xBF\xBF");
  const std::array<std::string, 5> requiredGlyphs = {"\xCF\x86", "\xE2\x82\x81", "\xE2\x82\x82", "\xCE\xA6", "\xC2\xB0"};
  for(const auto& glyph : requiredGlyphs)
  {
    const auto pixels = RenderFiraGlyph(font, glyph);
    REQUIRE(pixels != missingGlyph);
    REQUIRE(std::any_of(pixels.begin(), pixels.end(), [](uint8_t value) { return value < 128; }));
  }
}

TEST_CASE("ebsdlib::ODFSectionCompositor::AxisLabelBounds", "[EbsdLib][ODFSectionCompositor]")
{
  const int32_t width = GENERATE(128, 512, 864);
  const bool vertical = GENERATE(false, true);
  const double maximum = vertical ? 90.0 : 360.0;
  const int32_t length = vertical ? width / 4 : width;
  const float tickSize = std::min(std::max(10.0f, width / 24.0f) * 0.7f, std::max(8.0f, width / 32.0f) * 0.8f);
  const auto font = fonts::GetFiraSansRegular();
  canvas_ity::canvas context(1, 1);
  context.set_font(font.data(), static_cast<int>(font.size()), tickSize);
  const auto labels = GenerateODFAxisLabelTicks(maximum, length, tickSize);
  REQUIRE(labels.front() == 0.0);
  REQUIRE(labels.back() == maximum);
  float previousRight = -4.0f;
  for(const double tick : labels)
  {
    const float textWidth = context.measure_text(fmt::format("{:g}", tick).c_str());
    const float position = static_cast<float>(tick / maximum * length);
    const float left = position - (tick == 0 ? 0 : (tick == maximum ? textWidth : textWidth / 2));
    CAPTURE(width, vertical, tick, left, previousRight);
    CHECK(left >= previousRight + 4.0f);
    CHECK(left >= 0.0f);
    CHECK(left + textWidth <= length);
    previousRight = left + textWidth;
  }
}

TEST_CASE("ebsdlib::ODFSectionCompositor::ComposedVerticalGlyphBounds", "[EbsdLib][ODFSectionCompositor]")
{
  const int32_t width = GENERATE(128, 512, 864);
  auto values = DoubleArrayType::CreateArray(32, "Uniform", true);
  values->initializeWithValue(1.0);
  auto config = RenderConfiguration(values.get());
  config.sectionWidth = width;
  config.sectionCount = 6;
  config.sectionsPerRow = 3;
  config.grid.laueOpsIndex = GENERATE(CrystalStructure::Triclinic, CrystalStructure::Hexagonal_High);
  const auto layout = ComputeODFSectionLayout(config);
  const auto result = ODFSectionCompositor{}.generateCompositeImage(config);
  REQUIRE(result);
  const auto font = fonts::GetFiraSansRegular();
  // An isolated glyph with a white border detects clipping, missing pixels, and adjacent tick-label ink.
  for(size_t sectionIndex = 0; sectionIndex < config.sectionCount; ++sectionIndex)
  {
    const auto origin = GetODFSectionPanelOrigin(layout, sectionIndex);
    const int32_t baselineX = static_cast<int32_t>(std::lround((sectionIndex % config.sectionsPerRow) * layout.panelSlotWidth + layout.margin + layout.fontPtSize));
    const int32_t baselineY = static_cast<int32_t>(std::lround(origin[1] + layout.panelHeight / 2.0f));
    const int32_t extent = static_cast<int32_t>(std::ceil(layout.fontPtSize)) + 3;
    canvas_ity::canvas reference(result.width, result.height);
    reference.set_color(canvas_ity::fill_style, 1, 1, 1, 1);
    reference.fill_rectangle(0, 0, static_cast<float>(result.width), static_cast<float>(result.height));
    reference.set_color(canvas_ity::fill_style, 0, 0, 0, 1);
    reference.set_font(font.data(), static_cast<int>(font.size()), layout.fontPtSize);
    reference.text_align = canvas_ity::center;
    reference.translate(static_cast<float>(baselineX), static_cast<float>(baselineY));
    reference.rotate(-1.5707963267948966f);
    reference.fill_text("\xCE\xA6", 0, 0);
    std::vector<uint8_t> pixels(static_cast<size_t>(4 * extent * extent * 4));
    reference.get_image_data(pixels.data(), 2 * extent, 2 * extent, 2 * extent * 4, baselineX - extent, baselineY - extent);
    int32_t minX = 2 * extent, minY = 2 * extent, maxX = 0, maxY = 0;
    for(int32_t y = 0; y < 2 * extent; ++y)
    {
      for(int32_t x = 0; x < 2 * extent; ++x)
      {
        if(pixels[(y * 2 * extent + x) * 4] < 250)
        {
          minX = std::min(minX, x);
          maxX = std::max(maxX, x);
          minY = std::min(minY, y);
          maxY = std::max(maxY, y);
        }
      }
    }
    REQUIRE(minX < maxX);
    CHECK(baselineX - extent + minX > 0);
    CHECK(baselineX - extent + maxX + 2 < origin[0] - 3);
    bool glyphMatches = true;
    for(int32_t y = minY - 2; y <= maxY + 2; ++y)
    {
      for(int32_t x = minX - 2; x <= maxX + 2; ++x)
      {
        const int32_t pageX = baselineX - extent + x;
        const int32_t pageY = baselineY - extent + y;
        REQUIRE(pageX >= 0);
        REQUIRE(pageX < result.width);
        REQUIRE(pageY >= 0);
        REQUIRE(pageY < result.height);
        const int expected = pixels[(y * 2 * extent + x) * 4];
        const int actual = result.image->getValue((static_cast<size_t>(pageY) * result.width + pageX) * 4);
        glyphMatches = glyphMatches && std::abs(expected - actual) <= 1;
      }
    }
    CAPTURE(width, sectionIndex);
    CHECK(glyphMatches);
    const auto limits = GetODFEulerPlotLimits(config.grid.laueOpsIndex);
    for(const double tick : GenerateODFAxisTicks(limits.phi1MaxDeg, layout.panelWidth))
    {
      const auto x = static_cast<int32_t>(std::ceil(origin[0] + tick / limits.phi1MaxDeg * layout.panelWidth)) - 1;
      const auto y = static_cast<int32_t>(std::ceil(origin[1] + layout.panelHeight)) + 1;
      // Fractional tick positions can cover less than half of the sampled pixel.
      CHECK(result.image->getValue((static_cast<size_t>(y) * result.width + x) * 4) < 255);
    }
    for(const double tick : GenerateODFAxisTicks(limits.phiMaxDeg, layout.panelHeight))
    {
      const auto x = static_cast<int32_t>(std::ceil(origin[0])) - 2;
      const auto y = static_cast<int32_t>(std::floor(origin[1] + tick / limits.phiMaxDeg * layout.panelHeight));
      CHECK(result.image->getValue((static_cast<size_t>(y) * result.width + x) * 4) < 255);
    }
  }
}

TEST_CASE("ebsdlib::ODFSectionCompositor::DefaultsAndLayout", "[EbsdLib][ODFSectionCompositor]")
{
  ODFSectionConfiguration config;
  REQUIRE(config.sectionCount == 6);
  REQUIRE(config.sectionsPerRow == 3);
  REQUIRE(config.sectionWidth == 512);
  REQUIRE(config.scaleMode == ODFScaleMode::Automatic);
  REQUIRE(config.manualMinimumMUD == 0.0);
  REQUIRE(config.manualMaximumMUD == 1.0);
  REQUIRE(config.phaseNumber == 1);
  REQUIRE(config.colorControlPoints.empty());
  config.grid.laueOpsIndex = CrystalStructure::Hexagonal_High;
  const auto layout = ComputeODFSectionLayout(config);
  REQUIRE(layout.panelWidth == 512);
  REQUIRE(layout.panelHeight == 128);
  REQUIRE(layout.columns == 3);
  REQUIRE(layout.rows == 2);
  REQUIRE(layout.fontPtSize == Approx(512.0 / 24.0));
  REQUIRE(layout.margin == 16.0f);
  REQUIRE(layout.leftAxisGutter == 41.0f);
  REQUIRE(layout.panelSlotWidth == 585.0f);
  REQUIRE(layout.panelSlotHeight == 240.0f);
  REQUIRE(layout.titleHeight == Approx(53.333333));
  REQUIRE(layout.legendWidth == 180.0f);
  REQUIRE(layout.pageWidth == 1967);
  REQUIRE(layout.pageHeight == 550);
}

TEST_CASE("ebsdlib::ODFSectionCompositor::EmptyDisplayedCrop", "[EbsdLib][ODFSectionCompositor]")
{
  auto values = DoubleArrayType::CreateArray(4, "CoarseGrid", true);
  values->initializeWithValue(1.0);
  auto config = RenderConfiguration(values.get());
  config.grid = {values.get(), {2, 1, 2}, {0, 0, 0}, {180, 180, 180}, ODFValueUnits::MUD, CrystalStructure::Hexagonal_High};
  config.sectionCount = 6;
  config.sectionsPerRow = 3;
  config.manualMaximumMUD = 1.0;
  config.scaleMode = GENERATE(ODFScaleMode::Manual, ODFScaleMode::Automatic);
  // Preparation must reject the empty crop before the renderer can read its values.
  REQUIRE_FALSE(PrepareODFSections(config.grid, config.sectionCount));
  const auto result = ODFSectionCompositor{}.generateCompositeImage(config);
  REQUIRE_FALSE(result);
  REQUIRE(result.errorCode == -7502);
  REQUIRE(result.image == nullptr);
}

TEST_CASE("ebsdlib::ODFSectionCompositor::InvalidConfiguration", "[EbsdLib][ODFSectionCompositor]")
{
  auto values = DoubleArrayType::CreateArray(32, "Uniform", true);
  values->initializeWithValue(1.0);
  ODFSectionConfiguration config;
  config.grid = {values.get(), {4, 2, 4}, {0, 0, 0}, {90, 90, 90}, ODFValueUnits::MUD, CrystalStructure::Triclinic};
  config.colorControlPoints = {0, 0, 0, 1, 1, 1, 0, 0};
  int32_t expectedCode = 0;
  SECTION("Nonpositive width")
  {
    config.sectionWidth = 0;
    expectedCode = -7510;
  }
  SECTION("Unrepresentable width")
  {
    config.sectionWidth = std::numeric_limits<int32_t>::max();
    expectedCode = -7510;
  }
  SECTION("Zero columns")
  {
    config.sectionsPerRow = 0;
    expectedCode = -7511;
  }
  SECTION("Too many columns")
  {
    config.sectionsPerRow = std::numeric_limits<size_t>::max();
    expectedCode = -7511;
  }
  SECTION("Unrepresentable section count")
  {
    config.sectionCount = std::numeric_limits<size_t>::max();
    expectedCode = -7510;
  }
  SECTION("Preparation error propagation")
  {
    config.grid.values = nullptr;
    expectedCode = -7500;
  }
  SECTION("Nonfinite automatic MUD maximum")
  {
    values->initializeWithValue(std::numeric_limits<double>::max());
    config.grid.units = ODFValueUnits::CountDensity;
    expectedCode = -7512;
  }
  SECTION("Invalid manual endpoints")
  {
    config.scaleMode = ODFScaleMode::Manual;
    config.manualMinimumMUD = 1;
    config.manualMaximumMUD = 1;
    expectedCode = -7512;
  }
  SECTION("Negative manual minimum")
  {
    config.scaleMode = ODFScaleMode::Manual;
    config.manualMinimumMUD = -1;
    expectedCode = -7512;
  }
  SECTION("Nonfinite manual maximum")
  {
    config.scaleMode = ODFScaleMode::Manual;
    config.manualMaximumMUD = std::numeric_limits<double>::infinity();
    expectedCode = -7512;
  }
  SECTION("Zero automatic maximum")
  {
    values->initializeWithValue(0.0);
    expectedCode = -7512;
  }
  SECTION("One color control")
  {
    config.colorControlPoints = {0, 0, 0, 1};
    expectedCode = -7513;
  }
  SECTION("Incomplete control")
  {
    config.colorControlPoints.push_back(0);
    expectedCode = -7513;
  }
  SECTION("Nonfinite control")
  {
    config.colorControlPoints[1] = std::numeric_limits<float>::quiet_NaN();
    expectedCode = -7513;
  }
  SECTION("Repeated positions")
  {
    config.colorControlPoints[4] = 0;
    expectedCode = -7513;
  }
  const auto result = ODFSectionCompositor{}.generateCompositeImage(config);
  REQUIRE_FALSE(result);
  REQUIRE(result.errorCode == expectedCode);
  REQUIRE_FALSE(result.errorMessage.empty());
}

TEST_CASE("ebsdlib::ODFSectionCompositor::RgbaPanelsAndChrome", "[EbsdLib][ODFSectionCompositor]")
{
  auto values = DoubleArrayType::CreateArray(32, "Planes", true);
  values->initializeWithValue(0.0);
  for(size_t phi1Index = 0; phi1Index < 4; phi1Index++)
  {
    for(size_t phiIndex = 0; phiIndex < 2; phiIndex++)
    {
      const size_t flatIndex = (phi1Index * 2 + phiIndex) * 4 + 2;
      values->setValue(flatIndex, 8.0);
    }
  }
  const auto config = RenderConfiguration(values.get());
  const auto layout = ComputeODFSectionLayout(config);
  REQUIRE(layout.panelHeight == 64);
  REQUIRE(layout.panelSlotHeight == 118);
  REQUIRE(layout.pageWidth == 534);
  REQUIRE(layout.pageHeight == 270);
  const auto result = ODFSectionCompositor{}.generateCompositeImage(config);
  REQUIRE(result);
  REQUIRE(result.image->getNumberOfComponents() == 4);
  REQUIRE(result.image->getNumberOfTuples() == static_cast<size_t>(result.width * result.height));
  REQUIRE(result.width == layout.pageWidth);
  REQUIRE(result.height == layout.pageHeight);
  REQUIRE(result.sectionAnglesDeg == std::vector<double>{0, 90, 180, 270});
  REQUIRE(result.appliedMinimumMUD == Approx(0));
  REQUIRE(result.appliedMaximumMUD == Approx(4));
  RequirePixel(result, 64, 76, {0, 0, 255, 255});
  RequirePixel(result, 64, 194, {255, 0, 0, 255});
  RequirePixel(result, 480, 266, {255, 255, 255, 255});
  RequirePixel(result, 64, 108, {0, 0, 0, 255});
  RequirePixel(result, 362, 44, {255, 0, 0, 255});
  RequirePixel(result, 362, 107, {0, 0, 255, 255});
  REQUIRE(HasInk(result, 8, 8, 250, 12));
  REQUIRE(HasInk(result, 8, 26, 128, 12));
  REQUIRE(HasInk(result, 8, 121, 128, 12));
  REQUIRE(HasInk(result, 378, 54, 148, 84));
  for(size_t pixelIndex = 0; pixelIndex < result.image->getNumberOfTuples(); pixelIndex++)
  {
    REQUIRE(result.image->getValue(pixelIndex * 4 + 3) == 255);
  }
}

TEST_CASE("ebsdlib::ODFSectionCompositor::SourceUnitsAndDenseChrome", "[EbsdLib][ODFSectionCompositor]")
{
  auto mudValues = DoubleArrayType::CreateArray(32, "MudValues", true);
  mudValues->initializeWithValue(1.5);
  auto densityValues = DoubleArrayType::CreateArray(32, "DensityValues", true);
  const double stepRad = std::acos(-1.0) / 2.0;
  for(size_t phi1Index = 0; phi1Index < 4; phi1Index++)
  {
    for(size_t phiIndex = 0; phiIndex < 2; phiIndex++)
    {
      for(size_t phi2Index = 0; phi2Index < 4; phi2Index++)
      {
        const size_t flatIndex = (phi1Index * 2 + phiIndex) * 4 + phi2Index;
        densityValues->setValue(flatIndex, 1.5 * stepRad * stepRad * stepRad * std::sin((phiIndex + 0.5) * stepRad) / (8.0 * std::acos(-1.0) * std::acos(-1.0)));
      }
    }
  }
  const auto mudConfig = RenderConfiguration(mudValues.get());
  auto densityConfig = RenderConfiguration(densityValues.get());
  densityConfig.grid.units = ODFValueUnits::CountDensity;
  const auto layout = ComputeODFSectionLayout(mudConfig);
  const auto mudResult = ODFSectionCompositor{}.generateCompositeImage(mudConfig);
  const auto densityResult = ODFSectionCompositor{}.generateCompositeImage(densityConfig);
  REQUIRE(mudResult);
  REQUIRE(densityResult);
  REQUIRE(mudResult.width == densityResult.width);
  REQUIRE(mudResult.height == densityResult.height);
  REQUIRE(mudResult.width == layout.pageWidth);
  REQUIRE(mudResult.height == layout.pageHeight);
  REQUIRE(mudResult.appliedMinimumMUD == Approx(densityResult.appliedMinimumMUD));
  REQUIRE(mudResult.appliedMaximumMUD == Approx(densityResult.appliedMaximumMUD));
  for(size_t sectionIndex = 0; sectionIndex < mudConfig.sectionCount; sectionIndex++)
  {
    const auto origin = GetODFSectionPanelOrigin(layout, sectionIndex);
    for(int32_t row = 0; row < layout.panelHeight; row++)
    {
      for(int32_t column = 0; column < layout.panelWidth; column++)
      {
        const size_t offset = (static_cast<size_t>(static_cast<int32_t>(origin[1]) + row) * mudResult.width + static_cast<int32_t>(origin[0]) + column) * 4;
        for(size_t componentIndex = 0; componentIndex < 4; componentIndex++)
        {
          REQUIRE(mudResult.image->getValue(offset + componentIndex) == densityResult.image->getValue(offset + componentIndex));
        }
      }
    }
  }
  REQUIRE_FALSE(HasInk(mudResult, 400, 28, 80, 16));
  REQUIRE(HasInk(densityResult, 400, 28, 80, 16));
  REQUIRE(HasPixelDifference(mudResult, densityResult, 378, 84, 148, 17));
  REQUIRE(HasInk(mudResult, 65, 108, 7, 6));
  REQUIRE(HasInk(mudResult, 86, 125, 20, 13));
}

TEST_CASE("ebsdlib::ODFSectionCompositor::AppliedScaleAndColorInterpolation", "[EbsdLib][ODFSectionCompositor]")
{
  auto values = DoubleArrayType::CreateArray(32, "Uniform", true);
  values->initializeWithValue(2.0);
  auto config = RenderConfiguration(values.get());
  std::array<int, 4> expected = {128, 0, 128, 255};
  double expectedMinimum = 0;
  double expectedMaximum = 4;
  SECTION("Linear midpoint")
  {
  }
  SECTION("Automatic uses displayed maximum")
  {
    config.scaleMode = ODFScaleMode::Automatic;
    config.manualMinimumMUD = std::numeric_limits<double>::quiet_NaN();
    expectedMaximum = 2;
    expected = {255, 0, 0, 255};
  }
  SECTION("Manual lower clipping")
  {
    config.manualMinimumMUD = 3;
    expectedMinimum = 3;
    expected = {0, 0, 255, 255};
  }
  SECTION("Explicit zero grid remains valid with manual scale")
  {
    values->initializeWithValue(0.0);
    expected = {0, 0, 255, 255};
  }
  SECTION("Manual upper clipping")
  {
    config.manualMaximumMUD = 1;
    expectedMaximum = 1;
    expected = {255, 0, 0, 255};
  }
  SECTION("Unequal control intervals")
  {
    config.colorControlPoints = {0, 0, 0, 1, 0.25f, 0, 1, 0, 1, 1, 0, 0};
    expected = {85, 170, 0, 255};
  }
  const auto result = ODFSectionCompositor{}.generateCompositeImage(config);
  REQUIRE(result);
  REQUIRE(result.appliedMinimumMUD == Approx(expectedMinimum));
  REQUIRE(result.appliedMaximumMUD == Approx(expectedMaximum));
  RequirePixel(result, 64, 76, expected);
}

TEST_CASE("ebsdlib::ODFSectionCompositor::AutomaticUsesCropAndInterpolation", "[EbsdLib][ODFSectionCompositor]")
{
  auto values = DoubleArrayType::CreateArray(32, "Cropped", true);
  values->initializeWithValue(100.0);
  for(size_t phi1Index = 0; phi1Index < 4; phi1Index++)
  {
    for(size_t phi2Index = 0; phi2Index < 4; phi2Index++)
    {
      const size_t flatIndex = phi1Index * 8 + phi2Index;
      values->setValue(flatIndex, static_cast<double>(phi2Index + 1));
    }
  }
  auto config = RenderConfiguration(values.get());
  config.grid.laueOpsIndex = CrystalStructure::Hexagonal_High;
  config.scaleMode = ODFScaleMode::Automatic;
  const auto result = ODFSectionCompositor{}.generateCompositeImage(config);
  REQUIRE(result);
  REQUIRE(result.sectionAnglesDeg == std::vector<double>{0, 15, 30, 45});
  REQUIRE(result.appliedMaximumMUD == Approx(2.5));
}

TEST_CASE("ebsdlib::ODFSectionCompositor::SpatialOrderAndUnusedSlot", "[EbsdLib][ODFSectionCompositor]")
{
  auto values = DoubleArrayType::CreateArray(32, "SpatialRamp", true);
  for(size_t phi1Index = 0; phi1Index < 4; phi1Index++)
  {
    for(size_t phiIndex = 0; phiIndex < 2; phiIndex++)
    {
      for(size_t phi2Index = 0; phi2Index < 4; phi2Index++)
      {
        const size_t flatIndex = (phi1Index * 2 + phiIndex) * 4 + phi2Index;
        values->setValue(flatIndex, static_cast<double>(phiIndex + phi1Index));
      }
    }
  }
  auto config = RenderConfiguration(values.get());
  config.sectionCount = 3;
  const auto result = ODFSectionCompositor{}.generateCompositeImage(config);
  REQUIRE(result);
  REQUIRE(result.sectionAnglesDeg == std::vector<double>{0, 120, 240});
  RequirePixel(result, 49, 60, {0, 0, 255, 255});
  RequirePixel(result, 81, 60, {64, 0, 191, 255});
  RequirePixel(result, 145, 60, {191, 0, 64, 255});
  RequirePixel(result, 49, 92, {64, 0, 191, 255});
  RequirePixel(result, 145, 92, {255, 0, 0, 255});
  RequirePixel(result, 266, 194, {255, 255, 255, 255});
  // Annotations must not cover any pixels inside the blue source cell.
  for(int32_t y = 46; y < 70; y++)
  {
    for(int32_t x = 35; x < 63; x++)
    {
      RequirePixel(result, x, y, {0, 0, 255, 255});
    }
  }
}
