#include <catch2/catch.hpp>

#include "EbsdLib/Utilities/ColorTable.h"
#include "EbsdLib/Utilities/ColorUtilities.h"

#include <cmath>
#include <vector>

using namespace ebsdlib;

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ColorTableTest::RgbColor_dRgb_RoundTrip", "[EbsdLib][ColorTableTest]")
{
  int r = 128, g = 64, b = 32, a = 255;
  Rgb color = RgbColor::dRgb(r, g, b, a);
  REQUIRE(RgbColor::dRed(color) == r);
  REQUIRE(RgbColor::dGreen(color) == g);
  REQUIRE(RgbColor::dBlue(color) == b);
  REQUIRE(RgbColor::dAlpha(color) == a);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ColorTableTest::RgbColor_dRgb_Extremes", "[EbsdLib][ColorTableTest]")
{
  // Pure black with full alpha
  Rgb black = RgbColor::dRgb(0, 0, 0, 255);
  REQUIRE(RgbColor::dRed(black) == 0);
  REQUIRE(RgbColor::dGreen(black) == 0);
  REQUIRE(RgbColor::dBlue(black) == 0);
  REQUIRE(RgbColor::dAlpha(black) == 255);

  // Pure white with full alpha
  Rgb white = RgbColor::dRgb(255, 255, 255, 255);
  REQUIRE(RgbColor::dRed(white) == 255);
  REQUIRE(RgbColor::dGreen(white) == 255);
  REQUIRE(RgbColor::dBlue(white) == 255);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ColorTableTest::RgbColor_dGray", "[EbsdLib][ColorTableTest]")
{
  // The formula is: (R*11 + G*16 + B*5) / 32
  Rgb color = RgbColor::dRgb(100, 150, 200, 255);
  int expected = (100 * 11 + 150 * 16 + 200 * 5) / 32;
  REQUIRE(RgbColor::dGray(color) == expected);

  // Pure white should be close to 255
  Rgb white = RgbColor::dRgb(255, 255, 255, 255);
  REQUIRE(RgbColor::dGray(white) == 255);

  // Pure black should be 0
  Rgb black = RgbColor::dRgb(0, 0, 0, 255);
  REQUIRE(RgbColor::dGray(black) == 0);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ColorTableTest::RgbColor_fRgb", "[EbsdLib][ColorTableTest]")
{
  Rgb color = RgbColor::dRgb(255, 128, 0, 255);
  auto [fr, fg, fb] = RgbColor::fRgb(color);
  REQUIRE(fr == Approx(1.0f));
  REQUIRE(fg == Approx(128.0f / 255.0f));
  REQUIRE(fb == Approx(0.0f));
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ColorTableTest::RgbColor_compare", "[EbsdLib][ColorTableTest]")
{
  Rgb a = RgbColor::dRgb(100, 200, 50, 255);
  Rgb b = RgbColor::dRgb(100, 200, 50, 255);
  Rgb c = RgbColor::dRgb(100, 200, 51, 255);

  REQUIRE(RgbColor::compare(a, b) == true);
  REQUIRE(RgbColor::compare(a, c) == false);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ColorTableTest::GetColorTable", "[EbsdLib][ColorTableTest]")
{
  int numColors = 10;
  std::vector<float> colors(3 * numColors, 0.0f);
  EbsdColorTable::GetColorTable(numColors, colors);

  // All values should be in [0, 1] range
  for(size_t i = 0; i < colors.size(); i++)
  {
    CHECK(colors[i] >= 0.0f);
    CHECK(colors[i] <= 1.0f);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ColorTableTest::GetColorTable_SingleColor", "[EbsdLib][ColorTableTest]")
{
  std::vector<float> colors(3, 0.0f);
  EbsdColorTable::GetColorTable(1, colors);
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ColorTableTest::ConvertHSVtoRgb_KnownValues", "[EbsdLib][ColorTableTest]")
{
  // Red: h=0, s=1, v=1
  {
    Rgb color = ColorUtilities::ConvertHSVtoRgb(0.0f, 1.0f, 1.0f);
    REQUIRE(RgbColor::dRed(color) == 255);
    REQUIRE(RgbColor::dGreen(color) == 0);
    REQUIRE(RgbColor::dBlue(color) == 0);
  }

  // Green: h=0.333, s=1, v=1
  {
    Rgb color = ColorUtilities::ConvertHSVtoRgb(1.0f / 3.0f, 1.0f, 1.0f);
    REQUIRE(RgbColor::dRed(color) == 0);
    REQUIRE(RgbColor::dGreen(color) == 255);
    CHECK(RgbColor::dBlue(color) <= 1); // allow rounding
  }

  // Blue: h=0.667, s=1, v=1
  {
    Rgb color = ColorUtilities::ConvertHSVtoRgb(2.0f / 3.0f, 1.0f, 1.0f);
    CHECK(RgbColor::dRed(color) <= 1);
    REQUIRE(RgbColor::dGreen(color) == 0);
    REQUIRE(RgbColor::dBlue(color) == 255);
  }

  // White: s=0, v=1
  {
    Rgb color = ColorUtilities::ConvertHSVtoRgb(0.0f, 0.0f, 1.0f);
    REQUIRE(RgbColor::dRed(color) == 255);
    REQUIRE(RgbColor::dGreen(color) == 255);
    REQUIRE(RgbColor::dBlue(color) == 255);
  }

  // Black: v=0
  {
    Rgb color = ColorUtilities::ConvertHSVtoRgb(0.0f, 1.0f, 0.0f);
    REQUIRE(RgbColor::dRed(color) == 0);
    REQUIRE(RgbColor::dGreen(color) == 0);
    REQUIRE(RgbColor::dBlue(color) == 0);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ColorTableTest::Hsv2Rgb_KnownValues", "[EbsdLib][ColorTableTest]")
{
  // Red: h=0 degrees, s=1, v=1
  {
    Rgb color = ColorUtilities::Hsv2Rgb(0.0f, 1.0f, 1.0f);
    REQUIRE(RgbColor::dRed(color) == 255);
    REQUIRE(RgbColor::dGreen(color) == 0);
    REQUIRE(RgbColor::dBlue(color) == 0);
  }

  // Green: h=120 degrees, s=1, v=1
  {
    Rgb color = ColorUtilities::Hsv2Rgb(120.0f, 1.0f, 1.0f);
    REQUIRE(RgbColor::dRed(color) == 0);
    REQUIRE(RgbColor::dGreen(color) == 255);
    CHECK(RgbColor::dBlue(color) <= 1);
  }

  // Blue: h=240 degrees, s=1, v=1
  {
    Rgb color = ColorUtilities::Hsv2Rgb(240.0f, 1.0f, 1.0f);
    CHECK(RgbColor::dRed(color) <= 1);
    REQUIRE(RgbColor::dGreen(color) == 0);
    REQUIRE(RgbColor::dBlue(color) == 255);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ColorTableTest::GenerateColors", "[EbsdLib][ColorTableTest]")
{
  int count = 12;
  auto colors = ColorUtilities::GenerateColors(count);

  REQUIRE(colors.size() == static_cast<size_t>(count));

  for(const auto& color : colors)
  {
    // Alpha should be 255 (opaque)
    REQUIRE(RgbColor::dAlpha(color) == 255);

    // RGB values in valid range
    CHECK(RgbColor::dRed(color) >= 0);
    CHECK(RgbColor::dRed(color) <= 255);
    CHECK(RgbColor::dGreen(color) >= 0);
    CHECK(RgbColor::dGreen(color) <= 255);
    CHECK(RgbColor::dBlue(color) >= 0);
    CHECK(RgbColor::dBlue(color) <= 255);
  }
}

// -----------------------------------------------------------------------------
TEST_CASE("ebsdlib::ColorTableTest::GenerateColors_Single", "[EbsdLib][ColorTableTest]")
{
  auto colors = ColorUtilities::GenerateColors(1);
  REQUIRE(colors.size() == 1);
}
