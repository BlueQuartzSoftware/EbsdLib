#include <catch2/catch.hpp>

#include "EbsdLib/Utilities/ColorSpaceUtils.hpp"

TEST_CASE("ebsdlib::ColorSpaceUtils::HslToRgb", "[EbsdLib][ColorSpaceUtils]")
{
  SECTION("Pure Red")
  {
    auto [r, g, b] = ebsdlib::color::hslToRgb(0.0, 1.0, 0.5);
    REQUIRE(r == Approx(1.0).margin(1e-6));
    REQUIRE(g == Approx(0.0).margin(1e-6));
    REQUIRE(b == Approx(0.0).margin(1e-6));
  }
  SECTION("Pure Green")
  {
    auto [r, g, b] = ebsdlib::color::hslToRgb(1.0 / 3.0, 1.0, 0.5);
    REQUIRE(r == Approx(0.0).margin(1e-6));
    REQUIRE(g == Approx(1.0).margin(1e-6));
    REQUIRE(b == Approx(0.0).margin(1e-6));
  }
  SECTION("Pure Blue")
  {
    auto [r, g, b] = ebsdlib::color::hslToRgb(2.0 / 3.0, 1.0, 0.5);
    REQUIRE(r == Approx(0.0).margin(1e-6));
    REQUIRE(g == Approx(0.0).margin(1e-6));
    REQUIRE(b == Approx(1.0).margin(1e-6));
  }
  SECTION("White")
  {
    auto [r, g, b] = ebsdlib::color::hslToRgb(0.0, 0.0, 1.0);
    REQUIRE(r == Approx(1.0).margin(1e-6));
    REQUIRE(g == Approx(1.0).margin(1e-6));
    REQUIRE(b == Approx(1.0).margin(1e-6));
  }
  SECTION("Black")
  {
    auto [r, g, b] = ebsdlib::color::hslToRgb(0.0, 0.0, 0.0);
    REQUIRE(r == Approx(0.0).margin(1e-6));
    REQUIRE(g == Approx(0.0).margin(1e-6));
    REQUIRE(b == Approx(0.0).margin(1e-6));
  }
  SECTION("50% Gray")
  {
    auto [r, g, b] = ebsdlib::color::hslToRgb(0.0, 0.0, 0.5);
    REQUIRE(r == Approx(0.5).margin(1e-6));
    REQUIRE(g == Approx(0.5).margin(1e-6));
    REQUIRE(b == Approx(0.5).margin(1e-6));
  }
  SECTION("Yellow (H=60deg)")
  {
    auto [r, g, b] = ebsdlib::color::hslToRgb(1.0 / 6.0, 1.0, 0.5);
    REQUIRE(r == Approx(1.0).margin(1e-6));
    REQUIRE(g == Approx(1.0).margin(1e-6));
    REQUIRE(b == Approx(0.0).margin(1e-6));
  }
}

TEST_CASE("ebsdlib::ColorSpaceUtils::HslToHsv", "[EbsdLib][ColorSpaceUtils]")
{
  SECTION("Full saturation, mid lightness -> V=1, S=1")
  {
    auto [h, s, v] = ebsdlib::color::hslToHsv(0.0, 1.0, 0.5);
    REQUIRE(h == Approx(0.0));
    REQUIRE(s == Approx(1.0));
    REQUIRE(v == Approx(1.0));
  }
  SECTION("Zero saturation -> S_hsv = 0")
  {
    auto [h, s, v] = ebsdlib::color::hslToHsv(0.5, 0.0, 0.5);
    REQUIRE(s == Approx(0.0));
    REQUIRE(v == Approx(0.5));
  }
}

TEST_CASE("ebsdlib::ColorSpaceUtils::RoundTrip", "[EbsdLib][ColorSpaceUtils]")
{
  for(double hue = 0.0; hue < 1.0; hue += 0.1)
  {
    auto [r, g, b] = ebsdlib::color::hslToRgb(hue, 1.0, 0.5);
    REQUIRE(r >= 0.0);
    REQUIRE(r <= 1.0);
    REQUIRE(g >= 0.0);
    REQUIRE(g <= 1.0);
    REQUIRE(b >= 0.0);
    REQUIRE(b <= 1.0);
    double maxVal = std::max({r, g, b});
    REQUIRE(maxVal == Approx(1.0).margin(1e-6));
  }
}
