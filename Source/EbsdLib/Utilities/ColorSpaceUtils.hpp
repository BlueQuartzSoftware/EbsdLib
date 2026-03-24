#pragma once

#include "EbsdLib/EbsdLib.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace ebsdlib
{
namespace color
{

/**
 * @brief Convert HSL to RGB. All inputs and outputs in [0, 1].
 * @param h Hue in [0, 1) where 0=red, 1/3=green, 2/3=blue
 * @param s Saturation in [0, 1]
 * @param l Lightness in [0, 1]
 * @return {r, g, b} each in [0, 1]
 */
inline std::array<double, 3> hslToRgb(double h, double s, double l)
{
  double c = (1.0 - std::abs(2.0 * l - 1.0)) * s;
  double hp = h * 6.0;
  double x = c * (1.0 - std::abs(std::fmod(hp, 2.0) - 1.0));
  double m = l - c / 2.0;

  double r1 = 0.0;
  double g1 = 0.0;
  double b1 = 0.0;

  if(hp < 1.0)
  {
    r1 = c;
    g1 = x;
    b1 = 0.0;
  }
  else if(hp < 2.0)
  {
    r1 = x;
    g1 = c;
    b1 = 0.0;
  }
  else if(hp < 3.0)
  {
    r1 = 0.0;
    g1 = c;
    b1 = x;
  }
  else if(hp < 4.0)
  {
    r1 = 0.0;
    g1 = x;
    b1 = c;
  }
  else if(hp < 5.0)
  {
    r1 = x;
    g1 = 0.0;
    b1 = c;
  }
  else
  {
    r1 = c;
    g1 = 0.0;
    b1 = x;
  }

  return {std::clamp(r1 + m, 0.0, 1.0), std::clamp(g1 + m, 0.0, 1.0), std::clamp(b1 + m, 0.0, 1.0)};
}

/**
 * @brief Convert HSL to HSV. All inputs and outputs in [0, 1].
 */
inline std::array<double, 3> hslToHsv(double h, double s, double l)
{
  double l2 = 2.0 * l;
  double s2 = s * ((l2 <= 1.0) ? l2 : (2.0 - l2));
  double v = (l2 + s2) / 2.0;
  double sv = (l2 + s2 > 1e-12) ? (2.0 * s2 / (l2 + s2)) : 0.0;
  return {h, sv, v};
}

/**
 * @brief Convert RGB [0,1] to 8-bit [0,255] clamped.
 */
inline std::array<uint8_t, 3> rgbToBytes(double r, double g, double b)
{
  return {static_cast<uint8_t>(std::clamp(r * 255.0, 0.0, 255.0)), static_cast<uint8_t>(std::clamp(g * 255.0, 0.0, 255.0)), static_cast<uint8_t>(std::clamp(b * 255.0, 0.0, 255.0))};
}

} // namespace color
} // namespace ebsdlib
