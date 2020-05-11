/* ============================================================================
 * Copyright (c) 2009-2016 BlueQuartz Software, LLC
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
 * The code contained herein was partially funded by the followig contracts:
 *    United States Air Force Prime Contract FA8650-07-D-5800
 *    United States Air Force Prime Contract FA8650-10-D-5210
 *    United States Prime Contract Navy N00173-07-C-2068
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include "ColorUtilities.h"

#include "EbsdLib/Math/EbsdLibMath.h"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
ColorUtilities::ColorUtilities() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
ColorUtilities::~ColorUtilities() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdLib::Rgb ColorUtilities::ConvertHSVtoRgb(float h, float s, float v)
{
  // hsv to rgb (from wikipedia hsv/hsl page)
  float c = v * s;
  h = h * 6.0f;
  float x = c * (1.0f - fabs(fmod(h, 2.0f) - 1.0f));
  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;

  if(h >= 0.0f)
  {
    if(h < 1.0f)
    {
      r = c;
      g = x;
    }
    else if(h < 2.0f)
    {
      r = x;
      g = c;
    }
    else if(h < 3.0f)
    {
      g = c;
      b = x;
    }
    else if(h < 4.0f)
    {
      g = x;
      b = c;
    }
    else if(h < 5.0f)
    {
      r = x;
      b = c;
    }
    else if(h < 6.0f)
    {
      r = c;
      b = x;
    }
  }

  // adjust lumosity
  r = r + (v - c);
  g = g + (v - c);
  b = b + (v - c);

  if(r > 1.0f)
  {
    r = 1.0f;
  }
  if(g > 1.0f)
  {
    g = 1.0f;
  }
  if(b > 1.0f)
  {
    b = 1.0f;
  }
  if(r < 0.0f)
  {
    r = 0.0f;
  }
  if(g < 0.0f)
  {
    g = 0.0f;
  }
  if(b < 0.0f)
  {
    b = 0.0f;
  }

  return RgbColor::dRgb(r * 255, g * 255, b * 255, 0);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdLib::Rgb ColorUtilities::Hsv2Rgb(float h, float s, float v)
{
  double hh, p, q, t, ff;
  long i;
  struct
  {
    double r; // a fraction between 0 and 1
    double g; // a fraction between 0 and 1
    double b; // a fraction between 0 and 1
  } out;

  struct
  {
    double h; // angle in degrees
    double s; // a fraction between 0 and 1
    double v; // a fraction between 0 and 1
  } in;
  in.h = h;
  in.s = s;
  in.v = v;

  if(in.s <= 0.0)
  { // < is bogus, just shuts up warnings
    out.r = in.v;
    out.g = in.v;
    out.b = in.v;
    return RgbColor::dRgb(out.r * 255, out.g * 255, out.b * 255, 255);
  }
  hh = in.h;
  if(hh >= 360.0)
  {
    hh = 0.0;
  }
  hh /= 60.0;
  i = (long)hh;
  ff = hh - i;
  p = in.v * (1.0 - in.s);
  q = in.v * (1.0 - (in.s * ff));
  t = in.v * (1.0 - (in.s * (1.0 - ff)));

  switch(i)
  {
  case 0:
    out.r = in.v;
    out.g = t;
    out.b = p;
    break;
  case 1:
    out.r = q;
    out.g = in.v;
    out.b = p;
    break;
  case 2:
    out.r = p;
    out.g = in.v;
    out.b = t;
    break;

  case 3:
    out.r = p;
    out.g = q;
    out.b = in.v;
    break;
  case 4:
    out.r = t;
    out.g = p;
    out.b = in.v;
    break;
  case 5:
  default:
    out.r = in.v;
    out.g = p;
    out.b = q;
    break;
  }
  return RgbColor::dRgb(out.r * 255, out.g * 255, out.b * 255, 255);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QVector<EbsdLib::Rgb> ColorUtilities::GenerateColors(int count, int saturation, int value)
{
  QVector<EbsdLib::Rgb> colors(count);
  float s = static_cast<float>(saturation) / 255.0;
  float v = static_cast<float>(value) / 255.0;
  float increment = 360.0 / count;
  for(float i = 0; i < static_cast<float>(count); i = i + 1.0f)
  {
    EbsdLib::Rgb c = ColorUtilities::Hsv2Rgb(i * increment, s, v);
    colors[i] = c;
  }
  return colors;
}
