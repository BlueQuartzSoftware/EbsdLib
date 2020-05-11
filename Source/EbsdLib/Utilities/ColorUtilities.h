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

#pragma once

#include <QtCore/QVector>

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Core/EbsdLibConstants.h"
#include "EbsdLib/Utilities/ColorTable.h"

class EbsdLib_EXPORT ColorUtilities
{
public:
  ~ColorUtilities();

  /**
   * @brief
   * @param h 0-1 Range
   * @param s 0-1 Range
   * @param v 0-1 Range
   * @return Rgb Conversion
   */
  static EbsdLib::Rgb ConvertHSVtoRgb(float h, float s, float v);

  /**
   * @brief
   * @param h angle in degrees
   * @param s a fraction between 0 and 1
   * @param v a fraction between 0 and 1
   * @return Rgb Conversion
   */
  static EbsdLib::Rgb Hsv2Rgb(float h, float s, float v);

  /**
   * @brief Generates a vector of colors based on the HSV color wheel. The colors
   * are evenly distributed around the color wheel by taking 360/count.
   * @param count The total number of colors
   * @param saturation Range between 0-255
   * @param value      Range between 0-255
   * @return
   */
  static QVector<EbsdLib::Rgb> GenerateColors(int count, int saturation = 255, int value = 255);

protected:
  ColorUtilities();
};
