/* ============================================================================
 * Copyright (c) 2009-2019 BlueQuartz Software, LLC
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
 * The code contained herein was partially funded by the following contracts:
 *    United States Air Force Prime Contract FA8650-07-D-5800
 *    United States Air Force Prime Contract FA8650-10-D-5210
 *    United States Prime Contract Navy N00173-07-C-2068
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include "ColorTable.h"

#include <algorithm>
#include <iostream>

//#include <QtCore/QJsonArray>

using namespace EbsdLib;
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdColorTable::EbsdColorTable() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
EbsdColorTable::~EbsdColorTable() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void EbsdColorTable::GetColorTable(int numColors, std::vector<float>& colorsOut)
{
  static const int numColorNodes = 8;
  float color[numColorNodes][3] = {
      {0.0F, 0.0F / 255.0F, 255.0F / 255.0F},            // blue
      {105.0F / 255.0F, 145.0F / 255.0F, 2.0F / 255.0F}, // yellow
      {0.0F / 255.0F, 255.0F / 255.0F, 29.0F / 255.0F},  // Green
      {180.0F / 255.0F, 255.0F / 255.0F, 0.0F / 255.0F},
      {255.0F / 255.0F, 215.0F / 255.0F, 6.0F / 255.0F},
      {255.0F / 255.0F, 143.0F / 255.0F, 1.0F / 255.0F},
      {255.0F / 255.0F, 69.0F / 255.0F, 0.0F / 255.0F},
      {255.0F / 255.0F, 0.0F / 255.0F, 0.0F / 255.0F} // red
  };

  static const int maxNodeIndex = numColorNodes - 1;
  const float stepSize = 1.0F / numColors;
  const float nodeStepSize = 1.0F / (maxNodeIndex);
  for(int i = 0; i < numColors; i++)
  {
    float pos = i * stepSize; // [0, 1] range
    int currColorBin = static_cast<int>(pos / nodeStepSize);
    float currFraction = (pos / nodeStepSize) - currColorBin;

    float r;
    float g;
    float b;
    currColorBin = std::min(currColorBin, maxNodeIndex);
    // currColorBin + 1 causes this to step out of color[] bounds when currColorBin == (numColorNodes - 1)
    if(i < numColors - 1)
    {
      r = color[currColorBin][0] * (1.0F - currFraction) + color[currColorBin + 1][0] * currFraction;
      g = color[currColorBin][1] * (1.0F - currFraction) + color[currColorBin + 1][1] * currFraction;
      b = color[currColorBin][2] * (1.0F - currFraction) + color[currColorBin + 1][2] * currFraction;
    }
    else
    {
      r = color[currColorBin][0];
      g = color[currColorBin][1];
      b = color[currColorBin][2];
    }
    colorsOut[3 * i] = r;
    colorsOut[3 * i + 1] = g;
    colorsOut[3 * i + 2] = b;
  }
}
