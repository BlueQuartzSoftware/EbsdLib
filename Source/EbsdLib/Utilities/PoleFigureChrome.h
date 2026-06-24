/* ============================================================================
 * Copyright (c) 2009-2025 BlueQuartz Software, LLC
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
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#pragma once

#include <array>
#include <string>
#include <vector>

#include "EbsdLib/EbsdLib.h"
#include "EbsdLib/Utilities/PoleFigureCompositor.h" // CompositePoleFigureConfiguration_t, LayoutMetrics

namespace canvas_ity
{
class canvas;
} // namespace canvas_ity

namespace ebsdlib
{
/** @brief Fills the entire page with a white background. */
EbsdLib_EXPORT void DrawPoleFigureBackground(canvas_ity::canvas& context, const LayoutMetrics& layout);

/** @brief Draws the composite title at the top of the page. */
EbsdLib_EXPORT void DrawPoleFigureTitle(canvas_ity::canvas& context, const std::string& title, float pageWidth, float fontPtSize, float margins, const std::vector<unsigned char>& latoBold);

/** @brief Draws the phase/sample info block used in the legend slot for discrete figures. */
EbsdLib_EXPORT void DrawPoleFigureInfoBlock(canvas_ity::canvas& context, const CompositePoleFigureConfiguration_t& config, std::array<float, 2> position, float margins, float fontPtSize,
                                            const std::vector<unsigned char>& latoRegular);

/**
 * @brief Draws the bounding circle, X/Y axis lines, axis labels, and direction
 * subtitle for one pole figure at the given origin. Does NOT draw the figure data
 * (raster blit or markers) — the caller draws that first.
 */
EbsdLib_EXPORT void DrawPoleFigureFrame(canvas_ity::canvas& context, const CompositePoleFigureConfiguration_t& config, std::array<float, 2> origin, const std::string& poleFigureName, float fontPtSize,
                                        float margins, const std::vector<unsigned char>& latoBold, const std::vector<unsigned char>& firaSans);
} // namespace ebsdlib
