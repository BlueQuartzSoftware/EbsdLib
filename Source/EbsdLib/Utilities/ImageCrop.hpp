/* ============================================================================
 * Copyright (c) 2009-2025 BlueQuartz Software, LLC
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#pragma once

#include "EbsdLib/Core/EbsdDataArray.hpp"
#include "EbsdLib/EbsdLib.h"

#include <cstdint>

namespace ebsdlib
{

/**
 * @brief Result of a CropImageToContent call: the cropped pixel buffer plus
 *        its dimensions. The image is laid out row-major with `channels`
 *        consecutive components per pixel (e.g. RGB: channels=3, RGBA: 4).
 */
struct EbsdLib_EXPORT CroppedImage
{
  UInt8ArrayType::Pointer image;
  int width = 0;
  int height = 0;
};

/**
 * @brief Crop an RGB or RGBA image down to the bounding box of its non-white
 *        pixels, with a configurable padding applied on each side and clamped
 *        to the canvas boundaries.
 *
 * The IPF triangle legend renderer paints a small SST wedge onto a much
 * larger square canvas (because the legend method takes only `canvasDim`
 * and the SST geometry is per-Laue-class). The result is a canvas with a
 * lot of whitespace, especially for hex/trig classes whose SST is a
 * narrow wedge of the unit circle. Calling this helper after rendering
 * produces an image whose dimensions track the painted content, similar
 * to how MTEX's IPF legend output is sized.
 *
 * Behaviour:
 *  - "White pixel" is exact (255, 255, 255). The legend painter fills its
 *    background with pure white, so this is sufficient.
 *  - If every pixel is white (degenerate case), the original canvas is
 *    returned unchanged rather than an empty image.
 *  - Padding is clamped: a padding so large it would push the bounding
 *    box past the canvas just yields the full canvas as output.
 *
 * @param src       Source image (canvasWidth*canvasHeight tuples, channels per pixel).
 * @param canvasWidth   Source image pixel width.
 * @param canvasHeight  Source image pixel height.
 * @param channels  Number of color components per pixel (3 for RGB, 4 for RGBA).
 * @param padding   Padding (in pixels) to add on each side of the bounding box.
 * @return CroppedImage holding a freshly allocated UInt8ArrayType + dimensions.
 */
CroppedImage EbsdLib_EXPORT CropImageToContent(const UInt8ArrayType* src, int canvasWidth, int canvasHeight, int channels, int padding);

} // namespace ebsdlib
