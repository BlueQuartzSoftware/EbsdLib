/* ============================================================================
 * Copyright (c) 2009-2025 BlueQuartz Software, LLC
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include "EbsdLib/Utilities/ImageCrop.hpp"

#include <algorithm>
#include <cstring>

namespace ebsdlib
{

CroppedImage CropImageToContent(const UInt8ArrayType* src, int canvasWidth, int canvasHeight, int channels, int padding)
{
  CroppedImage out;
  if(src == nullptr || canvasWidth <= 0 || canvasHeight <= 0 || (channels != 3 && channels != 4))
  {
    return out;
  }

  const uint8_t* p = src->getPointer(0);

  // Find the bounding box of non-white pixels. "White" = (255, 255, 255).
  int minX = canvasWidth;
  int minY = canvasHeight;
  int maxX = -1;
  int maxY = -1;
  for(int y = 0; y < canvasHeight; ++y)
  {
    for(int x = 0; x < canvasWidth; ++x)
    {
      const size_t idx = (static_cast<size_t>(y) * static_cast<size_t>(canvasWidth) + static_cast<size_t>(x)) * static_cast<size_t>(channels);
      const bool isWhite = (p[idx] == 255 && p[idx + 1] == 255 && p[idx + 2] == 255);
      if(!isWhite)
      {
        minX = std::min(minX, x);
        minY = std::min(minY, y);
        maxX = std::max(maxX, x);
        maxY = std::max(maxY, y);
      }
    }
  }

  // Degenerate case: image is all white. Return original unchanged.
  if(maxX < 0)
  {
    out.width = canvasWidth;
    out.height = canvasHeight;
    out.image = UInt8ArrayType::CreateArray(static_cast<size_t>(canvasWidth) * static_cast<size_t>(canvasHeight), std::vector<size_t>{static_cast<size_t>(channels)}, src->getName(), true);
    std::memcpy(out.image->getPointer(0), p, static_cast<size_t>(canvasWidth) * static_cast<size_t>(canvasHeight) * static_cast<size_t>(channels));
    return out;
  }

  // Apply padding and clamp to canvas.
  minX = std::max(0, minX - padding);
  minY = std::max(0, minY - padding);
  maxX = std::min(canvasWidth - 1, maxX + padding);
  maxY = std::min(canvasHeight - 1, maxY + padding);

  out.width = maxX - minX + 1;
  out.height = maxY - minY + 1;
  out.image = UInt8ArrayType::CreateArray(static_cast<size_t>(out.width) * static_cast<size_t>(out.height), std::vector<size_t>{static_cast<size_t>(channels)}, src->getName() + "_cropped", true);
  uint8_t* dst = out.image->getPointer(0);
  for(int y = 0; y < out.height; ++y)
  {
    const size_t srcRowStart = (static_cast<size_t>(minY + y) * static_cast<size_t>(canvasWidth) + static_cast<size_t>(minX)) * static_cast<size_t>(channels);
    const size_t dstRowStart = static_cast<size_t>(y) * static_cast<size_t>(out.width) * static_cast<size_t>(channels);
    std::memcpy(dst + dstRowStart, p + srcRowStart, static_cast<size_t>(out.width) * static_cast<size_t>(channels));
  }
  return out;
}

} // namespace ebsdlib
