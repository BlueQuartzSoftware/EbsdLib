#pragma once

#include <cstdint>
#include <string>
#include <utility>

#include "EbsdLib/EbsdLib.h"

namespace PngWriter
{
/**
 * @brief WriteColorImage Writes an RGB or RGBA image to a PNG file.
 * @param filepath Output file path
 * @param width Width of Image in pixels
 * @param height Height of Image in pixels
 * @param samplesPerPixel RGB=3, RGBA=4
 * @param data The image data, row-major, 8-bit per channel
 * @return Pair of (status, message). status == 0 on success.
 */
EbsdLib_EXPORT std::pair<int32_t, std::string> WriteColorImage(const std::string& filepath, int32_t width, int32_t height, uint16_t samplesPerPixel, const uint8_t* data);

/**
 * @brief WriteGrayScaleImage Writes a single-channel 8-bit image to a PNG file.
 * @param filepath Output file path
 * @param width Width of Image in pixels
 * @param height Height of Image in pixels
 * @param data The image data, row-major, 8-bit single channel
 * @return Pair of (status, message). status == 0 on success.
 */
EbsdLib_EXPORT std::pair<int32_t, std::string> WriteGrayScaleImage(const std::string& filepath, int32_t width, int32_t height, const uint8_t* data);

}; // namespace PngWriter
