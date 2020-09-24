#pragma once

#include <cstdint>
#include <string>
#include <utility>

class TiffWriter
{
public:
  TiffWriter();
  ~TiffWriter();
  TiffWriter(const TiffWriter&) = delete;            // Copy Constructor Not Implemented
  TiffWriter(TiffWriter&&) = delete;                 // Move Constructor Not Implemented
  TiffWriter& operator=(const TiffWriter&) = delete; // Copy Assignment Not Implemented
  TiffWriter& operator=(TiffWriter&&) = delete;      // Move Assignment Not Implemented

  /**
   * @brief WriteColorImage Writes an RGB or RGBA image to a tiff file.
   * @param filepath Output file path
   * @param width Width of Image
   * @param height Height of Image
   * @param samplesPerPixel RGB=3, RGBA=4
   * @param data The image data to be written
   * @return
   */
  static std::pair<int32_t, std::string> WriteColorImage(const std::string& filepath, int32_t width, int32_t height, uint16_t samplesPerPixel, const uint8_t* data);
};
