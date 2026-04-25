#include "EbsdLib/Utilities/PngWriter.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

namespace PngWriter
{
std::pair<int32_t, std::string> WriteColorImage(const std::string& filepath, int32_t width, int32_t height, uint16_t samplesPerPixel, const uint8_t* data)
{
  if(width <= 0 || height <= 0 || data == nullptr)
  {
    return {-1, "PngWriter::WriteColorImage: invalid dimensions or null data"};
  }
  if(samplesPerPixel != 3 && samplesPerPixel != 4)
  {
    return {-1, "PngWriter::WriteColorImage: samplesPerPixel must be 3 (RGB) or 4 (RGBA)"};
  }
  const int strideBytes = width * samplesPerPixel;
  const int ok = stbi_write_png(filepath.c_str(), width, height, samplesPerPixel, data, strideBytes);
  if(ok == 0)
  {
    return {-1, "PngWriter::WriteColorImage: stbi_write_png failed for path " + filepath};
  }
  return {0, "OK"};
}

std::pair<int32_t, std::string> WriteGrayScaleImage(const std::string& filepath, int32_t width, int32_t height, const uint8_t* data)
{
  if(width <= 0 || height <= 0 || data == nullptr)
  {
    return {-1, "PngWriter::WriteGrayScaleImage: invalid dimensions or null data"};
  }
  const int strideBytes = width;
  const int ok = stbi_write_png(filepath.c_str(), width, height, 1, data, strideBytes);
  if(ok == 0)
  {
    return {-1, "PngWriter::WriteGrayScaleImage: stbi_write_png failed for path " + filepath};
  }
  return {0, "OK"};
}
} // namespace PngWriter
