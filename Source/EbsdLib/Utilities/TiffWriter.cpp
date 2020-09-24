#include "TiffWriter.h"

#include <array>
#include <fstream>
#include <vector>

TiffWriter::TiffWriter() = default;

TiffWriter::~TiffWriter() = default;

using WORD = int16_t;
using DWORD = int32_t;

class TIFTAG
{
public:
  TIFTAG() = default;
  TIFTAG(int16_t tagId, int16_t dataType, int32_t dataCount, int32_t dataOffset)
  : m_TagId(tagId)
  , m_DataType(dataType)
  , m_DataCount(dataCount)
  , m_DataOffset(dataOffset)
  {
  }

  void write(std::ostream& out) const
  {
    out.write(reinterpret_cast<const char*>(&m_TagId), sizeof(int16_t));
    out.write(reinterpret_cast<const char*>(&m_DataType), sizeof(int16_t));
    out.write(reinterpret_cast<const char*>(&m_DataCount), sizeof(int32_t));
    out.write(reinterpret_cast<const char*>(&m_DataOffset), sizeof(int32_t));
  }

public:
  TIFTAG(const TIFTAG&) = default;
  TIFTAG(TIFTAG&&) = default;
  TIFTAG& operator=(const TIFTAG&) = default;
  TIFTAG& operator=(TIFTAG&&) = default;

private:
  int16_t m_TagId = 0;      /* The tag identifier  */
  int16_t m_DataType = 0;   /* The scalar type of the data items  */
  int32_t m_DataCount = 0;  /* The number of items in the tag data  */
  int32_t m_DataOffset = 0; /* The byte offset to the data items  */
};

namespace EbsdLib
{
namespace TiffTags
{

} // namespace TiffTags
} // namespace EbsdLib

// -----------------------------------------------------------------------------
std::pair<int32_t, std::string> TiffWriter::WriteColorImage(const std::string& filepath, int32_t width, int32_t height, uint16_t samplesPerPixel, const uint8_t* data)
{

  // Check for Endianess of the system and write the appropriate magic number according to the tiff spec
  uint32_t i = 0x01020304;
  uint8_t* u8 = reinterpret_cast<uint8_t*>(&i);
  bool bigEndian = false;
  std::array<char, 4> magicNumber = {0x49, 0x49, 0x2A, 0x00};

  if(u8[0] == 1)
  {
    bigEndian = true;
    magicNumber = {0x4D, 0x4D, 0x00, 0x2A}; // Big
  }

  // open file and write header
  std::ofstream outputFile(filepath.c_str(), std::ios::out | std::ios::binary);
  if(!outputFile.is_open())
  {
    return {-1, "Could not open output file for writing"};
  }

  outputFile.write(magicNumber.data(), 4);
  // Generate the offset into the Image File Directory (ifd) which we are going to write first
  uint32_t ifd_Offset = 8;
  outputFile.write(reinterpret_cast<char*>(&ifd_Offset), sizeof(uint32_t));

  std::vector<TIFTAG> tags;
  tags.push_back(TIFTAG(0x00FE, 0x0004, 1, 0x00000000));                       // NewSubfileType
  tags.push_back(TIFTAG(0x0100, 0x0004, 1, width));                            // ImageWidth
  tags.push_back(TIFTAG(0x0101, 0x0004, 1, height));                           // ImageLength
  tags.push_back(TIFTAG(0x0102, 0x0003, 1, 8 * sizeof(char)));                 // BitsPerSample
  tags.push_back(TIFTAG(0x0103, 0x0003, 1, 0x0001));                           // Compression
  tags.push_back(TIFTAG(0x0106, 0x0003, 1, 0x0002));                           // PhotometricInterpretation  // For SamplesPerPixel = 3 or 4 (RGB or RGBA)
  tags.push_back(TIFTAG(0x0112, 0x0003, 1, 1));                                // Orientation
  tags.push_back(TIFTAG(0x0115, 0x0003, 1, samplesPerPixel));                  // SamplesPerPixel
  tags.push_back(TIFTAG(0x0116, 0x0004, 1, height));                           // RowsPerStrip
  tags.push_back(TIFTAG(0x0117, 0x0004, 1, width * height * samplesPerPixel)); // StripByteCounts
  // TIFTAG XResolution;
  // TIFTAG YResolution;
  // TIFTAG ResolutionUnit;
  tags.push_back(TIFTAG(0x011c, 0x0003, 1, 0x0001)); // PlanarConfiguration

  // Now compute the offset to the image data so that we can put that into the tag.
  // THESE NEXT 2 LINES MUST BE THE LAST TAG TO BE PUSH'ED BACK INTO THE VECTOR OR THE MATH WILL BE WRONG
  int32_t imageDataOffset = 8 + ((tags.size() + 1) * 12) + 6; // Header + tags + IDF Tag entry count and Next IFD Offset
  tags.push_back(TIFTAG(0x0111, 0x0004, 1, imageDataOffset)); // StripOffsets

  // Write the number of tags to the IFD section
  uint16_t numEntries = static_cast<uint16_t>(tags.size());
  outputFile.write(reinterpret_cast<const char*>(&numEntries), sizeof(numEntries));
  // write the tags to the file.
  for(const auto& tag : tags)
  {
    tag.write(outputFile);
  }
  // Write the "Next Tag Offset"
  uint32_t nextOffset = 0;
  outputFile.write(reinterpret_cast<const char*>(&nextOffset), sizeof(nextOffset));

  // Now write the actual image data
  int32_t imageByteCount = width * height * samplesPerPixel;
  outputFile.write(reinterpret_cast<const char*>(data), imageByteCount);

  // and we are done.
  return {0, "No Error"};
}
