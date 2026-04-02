
#include <catch2/catch.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <string>

namespace ebsdlib::unit_test
{
using usize = size_t;
inline constexpr float EPSILON = 0.0001;

// template <class T>
// std::string ComputeMD5Hash(const std::vector<T>& outputDataArray)
// {
//   const T* dataPtr = outputDataArray.data();
//   usize arraySize = outputDataArray.size();
//   MD5 md5;
//   md5.update(reinterpret_cast<const uint8*>(dataPtr), arraySize * sizeof(T));
//   md5.finalize();
//   return md5.hexdigest();
// }

/**
 * @brief This class will decompress a tar.gz file using the locally installed copy of cmake and when
 * then class goes out of scope the extracted contents will be deleted from disk.
 */
class TestFileSentinel
{
public:
  /**
   * @brief Construct a File Sentinel object that will decompress on construction and remove the
   * contents on destruction.
   *
   * @param testFilesDir The directory where the archive is located
   * @param inputArchiveName The full name of the archive. The location is assumed to be in the TestFiles directory
   * @param expectedTopLevelOutput The name of the decompressed folder or file. WARNING: This assumes
   * that only a single file or single directory are part of the archive. In the case of a directory, the
   * directory itself can have as many subdirectories as needed.
   * @param decompressFiles Decompress the archive
   * @param removeTemp delete files that were decompressed
   */
  TestFileSentinel(std::string testFilesDir, std::string inputArchiveName, std::string expectedTopLevelOutput, bool decompressFiles = true, bool removeTemp = true);

  ~TestFileSentinel();

  TestFileSentinel(const TestFileSentinel&) = delete;            // Copy Constructor Not Implemented
  TestFileSentinel(TestFileSentinel&&) = delete;                 // Move Constructor Not Implemented
  TestFileSentinel& operator=(const TestFileSentinel&) = delete; // Copy Assignment Not Implemented
  TestFileSentinel& operator=(TestFileSentinel&&) = delete;      // Move Assignment Not Implemented

  /**
   * @brief Does the actual decompression of the archive.
   * @return
   */
  std::error_code decompress();

private:
  std::string m_TestFilesDir;
  std::string m_InputArchiveName;
  std::string m_ExpectedTopLevelOutput;
  bool m_Decompress;
  bool m_RemoveTemp;
};

} // namespace ebsdlib::unit_test
