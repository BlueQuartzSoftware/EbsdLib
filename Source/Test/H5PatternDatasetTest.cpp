#include <catch2/catch.hpp>

#include "H5Support/H5ScopedSentinel.h"

#include "EbsdLib/IO/BrukerNano/EspritConstants.h"
#include "EbsdLib/IO/BrukerNano/H5EspritReader.h"
#include "EbsdLib/IO/TSL/AngConstants.h"
#include "EbsdLib/IO/TSL/H5OIMReader.h"
#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

#include <array>
#include <filesystem>
#include <string>
#include <vector>

using namespace ebsdlib;

namespace
{
enum class PatternCase
{
  Valid,
  Rank2,
  FewerPatterns,
  FloatingPoint,
  Unreadable
};

/**
 * @brief Writes a pattern dataset of the given shape into a disposable copy of a scan. Each pattern is filled
 * with its own file tuple index (mod 251). When externalFile is not empty the dataset is stored in that
 * external file, which is never written, so reading the dataset fails.
 */
void WritePatternDataset(const std::filesystem::path& filePath, const std::string& datasetPath, const std::vector<hsize_t>& dims, hid_t fileType, const std::string& externalFile)
{
  const hid_t fileId = H5Fopen(filePath.string().c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
  REQUIRE(fileId >= 0);
  H5Support::H5ScopedFileSentinel sentinel(fileId, true);
  const hid_t spaceId = H5Screate_simple(static_cast<int>(dims.size()), dims.data(), nullptr);
  REQUIRE(spaceId >= 0);
  const hid_t dcplId = H5Pcreate(H5P_DATASET_CREATE);
  REQUIRE(dcplId >= 0);
  if(!externalFile.empty())
  {
    REQUIRE(H5Pset_external(dcplId, externalFile.c_str(), 0, H5F_UNLIMITED) >= 0);
  }
  const hid_t datasetId = H5Dcreate2(fileId, datasetPath.c_str(), fileType, spaceId, H5P_DEFAULT, dcplId, H5P_DEFAULT);
  REQUIRE(datasetId >= 0);
  if(externalFile.empty())
  {
    size_t numElements = 1;
    for(hsize_t dim : dims)
    {
      numElements *= static_cast<size_t>(dim);
    }
    const size_t patternSize = numElements / static_cast<size_t>(dims[0]);
    std::vector<uint8_t> values(numElements);
    for(size_t i = 0; i < numElements; i++)
    {
      values[i] = static_cast<uint8_t>((i / patternSize) % 251);
    }
    REQUIRE(H5Dwrite(datasetId, H5T_NATIVE_UINT8, H5S_ALL, H5S_ALL, H5P_DEFAULT, values.data()) >= 0);
  }
  REQUIRE(H5Dclose(datasetId) >= 0);
  REQUIRE(H5Pclose(dcplId) >= 0);
  REQUIRE(H5Sclose(spaceId) >= 0);
}

/**
 * @brief Copies a scan file and adds a pattern dataset for the requested case.
 * @return The expected error code (0 for the valid case) and a fragment of the expected message.
 */
std::pair<int, std::string> PreparePatternFile(PatternCase patternCase, const std::filesystem::path& inputFile, const std::filesystem::path& outputPath, const std::string& datasetPath,
                                               hsize_t numPoints, int firstErrorCode)
{
  const std::filesystem::path missingExternalPath = outputPath.parent_path() / (outputPath.stem().string() + "_missing.raw");
  std::filesystem::create_directories(outputPath.parent_path());
  std::filesystem::remove(missingExternalPath);
  std::filesystem::copy_file(inputFile, outputPath, std::filesystem::copy_options::overwrite_existing);
  switch(patternCase)
  {
  case PatternCase::Valid:
    WritePatternDataset(outputPath, datasetPath, {numPoints, 3, 2}, H5T_NATIVE_UINT8, "");
    return {0, ""};
  case PatternCase::Rank2:
    WritePatternDataset(outputPath, datasetPath, {numPoints, 6}, H5T_NATIVE_UINT8, "");
    return {firstErrorCode, "has 2 dimensions"};
  case PatternCase::FewerPatterns:
    WritePatternDataset(outputPath, datasetPath, {10, 3, 2}, H5T_NATIVE_UINT8, "");
    return {firstErrorCode - 1, "holds 10 patterns but the scan has " + std::to_string(numPoints) + " points"};
  case PatternCase::FloatingPoint:
    WritePatternDataset(outputPath, datasetPath, {numPoints, 3, 2}, H5T_NATIVE_FLOAT, "");
    return {firstErrorCode - 2, "does not hold integer pixel values"};
  case PatternCase::Unreadable:
    WritePatternDataset(outputPath, datasetPath, {numPoints, 3, 2}, H5T_NATIVE_UINT8, missingExternalPath.string());
    return {firstErrorCode - 3, "could not be read"};
  }
  return {0, ""};
}

void CheckPatternMessage(const std::string& message, const std::string& className, const std::string& datasetName, const std::string& scanName, const std::filesystem::path& filePath,
                         const std::string& expectedText)
{
  INFO(message);
  REQUIRE(message.find(className + " Error: The '" + datasetName + "' dataset of scan '" + scanName + "' in file '" + filePath.string() + "'") != std::string::npos);
  REQUIRE(message.find(expectedText) != std::string::npos);
}
} // namespace

TEST_CASE("ebsdlib::H5PatternDatasetTest::H5EspritReaderValidatesRawPatterns", "[EbsdLib][H5PatternDatasetTest]")
{
  auto patternCase = GENERATE(PatternCase::Valid, PatternCase::Rank2, PatternCase::FewerPatterns, PatternCase::FloatingPoint, PatternCase::Unreadable);
  const std::filesystem::path outputPath = std::filesystem::path(ebsdlib::unit_test::k_TestTempDir) / "H5PatternDatasetTest" / "H5Esprit_RawPatterns.h5";
  const std::string scanName = "Section_435";
  const auto [expectedError, expectedText] =
      PreparePatternFile(patternCase, ebsdlib::unit_test::H5EspritReaderTest::InputFile, outputPath, scanName + "/EBSD/Data/" + ebsdlib::H5Esprit::RawPatterns, 600 * 600, -90330);

  H5EspritReader::Pointer reader = H5EspritReader::New();
  reader->setFileName(outputPath.string());
  reader->setHDF5Path(scanName);
  reader->setReadPatternData(true);
  reader->setArraysToRead({ebsdlib::H5Esprit::MAD});
  const int err = reader->readFile();
  INFO(reader->getErrorMessage());
  if(expectedError == 0)
  {
    REQUIRE(err >= 0);
    std::array<int, 2> patternDims = {0, 0};
    reader->getPatternDims(patternDims);
    REQUIRE(patternDims[0] == 3);
    REQUIRE(patternDims[1] == 2);
    const uint8_t* patterns = reader->getPatternData();
    REQUIRE(patterns != nullptr);
    // File tuples 30 and 31 are swapped in this scan, so cell 30 holds the pattern of file tuple 31.
    REQUIRE(patterns[30 * 6] == 31);
    REQUIRE(patterns[31 * 6 + 5] == 30);
  }
  else
  {
    REQUIRE(err == expectedError);
    REQUIRE(reader->getErrorCode() == expectedError);
    CheckPatternMessage(reader->getErrorMessage(), "H5EspritReader", ebsdlib::H5Esprit::RawPatterns, scanName, outputPath, expectedText);
    REQUIRE(reader->getPatternData() == nullptr);
  }
  reader = H5EspritReader::NullPointer();
  std::filesystem::remove(outputPath);
}

TEST_CASE("ebsdlib::H5PatternDatasetTest::H5OIMReaderValidatesPattern", "[EbsdLib][H5PatternDatasetTest]")
{
  auto patternCase = GENERATE(PatternCase::Valid, PatternCase::Rank2, PatternCase::FewerPatterns, PatternCase::FloatingPoint, PatternCase::Unreadable);
  const std::filesystem::path outputPath = std::filesystem::path(ebsdlib::unit_test::k_TestTempDir) / "H5PatternDatasetTest" / "EdaxOIM_Pattern.h5";
  const std::string scanName = "Scan_1";
  const auto [expectedError, expectedText] =
      PreparePatternFile(patternCase, ebsdlib::unit_test::AngImportTest::EdaxOIMH5File, outputPath, scanName + "/EBSD/Data/" + ebsdlib::Ang::PatternData, 186 * 151, -90430);

  H5OIMReader::Pointer reader = H5OIMReader::New();
  reader->setFileName(outputPath.string());
  reader->setHDF5Path(scanName);
  reader->setReadPatternData(true);
  const int err = reader->readFile();
  INFO(reader->getErrorMessage());
  if(expectedError == 0)
  {
    REQUIRE(err >= 0);
    std::array<int, 2> patternDims = {0, 0};
    reader->getPatternDims(patternDims);
    REQUIRE(patternDims[0] == 3);
    REQUIRE(patternDims[1] == 2);
    const uint8_t* patterns = reader->getPatternData();
    REQUIRE(patterns != nullptr);
    REQUIRE(patterns[5] == 0);
    REQUIRE(patterns[6] == 1);
  }
  else
  {
    // H5OIMReader::readFile() reports every readData() failure as -900022 and keeps the readData() message.
    REQUIRE(err == -900022);
    CheckPatternMessage(reader->getErrorMessage(), "H5OIMReader", ebsdlib::Ang::PatternData, scanName, outputPath, expectedText);
    REQUIRE(reader->getPatternData() == nullptr);
  }
  reader = H5OIMReader::NullPointer();
  std::filesystem::remove(outputPath);
}
