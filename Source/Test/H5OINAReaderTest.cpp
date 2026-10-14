#include <catch2/catch.hpp>

#include "H5Support/H5Lite.h"
#include "H5Support/H5ScopedSentinel.h"

#include "EbsdLib/IO/HKL/CtfConstants.h"
#include "EbsdLib/IO/HKL/H5OINAReader.h"
#include "EbsdLib/Test/EbsdLibTestFileLocations.h"

#include "UnitTestSupport.hpp"

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

using namespace ebsdlib;

namespace
{
const std::string k_ScanName = "1";

template <typename T>
void RequireArraysEqual(const H5OINAReader::Pointer& expected, const H5OINAReader::Pointer& actual, const std::string& name, size_t count)
{
  const auto* e = static_cast<const T*>(expected->getPointerByName(name));
  const auto* a = static_cast<const T*>(actual->getPointerByName(name));
  DREAM3D_REQUIRE_VALID_POINTER(e)
  DREAM3D_REQUIRE_VALID_POINTER(a)
  size_t mismatches = 0;
  for(size_t i = 0; i < count; i++)
  {
    if(e[i] != a[i])
    {
      mismatches++;
    }
  }
  DREAM3D_REQUIRED(mismatches, ==, 0)
}

H5OINAReader::Pointer ReadScan(const std::string& path, bool selective = false)
{
  H5OINAReader::Pointer reader = H5OINAReader::New();
  reader->setFileName(path);
  reader->setHDF5Path(k_ScanName);
  reader->setReadPatternData(false);
  if(selective)
  {
    reader->setArraysToRead({ebsdlib::H5OINA::Euler});
    reader->readAllArrays(false);
  }
  const int err = reader->readFile();
  DREAM3D_REQUIRED(err, >=, 0)
  return reader;
}
} // namespace

TEST_CASE("ebsdlib::H5OINAReaderTest::OutOfOrderTuplesArePlacedByPosition", "[EbsdLib][H5OINAReaderTest]")
{
  const bool selective = GENERATE(false, true);
  H5OINAReader::Pointer ordered = ReadScan(ebsdlib::unit_test::H5OINAReaderTest::OrderedFile);
  H5OINAReader::Pointer shuffled = ReadScan(ebsdlib::unit_test::H5OINAReaderTest::OutOfOrderFile, selective);

  const size_t numCols = static_cast<size_t>(ordered->getXCells());
  const size_t numRows = static_cast<size_t>(ordered->getYCells());
  DREAM3D_REQUIRED(numCols, ==, 25)
  DREAM3D_REQUIRED(numRows, ==, 25)
  const size_t count = numCols * numRows;

  RequireArraysEqual<float>(ordered, shuffled, ebsdlib::H5OINA::X, count);
  RequireArraysEqual<float>(ordered, shuffled, ebsdlib::H5OINA::Y, count);
  RequireArraysEqual<float>(ordered, shuffled, ebsdlib::H5OINA::Euler, count * 3);
  RequireArraysEqual<float>(ordered, shuffled, ebsdlib::H5OINA::MeanAngularDeviation, count);
  RequireArraysEqual<uint8_t>(ordered, shuffled, ebsdlib::H5OINA::Phase, count);
  RequireArraysEqual<uint8_t>(ordered, shuffled, ebsdlib::H5OINA::BandContrast, count);
  RequireArraysEqual<uint8_t>(ordered, shuffled, ebsdlib::H5OINA::BandSlope, count);
  RequireArraysEqual<uint8_t>(ordered, shuffled, ebsdlib::H5OINA::Bands, count);
  RequireArraysEqual<uint8_t>(ordered, shuffled, ebsdlib::H5OINA::Error, count);

  // The ordered file itself must be raster ordered: X advances by X Step along each row.
  const auto* x = static_cast<const float*>(ordered->getPointerByName(ebsdlib::H5OINA::X));
  const float xStep = ordered->getXStep();
  size_t misplaced = 0;
  for(size_t i = 0; i < count; i++)
  {
    if(std::llround((x[i] - x[0]) / xStep) != static_cast<long long>(i % numCols))
    {
      misplaced++;
    }
  }
  DREAM3D_REQUIRED(misplaced, ==, 0)
}

TEST_CASE("ebsdlib::H5OINAReaderTest::MalformedGridIsRejected", "[EbsdLib][H5OINAReaderTest]")
{
  const std::filesystem::path outputPath = std::filesystem::path(ebsdlib::unit_test::k_TestTempDir) / "H5OINA_Malformed.h5oina";
  std::filesystem::copy_file(ebsdlib::unit_test::H5OINAReaderTest::OrderedFile, outputPath, std::filesystem::copy_options::overwrite_existing);
  int expectedError = -101103;
  std::string datasetName = ebsdlib::H5OINA::X;
  {
    const hid_t fileId = H5Fopen(outputPath.string().c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
    REQUIRE(fileId >= 0);
    H5Support::H5ScopedFileSentinel sentinel(fileId, true);
    SECTION("Duplicate position")
    {
      std::vector<float> x;
      REQUIRE(H5Support::H5Lite::readVectorDataset(fileId, "1/EBSD/Data/X", x) >= 0);
      x[1] = x[0];
      const hid_t datasetId = H5Dopen2(fileId, "1/EBSD/Data/X", H5P_DEFAULT);
      REQUIRE(datasetId >= 0);
      const herr_t writeError = H5Dwrite(datasetId, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, x.data());
      REQUIRE(H5Dclose(datasetId) >= 0);
      REQUIRE(writeError >= 0);
    }
    SECTION("Truncated dataset")
    {
      datasetName = GENERATE(ebsdlib::H5OINA::X, ebsdlib::H5OINA::Y, ebsdlib::H5OINA::Euler, ebsdlib::H5OINA::MeanAngularDeviation, ebsdlib::H5OINA::BandContrast, ebsdlib::H5OINA::BandSlope,
                             ebsdlib::H5OINA::Bands, ebsdlib::H5OINA::Error, ebsdlib::H5OINA::Phase);
      const std::string datasetPath = "1/EBSD/Data/" + datasetName;
      REQUIRE(H5Ldelete(fileId, datasetPath.c_str(), H5P_DEFAULT) >= 0);
      const std::vector<hsize_t> dims = {624};
      const std::vector<float> values(624, 0.0f);
      REQUIRE(H5Support::H5Lite::writeVectorDataset(fileId, datasetPath, dims, values) >= 0);
      expectedError = -101100;
    }
  }
  H5OINAReader::Pointer reader = H5OINAReader::New();
  reader->setFileName(outputPath.string());
  reader->setHDF5Path(k_ScanName);
  reader->setReadPatternData(false);
  REQUIRE(reader->readFile() == expectedError);
  REQUIRE(reader->getErrorCode() == expectedError);
  const std::string message = reader->getErrorMessage();
  REQUIRE(message.find("H5OINAReader Error:") == 0);
  REQUIRE(message.find(datasetName) != std::string::npos);
  REQUIRE(message.find("25 x 25") != std::string::npos);
  if(expectedError == -101103)
  {
    REQUIRE(message.find("tuple 1") != std::string::npos);
  }
  std::filesystem::remove(outputPath);
}
